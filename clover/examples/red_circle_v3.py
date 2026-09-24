from clover import long_callback, srv
from std_srvs.srv import Trigger
import rospy
import math
import cv2
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from cv_bridge import CvBridge
import numpy as np
import tf2_ros
import tf2_geometry_msgs

rospy.init_node('cv', disable_signals=True)
bridge = CvBridge()

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)

# This line is new!!!
release = rospy.ServiceProxy('simple_offboard/release', Trigger)

land = rospy.ServiceProxy('land', Trigger)

def wait_arrival(tolerance=0.1, time=1.0, telem=get_telemetry(frame_id='navigate_target')):
    while not (telem.x ** 2 + telem.y ** 2 + telem.z ** 2) ** 0.5 < tolerance:
       telem = get_telemetry(frame_id='navigate_target')
    rospy.sleep(time)

camera_info = rospy.wait_for_message('main_camera/camera_info', CameraInfo)
camera_matrix = np.float64(camera_info.K).reshape(3, 3)
distortion = np.float64(camera_info.D).flatten()
def img_xy_to_point(xy, dist):
	xy = cv2.undistortPoints(xy, camera_matrix, distortion, P=camera_matrix)[0][0]
	xy -= camera_info.width // 2, camera_info.height // 2
	fx = camera_matrix[0, 0]
	fy = camera_matrix[1, 1]
	return Point(x=xy[0] * dist / fx, y=xy[1] * dist / fy, z=dist)

xy = None
def get_center_of_mass(mask):
    global xy
    M = cv2.moments(mask)
    if M['m00'] == 0:
        return xy
    xy = M['m10'] // M['m00'], M['m01'] // M['m00']
    return xy

time_found = None
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
@long_callback
def image_callback(msg):
    img_hsv = cv2.cvtColor(bridge.imgmsg_to_cv2(msg, 'bgr8'), cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(img_hsv, (0, 150, 150), (15, 255, 255))
    mask2 = cv2.inRange(img_hsv, (160, 150, 150), (180, 255, 255))
    mask = cv2.bitwise_or(mask1, mask2)

    xy = get_center_of_mass(mask)
    if xy is None:
        move_to_next_search_point()
        return

    global time_found
    time_found = time_found or rospy.get_time()
    if rospy.get_time() - time_found > 20:
        # global image_sub
        # image_sub.unregister()

        # This line is new!!!
        release()
        
        wait_arrival(time=5)
        navigate(x=origin.x, y=origin.y, z=0.5, yaw=float('nan'), speed=1, frame_id='map')
        wait_arrival(time=2)
        land()
        return

    altitude = get_telemetry('terrain').z
    target = PointStamped(msg.header, img_xy_to_point(xy, altitude))
    setpoint = tf_buffer.transform(target, 'map', timeout=rospy.Duration(0.2))
    set_position(x=setpoint.point.x, y=setpoint.point.y, z=float('nan'), yaw=float('nan'), frame_id=setpoint.header.frame_id)

x_range = np.round(np.linspace(0 - 6 / 2, 0 + 6 / 2, int(6/.33) + 1),2)
y_range = np.round(np.linspace(0 - 6 / 2, 0 + 6 / 2, int(6/1) + 1),2)
search_pattern = ((x, y) for i, y in enumerate(y_range)
    for x in (x_range if i % 2 == 0 else reversed(x_range)))
origin = get_telemetry()
def move_to_next_search_point():
    try:
        cur_target = next(search_pattern)
        set_position(x=cur_target[0], y=cur_target[1], z=float('nan'), yaw=float('nan'), frame_id='map')
        wait_arrival(tolerance=0.66, time=-1)
    except StopIteration:
        navigate(x=origin.x, y=origin.y, z=0.5, yaw=float('nan'), speed=1, frame_id='map')
        wait_arrival(time=2)
        land()
        rospy.signal_shutdown("Circle not found.")

def navigate_wait(x=0, y=0, z=0, speed=0.5, frame_id='body', auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=float('nan'), speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        raise Exception(res.message)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < 0.2:
            return
        rospy.sleep(0.2)


navigate_wait(z=2, frame_id='body', auto_arm=True)

pattern_start = next(search_pattern)
navigate(x=pattern_start[0], y=pattern_start[1], z=float('nan'), speed=1)
wait_arrival()

image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback, queue_size=1)
rospy.spin()
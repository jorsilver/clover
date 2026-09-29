from clover import srv
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

def get_center_of_mass(mask):
    M = cv2.moments(mask)
    return (M['m10'] // M['m00'], M['m01'] // M['m00']) if M['m00'] != 0 else None

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

def detect_target():
    msg = rospy.wait_for_message('main_camera/image_raw_throttled', Image)
    img_hsv = cv2.cvtColor(bridge.imgmsg_to_cv2(msg, 'bgr8'), cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(img_hsv, (0, 150, 150), (15, 255, 255))
    mask2 = cv2.inRange(img_hsv, (160, 150, 150), (180, 255, 255))
    mask = cv2.bitwise_or(mask1, mask2)
    return msg, get_center_of_mass(mask)

def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)

def navigate_wait(x=0, y=0, z=0, speed=0.5, frame_id='body', auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=float('nan'), speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        raise Exception(res.message)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < 0.2:
            return
        rospy.sleep(0.2)

def search_area(cx, cy, size):
    x_range = np.round(np.linspace(cx - size / 2, cx + size / 2, int(size/.33) + 1),2)
    y_range = np.round(np.linspace(cy - size / 2, cy + size / 2, int(size/1) + 1),2)
    pattern = ((x, y) for i, y in enumerate(y_range)
        for x in (x_range if i % 2 == 0 else reversed(x_range)))
    for tx, ty in pattern:
        if rospy.is_shutdown():
            return False
        set_position(x=tx, y=ty, z=float('nan'), yaw=float('nan'), frame_id='map')
        wait_arrival(tolerance=0.66, time=-1)
        if detect_target()[1] is not None:
            return True
    return False

def track_target(seconds):
    start = rospy.get_time()
    while rospy.get_time() - start < seconds and not rospy.is_shutdown():
        msg, xy = detect_target()
        if xy is None:
            continue
        point = img_xy_to_point(xy, get_telemetry('terrain').z)
        setpoint = tf_buffer.transform(PointStamped(msg.header, point), 'map', timeout=rospy.Duration(0.2))
        set_position(x=setpoint.point.x, y=setpoint.point.y, z=float('nan'), yaw=float('nan'), frame_id=setpoint.header.frame_id)

launch_origin = get_telemetry(frame_id='map')
def return_to_launch(z):
    navigate_wait(x=launch_origin.x, y=launch_origin.y, z=z, speed=1, frame_id='map')


_b('cs_takeoff')
navigate_wait(z=2, frame_id='body', auto_arm=True)
_b('cs_if')
if search_area(0, 0, 6):
    _b('cs_track')
    track_target(5)
    _b('cs_land')
    land_wait()
else:
    _b('cs_rtl')
    return_to_launch(0.5)
    _b('cs_land2')
    land_wait()

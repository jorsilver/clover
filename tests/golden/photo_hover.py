from clover import srv
from std_srvs.srv import Trigger
import rospy
import math
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from datetime import datetime

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

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

def hover(alt):
    _telem = get_telemetry(frame_id='map')
    navigate_wait(x=_telem.x, y=_telem.y, z=alt, frame_id='map')

def take_photo():
    _bridge = CvBridge()
    _img = _bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    _name = 'photo_%s.jpg' % datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    cv2.imwrite(_name, _img)
    rospy.loginfo('Saved photo: %s' % _name)
    return _name


_b('ph_takeoff')
navigate_wait(z=1, frame_id='body', auto_arm=True)
_b('ph_hover')
hover(2.5)
_b('ph_photo')
take_photo()
_b('ph_land')
land_wait()

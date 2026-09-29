from clover import srv
from std_srvs.srv import Trigger
import rospy
import math

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

def patrol_route(waypoints, repeat):
    for _ in range(int(repeat)):
        for wp in waypoints:
            if rospy.is_shutdown():
                return
            navigate_wait(x=wp[0], y=wp[1], z=wp[2], frame_id='map')


_b('pt_takeoff')
navigate_wait(z=2, frame_id='body', auto_arm=True)
_b('pt_route')
patrol_route([(2, 0, 2), (2, 2, 2), (0, 2, 2)], 2)
_b('pt_land')
land_wait()

# Copter clover class for nti LA olymp
import rospy
from clover import srv
from std_srvs.srv import Trigger
import math


class NavPoint(object):
    """
    Navigation Point class
    """
    def __init__(self, x, y, z=1, endpoint=False, frame='aruco_map'):
        self.x = x
        self.y = y
        self.z = z
        # If True, snapshot will not taken
        self.endpoint = endpoint
        self.frame = frame


class Copter:
    """
    Clover copter class
    """
    def __init__(self):
        # rospy.init_node('flight')
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
        self.set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
        self.set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
        self.set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
        self.set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
        self.land = rospy.ServiceProxy('land', Trigger)

    def navigate_wait(self, x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
        """
        Navigate to point and wait, until it reaches
        :param x: x
        :param y: y
        :param z: y
        :param yaw:
        :param speed:
        :param frame_id:
        :param auto_arm:
        :param tolerance:
        :return:
        """
        self.navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

        while not rospy.is_shutdown():
            telem = self.get_telemetry(frame_id='navigate_target')
            if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
                break
            rospy.sleep(0.2)

    def navigate_to_point(self, point):
        """
        Navigate to NavPoint obj and auto takeoff/land
        :param point: NavPoint obj
        :return:
        """
        telem = self.get_telemetry(frame_id='map')
        if point.endpoint and telem.z < 0.2:
            self.takeoff()
        self.navigate_wait(point.x, point.y, point.z, frame_id=point.frame)
        if point.endpoint and telem.z > 0.2:
            self.landing()
        

    def takeoff(self, height=1):
        """
        takeoff function
        :param height: takeoff height
        :return:
        """
        self.navigate_wait(z=height, frame_id='body', auto_arm=True)

    def landing(self):
        """
        Landing command
        :return:
        """
        self.land()
        while self.get_telemetry().armed:
            rospy.sleep(0.2)

    def wait_arrival(self, tolerance=0.2):
        """
        wait for navigating to corresponding point
        :return:
        """
        while not rospy.is_shutdown():
            telem = self.get_telemetry(frame_id='navigate_target')
            if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
                break
            rospy.sleep(0.2)
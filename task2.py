# -*- coding: utf-8 -*-
from copter import Copter, NavPoint
# from Objects_detector import ObjectsDetector
import rospy
from std_msgs.msg import String, Bool

CURRENT_POINT = 0
LOG_PATH = "/home/clover/Documents/CopterNTI/log.txt"

# We have choosen height 0.5m, because with other height copter has seen two or more markers in camera
Points = [NavPoint(0, 0, 0.5, endpoint=True), NavPoint(0, 2.5, 0.5), NavPoint(3.5, 0.5, 0.5), NavPoint(2, 1.5, 0.5), NavPoint(3.5, 3.5, 0.5), NavPoint(0, 0, endpoint=True)]

def logging(data):
    info = "Point #" + str(CURRENT_POINT) + ", X coordinate " + str(Points[CURRENT_POINT].x) + ", Y coordinate " + str(Points[CURRENT_POINT].y) + ", Recognized Color " + data.data + "\n"
    with open(LOG_PATH, 'a') as f:
        f.write(info)
        print("Writed, " + info)


if __name__ == "__main__":
    rospy.init_node('flight_cont')
    with open(LOG_PATH, 'w') as f:
        f.write("---FLIGHT LOG---\n")
    start = rospy.Publisher('detector/start', Bool, queue_size=100)
    logger = rospy.Subscriber('detector/detected', String, logging)
    clover = Copter()
    for point in Points:
        clover.navigate_to_point(point)
        if not point.endpoint:
            # Sending command to recognize objects
            start.publish(True)
            rospy.sleep(5)
        CURRENT_POINT += 1

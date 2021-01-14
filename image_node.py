import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Bool
from Objects_detector import ObjectsDetector

START_CONVERSION = False


rospy.init_node('object_detector')
bridge = CvBridge()
detector = ObjectsDetector(debug_mode=True)
image_pub = rospy.Publisher('detector/debug', Image, queue_size=100)
objects_detected = rospy.Publisher('detector/detected', String, queue_size=100)


def flag_setter(data):
    """
    Changes START_CONVERSION flag
    :param data:
    :return:
    """
    global START_CONVERSION 
    START_CONVERSION = data
    rospy.loginfo("Starting conversation %s" % rospy.get_time())


def image_callback(data):
    """
    Detect objects
    :param data:
    :return:
    """
    global START_CONVERSION
    if START_CONVERSION:
        cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
        objects, frame = detector.get_objects(cv_image)
        print(objects)
        for obj in objects:
            rospy.loginfo("Detected " + obj.__class__.__name__ + " with color " + obj.get_color())
        START_CONVERSION = False
        objects_detected.publish(objects[0].get_color())
        image_pub.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))


if __name__ == "__main__":
    image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)
    start_conversion = rospy.Subscriber('detector/start', Bool, flag_setter)
    rospy.spin()

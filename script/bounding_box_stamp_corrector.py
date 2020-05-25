#!/usr/bin/env python

import rospy
from darknet_ros_msgs.msg import BoundingBoxes

bounding_boxes_topic = ""
pub = ""

def callback(boundingBoxes):
    global pub
    boundingBoxes.header = boundingBoxes.image_header
    pub.publish(boundingBoxes)

def darknet_ros_stamp_corrector_node():
    rospy.init_node('darknet_ros_stamp_corrector')

    global bounding_boxes_topic
    global pub

    # Get parameters
    bounding_boxes_topic = rospy.get_param("bounding_boxes_topic", "/darknet_ros/bounding_boxes")

    # Create bounding_boxes_republisher.
    rospy.Subscriber(bounding_boxes_topic, BoundingBoxes, callback)
    pub = rospy.Publisher(bounding_boxes_topic+"_correct_stamp", BoundingBoxes, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        darknet_ros_stamp_corrector_node()
    except rospy.ROSInterruptException:
        pass

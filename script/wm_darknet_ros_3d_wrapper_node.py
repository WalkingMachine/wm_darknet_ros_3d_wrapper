#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
from darknet_ros_msgs.msg import BoundingBoxes


def synchronisedCallback(depth, bounding_boxes):
    rospy.loginfo("I heard something")


def wm_darknet_ros_3d_wrapper_node():
    rospy.init_node('darknet_ros_3d_wrapper_node')

    # Get parameters
    depth_topic = rospy.get_param("depth_topic", "/camera/depth/image_raw")
    bounding_boxes_topic = rospy.get_param("bounding_boxes_topic", "/darknet_ros/bounding_boxes")
    synchroniser_buffer = rospy.get_param("synchroniser_buffer", 50)
    synchroniser_time_tolerance = rospy.get_param("synchroniser_time_tolerance", 0.5)

    # Create depth and bounding boxes subscribers.
    depth_sub = Subscriber(depth_topic, Image)
    bounding_boxes_sub = Subscriber(bounding_boxes_topic, BoundingBoxes)
    # Synchronize topics
    ts = ApproximateTimeSynchronizer([depth_sub, bounding_boxes_sub], synchroniser_buffer, synchroniser_time_tolerance)
    ts.registerCallback(synchronisedCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        wm_darknet_ros_3d_wrapper_node()
    except rospy.ROSInterruptException:
        pass

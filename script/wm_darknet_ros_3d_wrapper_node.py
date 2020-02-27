#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
from darknet_ros_msgs.msg import BoundingBoxes
from sara_msgs.msg import BoundingBoxes3D, BoundingBox3D
from geometry_msgs.msg import Point
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

bridge = CvBridge()

def synchronisedCallback(depth, bounding_boxes):

    debug="I heard something"
    debug+="\n    image stamp: "+str(depth.header.stamp)
    debug+="\n    boxes stamp: "+str(bounding_boxes.header.stamp)
    try:
        depth_image = bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
    except CvBridgeError, e:
        print e

    depth_array = np.array(depth_image, dtype=np.float32)

    for box in bounding_boxes.bounding_boxes:
        # Get the distance in (m)
        distance_median = np.median(depth_array[box.xmin:box.xmax, box.ymin:box.ymax])/1000
        debug+="\n    distance_median: "+str(distance_median)+"(m)"

    rospy.logdebug(debug)

    pose =

def wm_darknet_ros_3d_wrapper_node():
    rospy.init_node('darknet_ros_3d_wrapper_node')

    # Get parameters
    depth_topic = rospy.get_param("depth_topic", "/camera/depth/image_raw")
    bounding_boxes_topic = rospy.get_param("bounding_boxes_topic", "/darknet_ros/bounding_boxes")
    synchroniser_buffer = rospy.get_param("synchroniser_buffer", 50)
    synchroniser_time_tolerance = rospy.get_param("synchroniser_time_tolerance", 0.5)

    rospy.loginfo("wm_darknet_ros_3d_wrapper_node settings:\n    depth_topic: "+str(depth_topic)
    +"\n    bounding_boxes_topic: "+str(bounding_boxes_topic)
    +"\n    synchroniser_buffer: "+str(synchroniser_buffer)
    +"\n    synchroniser_time_tolerance: "+str(synchroniser_time_tolerance))


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

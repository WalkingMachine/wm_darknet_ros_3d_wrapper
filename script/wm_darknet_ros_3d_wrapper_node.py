#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
from darknet_ros_msgs.msg import BoundingBoxes
from sara_msgs.msg import BoundingBoxes3D, BoundingBox3D
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from tf.transformations import quaternion_from_euler
import math

bridge = CvBridge()

depth_topic = ""
bounding_boxes_topic = ""
synchroniser_buffer = ""
synchroniser_time_tolerance = ""
camera_fov_width = ""
camera_fov_height = ""



def synchronisedCallback(depth, bounding_boxes):

    global depth_topic
    global bounding_boxes_topic
    global synchroniser_buffer
    global synchroniser_time_tolerance
    global camera_fov_width
    global camera_fov_height



    debug="I heard something"
    debug+="\n    image stamp: "+str(depth.header.stamp)
    debug+="\n    boxes stamp: "+str(bounding_boxes.header.stamp)
    try:
        depth_image = bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
    except CvBridgeError, e:
        print e

    depth_array = np.array(depth_image, dtype=np.float32)

    boxes3D=BoundingBoxes3D()
    boxes3D.header = bounding_boxes.image_header

    for box in bounding_boxes.bounding_boxes:
        # Get the distance in (m)
        distance_median = np.median(depth_array[box.xmin:box.xmax, box.ymin:box.ymax])/1000
        debug+="\n    distance_median: "+str(distance_median)+"(m)"

        # Create the 3D box
        box3D = BoundingBox3D()
        box3D.className = box.Class
        box3D.probability = box.probability
        box3D.pose.orientation = quaternion_from_euler(0, 0, 0)

        # Get 2d center
        x = (box.xmax + box.xmin)/2
        y = (box.ymax + box.ymin)/2

        # Get pixel to rad ratio
        xratio = camera_fov_width/depth_array.shape[1]
        yratio = camera_fov_height/depth_array.shape[0]

        # Get the IRL angles from the camera center to the object
        ax = -(x - depth_array.shape[1]/2)*xratio
        ay = -(y - depth_array.shape[0]/2)*yratio
        aw = -(box.xmax - depth_array.shape[1]/2)*xratio
        ah = -(box.ymax - depth_array.shape[0]/2)*yratio

        # Convert the angeles and distance to x y z coordinates
        px = -distance_median * math.sin(ax)
        py = -distance_median * math.sin(ay)
        pz = distance_median * math.cos(ax)*math.cos(ay)
        pxwh = -distance_median * math.sin(aw)
        pywh = -distance_median * math.sin(ah)
        pzwh = distance_median * math.cos(aw)*math.cos(ah)

        # Place the coordinates into the pose.
        box3D.pose.position.x = px
        box3D.pose.position.y = py
        box3D.pose.position.z = pz

        print(str(box3D))

    rospy.logdebug(debug)


def wm_darknet_ros_3d_wrapper_node():
    rospy.init_node('darknet_ros_3d_wrapper_node')

    global depth_topic
    global bounding_boxes_topic
    global synchroniser_buffer
    global synchroniser_time_tolerance
    global camera_fov_width
    global camera_fov_height


    # Get parameters
    depth_topic = rospy.get_param("depth_topic", "/camera/depth/image_raw")
    bounding_boxes_topic = rospy.get_param("bounding_boxes_topic", "/darknet_ros/bounding_boxes")
    synchroniser_buffer = rospy.get_param("synchroniser_buffer", 50)
    synchroniser_time_tolerance = rospy.get_param("synchroniser_time_tolerance", 0.5)
    camera_fov_width = rospy.get_param("camera_fov_width", 1.012290966)
    camera_fov_height = rospy.get_param("camera_fov_height", 0.785398163397)

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

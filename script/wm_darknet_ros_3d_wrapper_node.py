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
from visualization_msgs.msg import MarkerArray, Marker
from matplotlib import pyplot as plt

bridge = CvBridge()

depth_topic = ""
bounding_boxes_topic = ""
synchroniser_buffer = ""
synchroniser_time_tolerance = ""
camera_fov_width = ""
camera_fov_height = ""
markerPublisher = ""
boxPublisher = ""


def synchronisedCallback(depth, bounding_boxes):
    rospy.loginfo("begin")
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

    # Create the lists
    boxes3D=BoundingBoxes3D()
    boxes3D.header = bounding_boxes.header
    markers=MarkerArray()

    i = 0
    for box in bounding_boxes.bounding_boxes:
        # Get the distance in (m)

        # dimx, dimy
        # distance_median = np.median(depth_array[box.xmin+5:box.xmax-5, box.ymin+5:box.ymax-5])#/1000 # (m)
        # debug+="\n    distance_median: "+str(distance_median)+"(m)"


        mask = np.zeros(depth_image.shape[:2],np.uint8)
        bgdModel = np.zeros((1,65),np.float64)
        fgdModel = np.zeros((1,65),np.float64)

        rect = (50,50,450,290)
        cv2.grabCut(depth_image,mask,rect,bgdModel,fgdModel,5,cv2.GC_INIT_WITH_RECT)

        mask2 = np.where((mask==2)|(mask==0),0,1).astype('uint8')
        depth_image = depth_image*mask2[:,:,np.newaxis]

        plt.imshow(depth_image),plt.colorbar(),plt.show()








        # Create the 3D box
        box3D = BoundingBox3D()
        box3D.className = box.Class
        box3D.probability = box.probability

        # Create the marker
        marker = Marker()
        marker.header = bounding_boxes.header
        marker.ns = "bounding_box_3d"
        marker.id = i
        marker.type = marker.CUBE
        marker.color.r = 1
        marker.color.g = 1
        marker.color.b = 1
        marker.color.a = 0.8
        # marker.lifetime = rospy.Time(2)
        marker.frame_locked = True

        # Set the orientation
        quat = quaternion_from_euler(0, 0, 0)
        box3D.pose.orientation.x = quat[0]
        box3D.pose.orientation.y = quat[1]
        box3D.pose.orientation.z = quat[2]
        box3D.pose.orientation.w = quat[3]

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
        box3D.pose.position.x = float(-px)
        box3D.pose.position.y = float(py)
        box3D.pose.position.z = float(pz)

        # Set marker pose
        marker.pose = box3D.pose
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1 # TODO calculer la taille

        # Add the box to the lists
        boxes3D.boundingBoxes.append(box3D)
        markers.markers.append(marker)

        i += 1


    rospy.logdebug(debug)
    boxPublisher.publish(boxes3D)
    markerPublisher.publish(markers)
    rospy.loginfo("end")


def wm_darknet_ros_3d_wrapper_node():
    rospy.init_node('darknet_ros_3d_wrapper_node')

    global depth_topic
    global bounding_boxes_topic
    global synchroniser_buffer
    global synchroniser_time_tolerance
    global camera_fov_width
    global camera_fov_height
    global markerPublisher
    global boxPublisher


    # Get parameters
    depth_topic = rospy.get_param("depth_topic", "/head_xtion/depth/image_raw")
    bounding_boxes_topic = rospy.get_param("bounding_boxes_topic", "/darknet_ros/bounding_boxes")
    synchroniser_buffer = rospy.get_param("synchroniser_buffer", 60)
    synchroniser_time_tolerance = rospy.get_param("synchroniser_time_tolerance", 5.5)
    camera_fov_width = rospy.get_param("camera_fov_width", 1.012290966)
    camera_fov_height = rospy.get_param("camera_fov_height", 0.785398163397)

    rospy.loginfo("wm_darknet_ros_3d_wrapper_node settings:\n    depth_topic: "+str(depth_topic)
    +"\n    bounding_boxes_topic: "+str(bounding_boxes_topic)
    +"\n    synchroniser_buffer: "+str(synchroniser_buffer)
    +"\n    synchroniser_time_tolerance: "+str(synchroniser_time_tolerance))

    # Create depth and bounding boxes subscribers.
    depth_sub = Subscriber(depth_topic, Image)
    bounding_boxes_sub = Subscriber(bounding_boxes_topic+"_correct_stamp", BoundingBoxes)
    # Synchronize topics
    ts = ApproximateTimeSynchronizer([depth_sub, bounding_boxes_sub], synchroniser_buffer, synchroniser_time_tolerance)
    ts.registerCallback(synchronisedCallback)

    # Create the publishers
    boxPublisher = rospy.Publisher('~/boxes3d', BoundingBoxes3D, queue_size=10)
    markerPublisher = rospy.Publisher('~/boxes3d_markers', MarkerArray, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        wm_darknet_ros_3d_wrapper_node()
    except rospy.ROSInterruptException:
        pass

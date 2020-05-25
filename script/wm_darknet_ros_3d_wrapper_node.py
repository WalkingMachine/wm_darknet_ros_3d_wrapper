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
from  scipy import ndimage


# Declare the global variables
depth_topic = ""
bounding_boxes_topic = ""
synchroniser_buffer = ""
synchroniser_time_tolerance = ""
camera_fov_width = ""
camera_fov_height = ""
markerPublisher = ""
boxPublisher = ""
histogramPrecision = 15 # (bins/m) 15 bins result in 6.7cm per bins
display_gui = False
bridge = CvBridge()


# Get the distance in (m) and coordinates in pixel. Using Median.
def getDistanceMode(depth_array, box):
    # Get the global variables
    global histogramPrecision
    global display_gui

    width = box.ymax-box.ymin
    height = box.xmax-box.xmin

    mask = depth_array[box.ymin:box.ymax, box.xmin:box.xmax]
    mask[np.isnan(mask)] = 0
    # plt.imshow(depth_array),plt.colorbar(),plt.show()
    # plt.imshow(mask),plt.colorbar(),plt.show()

    data = mask.ravel()
    try:
        bincount = np.bincount((data*histogramPrecision).astype(int))
    except:
        print("errored data")
        print(data)

    bincount[0]=0

    distance_mode = bincount.argmax()/histogramPrecision

    mask[mask > distance_mode+2/histogramPrecision] = 0
    mask[mask < distance_mode-2/histogramPrecision] = 0

    center_of_mass = ndimage.measurements.center_of_mass(mask)
    y, x = center_of_mass
    x += box.xmin
    y += box.ymin

    distance = np.mean(data[((data < distance_mode+1/histogramPrecision)
    & (data > distance_mode-1/histogramPrecision))])

    return distance, x, y


def synchronisedCallback(depth, bounding_boxes):
    # Get the global variables
    global depth_topic
    global bounding_boxes_topic
    global synchroniser_buffer
    global synchroniser_time_tolerance
    global camera_fov_width
    global camera_fov_height
    global bridge


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

    X = []
    Y = []
    D = []


    if display_gui:
        plt.imshow(depth_array)
        plt.colorbar()

    for box in bounding_boxes.bounding_boxes:

        # Get the distance in (m) and coordinates in pixel
        distance, x, y = getDistanceMode(depth_array, box)

        # Display the gui if needed
        if display_gui:
            plt.scatter(x, y, c='r', s=10)
            plt.text(x, y-2, "{:.2f}m".format(distance), fontsize=9)

        # Create the 3D box
        box3D = BoundingBox3D()
        box3D.className = box.Class
        box3D.probability = box.probability

        # Create the marker
        marker = Marker()
        marker.header = bounding_boxes.header
        marker.ns = "bounding_box_3d"
        marker.id = box.id
        marker.type = marker.CUBE
        marker.color.r = 1
        marker.color.g = 1
        marker.color.b = 1
        marker.color.a = 0.8
        marker.frame_locked = True

        # Set the orientation
        quat = quaternion_from_euler(0, 0, 0)
        box3D.pose.orientation.x = quat[0]
        box3D.pose.orientation.y = quat[1]
        box3D.pose.orientation.z = quat[2]
        box3D.pose.orientation.w = quat[3]

        # Get pixel to rad ratio
        xratio = camera_fov_width/depth_array.shape[1]
        yratio = camera_fov_height/depth_array.shape[0]

        # Get the IRL angles from the camera center to the object
        ax = (x - depth_array.shape[1]/2)*xratio
        ay = -(y - depth_array.shape[0]/2)*yratio
        aw = -(box.ymax - depth_array.shape[1]/2)*xratio
        ah = -(box.xmax - depth_array.shape[0]/2)*yratio

        # Convert the angeles and distance to x y z coordinates
        px = -distance * math.sin(ax)
        py = -distance * math.sin(ay)
        pz = distance * math.cos(ax)*math.cos(ay)
        pxwh = -distance * math.sin(aw)
        pywh = -distance * math.sin(ah)
        pzwh = distance * math.cos(aw)*math.cos(ah)

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



    if display_gui:
        plt.show()

    rospy.logdebug(debug)
    boxPublisher.publish(boxes3D)
    markerPublisher.publish(markers)

def wm_darknet_ros_3d_wrapper_node():
    # Initiate the ros node
    rospy.init_node('darknet_ros_3d_wrapper_node')

    # Get the global variables
    global depth_topic
    global bounding_boxes_topic
    global synchroniser_buffer
    global synchroniser_time_tolerance
    global camera_fov_width
    global camera_fov_height
    global markerPublisher
    global boxPublisher
    global histogramPrecision
    global display_gui


    # Get parameters
    depth_topic = rospy.get_param("~depth_topic", "/head_xtion/depth/image_raw")
    bounding_boxes_topic = rospy.get_param("~bounding_boxes_topic", "/darknet_ros/bounding_boxes")
    synchroniser_buffer = rospy.get_param("~synchroniser_buffer", 60.0)
    synchroniser_time_tolerance = rospy.get_param("~synchroniser_time_tolerance", 5.5)
    camera_fov_width = rospy.get_param("~camera_fov_width", 1.012290966)
    camera_fov_height = rospy.get_param("~camera_fov_height", 0.785398163397)
    histogramPrecision = rospy.get_param("~histogramPrecision", 15.0)
    display_gui = rospy.get_param("~display_gui", False)

    rospy.loginfo("wm_darknet_ros_3d_wrapper_node settings:\n    depth_topic: "+str(depth_topic)
    +"\n    bounding_boxes_topic: "+str(bounding_boxes_topic)
    +"\n    synchroniser_buffer: "+str(synchroniser_buffer)
    +"\n    synchroniser_time_tolerance: "+str(synchroniser_time_tolerance)
    +"\n    camera_fov_width: "+str(camera_fov_width)
    +"\n    camera_fov_height: "+str(camera_fov_height)
    +"\n    histogramPrecision: "+str(histogramPrecision)
    +"\n    display_gui: "+str(display_gui))

    # Create depth and bounding boxes subscribers.
    depth_sub = Subscriber(depth_topic, Image)
    bounding_boxes_sub = Subscriber(bounding_boxes_topic+"_correct_stamp", BoundingBoxes)
    # Synchronize topics
    ts = ApproximateTimeSynchronizer([depth_sub, bounding_boxes_sub], synchroniser_buffer, synchroniser_time_tolerance)
    ts.registerCallback(synchronisedCallback)

    # Create the publishers
    boxPublisher = rospy.Publisher('~boxes3d', BoundingBoxes3D, queue_size=10)
    markerPublisher = rospy.Publisher('~boxes3d_markers', MarkerArray, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        wm_darknet_ros_3d_wrapper_node()
    except rospy.ROSInterruptException:
        pass

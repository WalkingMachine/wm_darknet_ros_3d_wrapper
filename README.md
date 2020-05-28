# Description
This repo contains code allowing to convert the 2D output of darknet into 3D boxes.

# Dependencies
* numpy
* sensor_msgs
* darknet_ros_msgs
* sara_msgs
* cv2
* matplotlib
* scipy

# Launchfile
*	wm_darknet_ros_3d_wrapper.launch
    * Start the 3D conversion stack.

### Parameters
  * darknet_ros_3d_wrapper_node/depth_topic
      * Select the topic from witch to get the depth images
  * darknet_ros_3d_wrapper_node/bounding_boxes_topic
      * Select the topic from witch to get the bounding boxes
  * darknet_ros_3d_wrapper_node/synchroniser_buffer
      * Set the buffer size for the topic synchronisation argorythm.
  * darknet_ros_3d_wrapper_node/synchroniser_time_tolerance
      * Set the time tolerance for the topic synchronisation argorythm.
  * darknet_ros_3d_wrapper_node/camera_fov_width
      * Set the angular width of the camera
  * darknet_ros_3d_wrapper_node/camera_fov_height
      * Set the angular height of the camera
  * darknet_ros_3d_wrapper_node/histogramPrecision
      * Set the number of divisions per meter used for the histogram selector
  * darknet_ros_3d_wrapper_node/display_gui
      * Render a plot of the current segmentation. (for debug purpose only)


# Nodes
## bounding_box_stamp_corrector 
Adjust the darknet output to have the desired timestamp.

### Topics provided
* Subscriber
    * /darknet_ros/bounding_boxes (darknet_ros_msgs/BoundingBoxes)
        * List of bounding boxes from darknet
* Publisher
    * /darknet_ros/bounding_boxes_correct_stamp (darknet_ros_msgs/BoundingBoxes)
        * List of bounding boxes with adjusted timestamp
        
## darknet_ros_3d_wrapper_node
Perform the 3D convertions of the bounding boxes.

### Topics provided
* Subscriber
    * /head_xtion/depth/image_raw (sensor_msgs/Image)
        * Depth image from the same camera used for darknet
    * /darknet_ros/bounding_boxes_correct_stamp (darknet_ros_msgs/BoundingBoxes)
        * List of bounding boxes from darknet
* Publisher
    * /darknet_ros_3d_wrapper_node/boxes3d (sara_msgs/BoundingBoxes3D)
        * List of 3d bounding boxes
    * /darknet_ros_3d_wrapper_node/boxes3d_markers (visualization_msgs/MarkerArray)
        * List of markers for rviz
        

# semmapping

Creates semantic maps from RGB-D camera images.

## Related packages

[hypermap_msgs](https://github.com/Eruvae/hypermap_msgs): Provides semantic messages published by this framework. You can use these messages to create a Hypermap. For more information, see the [hypermap package](https://github.com/Eruvae/hypermap).

[hypermap_rviz_plugin](https://github.com/Eruvae/hypermap_rviz_plugin): Contains plugin to visualize the generated semantic maps.

[darknet_ros](https://github.com/leggedrobotics/darknet_ros): YOLO wrapper for ROS, which can be used for object detection. You need at least the message package to compile this package.

[yolact_ros](https://github.com/Eruvae/yolact_ros): Yolact wrapper for ROS, which can be used for instance segmentation. You need at least the [message package](https://github.com/Eruvae/yolact_ros_msgs) to compile this package.

## Parameters

- **point_cloud**: Point cloud topic used for segmentation. 
- **camera_info**: Camera info topic. Currently only used to publish observation area.
- **laser_scanner**: Laser scanner topic. Currently only used to publish observation area.

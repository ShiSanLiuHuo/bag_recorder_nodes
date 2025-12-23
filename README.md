# bag_recorder_nodes
是一个用于记录ros2 bag的节点

由于相机的分辨率过高，需使用intra-process通信

相机参数： 曝光30000 增益最高

## run
```
ros2 launch livox_ros2_driver livox_lidar_launch.py
ros2 launch launch.py
```

## 注意
jazzy 和humble的metadtata有所不同
可以使用[这个](https://github.com/NeonVector/rosbag_converter_jazzy2humble)转化
# bag_recorder_nodes
是一个用于记录ros2 bag的节点 ，包含camera_node和lidar_node,其中图像经过压缩后发布，lidar数据直接发布，减少bag的体积

由于相机的分辨率过高，需使用intra-process通信

相机参数： 曝光30000 增益最高

## run
```
ros2 launch livox_ros2_driver livox_lidar_launch.py
ros2 launch launch.py
```

## 压缩效果
43s的rosbag文件，压缩前为25.8GB，压缩后为443.2MB

## 注意
本代码在 jazzy 和 humble 下均测试通过
> camera_node中使用了cv_bridge
> 在 jazzy 下，cv_bridge 头文件为 `cv_bridge/cv_bridge.hpp`
> 在 humble 下，cv_bridge 头文件为 `cv_bridge/cv_bridge.h`

但 jazzy 和 humble 的 metadata 有所不同
可以使用[这个](https://github.com/NeonVector/rosbag_converter_jazzy2humble)转化
实际上可以直接播放`.db3`文件
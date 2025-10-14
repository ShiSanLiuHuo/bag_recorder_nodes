# bag_recorder_nodes
是一个用于记录ros2 bag的节点

**只适用于ros2 jazzy**

由于writer的api做了修改，不能再humble上使用

## run
```
ros2 run bag_recorder_nodes bag_recorder_nodes
```

## rosbag
在**jazzy**上录制的rosbag无法在**humble**上播放

解决方案：[源神启动](https://github.com/NeonVector/rosbag_converter_jazzy2humble)
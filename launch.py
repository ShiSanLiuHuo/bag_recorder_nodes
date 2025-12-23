import os
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from launch.actions import Shutdown

def generate_launch_description():
    # config = os.path.join('/home/phoenix/Desktop/phoenix_s26_lidar/config/config.yaml')

    bag_recorder_node = ComposableNodeContainer(
        name='bag_recorder_node',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        # executable='component_container',
        output='both',
        emulate_tty=True,
        on_exit=Shutdown(),
        # prefix=['gdbserver localhost:3000'],
        # arguments=["--ros-args", "--log-level", "debug"],
        
        composable_node_descriptions=[
            ComposableNode(
                package='bag_recorder',
                plugin='BagRecorderNode',
                name='bag_recorder_node',
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
              package='camera',
              plugin='sensor::CameraNode',
              name='camera_node',
              extra_arguments=[{"use_intra_process_comms": True}],
            #   parameters=[config]
            )
        ]
    )

    return launch.LaunchDescription([bag_recorder_node])
"""Zero-shot box pose estimator on a robot's gripper camera (MobileSAM from
external/, see scripts/setup_external.sh).

    ros2 launch vision box_pose.launch.py                      # on demand: ~/detect
    ros2 launch vision box_pose.launch.py rate_hz:=1.0         # continuous
    ros2 service call /robot_1/box_pose_estimator/detect std_srvs/srv/Trigger
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument('namespace', default_value='robot_1', description='Robot whose gripper camera to use'),
        DeclareLaunchArgument('rate_hz', default_value='0.0', description='Continuous estimates per second; 0 = only on ~/detect'),
        DeclareLaunchArgument('target_frame', default_value='world', description='Frame of the published poses'),
        DeclareLaunchArgument('device', default_value='cuda', description='torch device for MobileSAM (falls back to cpu)'),
        DeclareLaunchArgument('weights', default_value='', description='MobileSAM checkpoint; empty = external/weights/mobile_sam.pt'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]
    params = {
        'rate_hz': ParameterValue(LaunchConfiguration('rate_hz'), value_type=float),
        'target_frame': LaunchConfiguration('target_frame'),
        'device': LaunchConfiguration('device'),
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
    }
    return LaunchDescription(args + [
        Node(
            package='vision',
            executable='box_pose_estimator',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[params, {'weights': LaunchConfiguration('weights')}],
        ),
    ])

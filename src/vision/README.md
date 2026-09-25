# Box pose estimator (zero-shot, MobileSAM)

`box_pose_estimator` finds box top faces in the gripper camera
(`/<ns>/camera/color`, `camera/depth`, `camera/camera_info`):
MobileSAM masks → depth back-projection → largest plane (RANSAC) → oriented
rectangle (`vision/box_geometry.py`) → pose in `world` via the robot's TF.

```bash
# once, inside the container: MobileSAM + timm submodules in external/, weights
./scripts/setup_external.sh
ros2 launch vision box_pose.launch.py              # add rate_hz:=1.0 for continuous
ros2 service call /robot_1/box_pose_estimator/detect std_srvs/srv/Trigger
ros2 run vision box_pose_check --ros-args -p trigger_period:=5.0   # errors vs Gazebo ground truth
```

Outputs under `/<ns>/box_pose_estimator/`: `detections` (PoseArray, top-face
centres, +Z = face normal, closest first), `markers` (MarkerArray, face
size in `scale`), `debug_image` (masks + fitted rectangles).
Masks are rejected when tiny, belt-sized, cut off by the image edge, not
planar, tilted more than `max_tilt_deg` (side faces) or outside
`min_size`–`max_size`.

External libraries are found through `shared_utils.external`
(`$ZSR_EXTERNAL_DIR`, default `/workspaces/isaac_ros-dev/external`).

---

# Launch

ros2 launch vision nvblox.launch.py input_type:=depth_image mode:=static
or
ros2 launch vision nvblox.launch.py input_type:=pointcloud mode:=static

# Quick Reference: Isaac ROS nvblox + Gazebo Sim + RViz2## 1. System Architecture

[ Gazebo Sim ] ---> (ros_gz_bridge) ---> [ ROS 2 (nvblox_node) ] ---> [ RViz2 ]

## 2. Gazebo to ROS 2 Bridge (`ros_gz_bridge`)

Run these commands to bridge the simulated RealSense camera and robot transforms:

```bash
# 1. Depth Image
ros2 run ros_gz_bridge parameter_bridge /camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image

# 2. Camera Info (Intrinsics)
ros2 run ros_gz_bridge parameter_bridge /camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo

# 3. Odometry (If not using a SLAM node)
ros2 run ros_gz_bridge parameter_bridge /model/your_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry
```

## 3. Minimal ROS 2 Launch File (`nvblox.launch.py`)

Save this file in your ROS 2 package to run `nvblox` via an optimized GPU container.

```python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='nvblox_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            output='screen',
            composable_node_descriptions=[
                ComposableNode(
                    package='isaac_ros_nvblox',
                    plugin='nvidia::isaac_ros::nvblox::NvbloxNode',
                    name='nvblox_node',
                    parameters=[{
                        'global_frame': 'world',         # Fixed Gazebo frame
                        'use_depth': True,
                        'use_color': False,
                        'use_lidar': False,
                        'voxel_size': 0.05,             # 5cm grid resolution
                        'esdf_slice_height': 0.5,       # Height for 2D obstacle slice
                        'max_depth_integration_distance_m': 5.0
                    }],
                    remappings=[
                        ('depth/image', '/camera/depth_image'),
                        ('depth/camera_info', '/camera/camera_info')
                    ]
                )
            ]
        )
    ])
```

## 4. RViz2 Visualization Setup

| Feature        | RViz2 Display Type | Topic                          | Required QoS Policy                                            |
| :------------- | :----------------- | :----------------------------- | :------------------------------------------------------------- |
| **3D Mesh**    | `MarkerArray`      | `/nvblox_node/mesh`            | **Reliability:** Best Effort <br> **Durability:** Volatile     |
| **2D Slice**   | `PointCloud2`      | `/nvblox_node/esdf_pointcloud` | **Reliability:** Best Effort <br> **Durability:** Volatile     |
| **Static Map** | `Map`              | `/nvblox_node/static_map`      | **Reliability:** Reliable <br> **Durability:** Transient Local |

> ⚠️ **Important:** If you do not change the QoS **Reliability** to **Best Effort** for the Mesh and PointCloud, RViz2 will display nothing.

If you need help, let me know if you want the URDF sensor block for the Gazebo camera or a pre-configured .rviz config file payload.

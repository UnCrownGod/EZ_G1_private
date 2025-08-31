# ROS2 Launch: 启动 Gazebo + world，并尝试 spawn G1（需你提供官方模型/包名）
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    world_file = os.path.join(os.getcwd(), "vision", "sim", "world", "lab.world")
    gz = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    ), launch_arguments={"world": world_file}.items())

    # TODO: 如果 Unitree 官方提供 spawn 脚本/包，在此 include。否则用 gazebo_ros spawn_entity + 你的 SDF.
    # 例:
    # spawn = ExecuteProcess(
    #     cmd=['ros2','run','gazebo_ros','spawn_entity.py','-entity','g1','-file', '/path/to/g1.sdf'],
    #     output='screen')

    return LaunchDescription([gz])

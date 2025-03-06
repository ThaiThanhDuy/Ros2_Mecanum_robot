import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['src/robot_main/launch/main.py']), # lidar
        ),
    
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['src/robot_main/launch/slam_toolbox_launch.py']), #run rviz
        ),

    ])
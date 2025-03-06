import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        
       

     
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['src/robot_main/launch/rplidar.py']), # lidar
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['src/robot_description/launch/rsp.launch.py']), #run udrf
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['src/robot_main/launch/launch.py']), #run rviz
        ),

    ])
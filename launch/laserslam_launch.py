from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():

    return LaunchDescription([
    	ExecuteProcess(
    	    cmd=['../build/listener'],
    	    output='screen'
    	),
    	ExecuteProcess(
    	    cmd=['ros2','bag','play','-r','0.1','/home/bruce/workspace/laser_slam_dataset/deut/cartographer_paper_deutsches_museum.db3'],
    	    output='screen'
    	),
	Node(
       	    package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join('/home/bruce/workspace/laser_slam/', 'rviz', 'rvizconfig.rviz')]],
            output='screen'
        )
    ])
    

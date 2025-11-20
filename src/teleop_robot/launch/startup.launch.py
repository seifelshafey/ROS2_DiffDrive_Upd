from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # 1. Define Paths
    pkg_share = FindPackageShare('teleop_robot')
    
    rviz_config_path = PathJoinSubstitution([
        pkg_share, 'rviz', 'my_robot.rviz'
    ])

    # 2. Include the RSP Launch file (This handles the URDF processing)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'rsp.launch.py'])
        ),
    )

    # 3. Define Nodes
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    spawn_node = Node(
        package='teleop_robot',
        executable='spawn', 
        name='spawn_node',
        output='screen',
        parameters=[{'origin': [0.0, 0.0, 0.0]}]
    )

    state_est_node = Node(
        package='teleop_robot',
        executable='state',
        name='state_est',
        output='screen'
    )

    # 4. Create the Launch Description
    return LaunchDescription([
        rsp,
        joint_state_publisher_gui_node,
        rviz_node,
        spawn_node,
        
        TimerAction(
            period=2.0,
            actions=[state_est_node]
        )
    ])
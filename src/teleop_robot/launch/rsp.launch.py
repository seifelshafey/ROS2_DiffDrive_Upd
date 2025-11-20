import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # 1. Declare the "use_sim_time" argument
    # This allows us to pass "true" for Gazebo and "false" for real robots
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true'
    )

    # 2. Process the URDF
    pkg_path = get_package_share_directory('teleop_robot')
    xacro_file = os.path.join(pkg_path, 'urdf', 'my_robot.urdf.xacro')
    
    # Generate the XML via xacro command
    robot_description_content = Command(['xacro ', xacro_file])
    
    # Wrap it in ParameterValue to treat it as a string
    xml_param = ParameterValue(robot_description_content, value_type=str)

    # 3. Create the Robot State Publisher Node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': xml_param,
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        node_robot_state_publisher
    ])
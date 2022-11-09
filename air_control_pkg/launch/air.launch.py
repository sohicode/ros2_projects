from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='air_control_pkg',
            executable='pm_server',
            name='particulate_matter_server',
            output='screen'
        ),
        Node(
            package='air_control_pkg',
            executable='pm_sensor',
            name='particulate_matter_sensor',
            output='screen'
        ),
        Node(
            package='air_control_pkg',
            executable='control_unit',
            name='central_control_unit_node',
            output='screen'
        ),
        Node(
            package='air_control_pkg',
            executable='air_cleaner',
            name='air_conditioner_node',
            output='screen'
        )
    ])




# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='air_control_pkg',
#             node_executable='param_talker',
#             node_name='particulate_matter_sensor',
#             output='screen',
#             parameters=[
#                 {'pm_10_parameter': 'earth'}
#             ]
#         )
#     ])
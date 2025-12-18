from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    calibration_node = Node(
        package='eureka_imuwt901',
        executable='wt901_calibration_node',
        name='wt901_calibration_node',
        output='screen'
    )

    main_node = TimerAction(
        period=1.0, 
        actions=[
            Node(
                package='eureka_imuwt901',
                executable='wt901_node',
                name='wt901_node',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        #calibration_node,
        main_node,
    ])
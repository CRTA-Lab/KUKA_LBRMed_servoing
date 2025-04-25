from launch_ros.actions import Node, PushRosNamespace

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, Event
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource



import os

def generate_launch_description():
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robots namespace'
    )

    model = DeclareLaunchArgument(
        'model',
        default_value='iiwa14_tool',
        description='Used lbr model',
        choices=['iiwa7', 'iiwa14', 'med7', 'med14','iiwa14_tool']
    )

    mode = DeclareLaunchArgument(
        'mode',
        default_value='hardware',
        description='Used lbr model',
        choices=['gazebo','mock','hardware']
    )
    
    namespace = LaunchConfiguration('namespace')
    

    robot_description = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('lbr_bringup'), 'launch'),
        '/hardware.launch.py']),
        launch_arguments={'model': LaunchConfiguration('model'), 'ctrl':'forward_position_controller'}.items(),
    )

    moveit_servo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('lbr_bringup'), 'launch'),
         '/moveit_servo.launch.py']),
         launch_arguments={'model': LaunchConfiguration('model'), 'mode':LaunchConfiguration('mode')}.items(), 

    )


    image_acq = GroupAction([
    PushRosNamespace(namespace),

    Node(
        package='lbr_prostate_sweep',
        executable='us_acquisition',
        output='log',
    )

    ])

    servoing = GroupAction([    
    PushRosNamespace(namespace),

    Node(
        package='lbr_prostate_sweep',
        executable='run_sweep_serv_wire',
        output='log',
    )

    ])

    aiming = GroupAction([    
    PushRosNamespace(namespace),

    Node(
        package='lbr_prostate_sweep',
        executable='needle_aim_wire',
        output='log',
    )

    ])

    ld = LaunchDescription([
            namespace_launch_arg,
            model,mode
    ])
    ld.add_action(image_acq)
    ld.add_action(servoing)
    ld.add_action(aiming)
    ld.add_action(robot_description)
    ld.add_action(moveit_servo)

    return ld
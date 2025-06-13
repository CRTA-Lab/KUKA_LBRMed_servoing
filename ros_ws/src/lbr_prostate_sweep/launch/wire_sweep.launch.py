from launch_ros.actions import Node, PushRosNamespace

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

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
        launch_arguments={'model': LaunchConfiguration('model'), 'ctrl':'joint_trajectory_controller','namespace': LaunchConfiguration('namespace')}.items(),
    )


    move_grup = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('iiwa14_tool_moveit_config'), 'launch'),
         '/move_group.launch.py']),
         launch_arguments={'robot_name': 'lbr','namespace': 'lbr','model': LaunchConfiguration('model'), 'mode':LaunchConfiguration('mode')}.items(),
         )
    
    rsp = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('iiwa14_tool_moveit_config'), 'launch'),
        '/rsp.launch.py']),
        launch_arguments={'namespace': 'lbr','model': LaunchConfiguration('model'), 'mode':LaunchConfiguration('mode')}.items(),
        )

    moveit_rviz = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('iiwa14_tool_moveit_config'), 'launch'),
         '/moveit_rviz.launch.py']),
         launch_arguments={'namespace': 'lbr'}.items())
    
    spawn_controllers = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('iiwa14_tool_moveit_config'), 'launch'),
         '/spawn_controllers.launch.py']),
         )

    moveit_servo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('lbr_bringup'), 'launch'),
         '/moveit_servo.launch.py']),
         launch_arguments={'model': LaunchConfiguration('model'), 'mode':LaunchConfiguration('mode')}.items() 

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

    helper = GroupAction([    
    PushRosNamespace(namespace),

    Node(
        package='lbr_prostate_sweep',
        executable='helper_node',
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
    ld.add_action(helper)
    ld.add_action(robot_description)
    ld.add_action(moveit_servo)
    ld.add_action(rsp)
    ld.add_action(move_grup)
    #ld.add_action(spawn_controllers)
    ld.add_action(moveit_rviz)

    return ld
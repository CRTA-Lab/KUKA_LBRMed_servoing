from launch_ros.actions import Node, PushRosNamespace

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, Event
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource



import os
import time

def generate_launch_description():


    log_dir = '~/KUKA_LBRMed_servoing/logs/{}/'.format(time.strftime("%Y-%m-%d_%H-%M-%S"))
    log_dir = os.path.expanduser(log_dir)
    os.mkdir(log_dir)

    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robots namespace'
    )

    model = DeclareLaunchArgument(
        'model',
        default_value='iiwa14',
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
        executable='run_sweep_serv',
        output='log',
        parameters=[{'data_logger_path': log_dir}],
    )

    ])

    aiming = GroupAction([    
    PushRosNamespace(namespace),

    Node(
        package='lbr_prostate_sweep',
        executable='needle_aim',
        output='log',
    )

    ])

    force_comp = GroupAction([    
    PushRosNamespace(namespace),

    Node(
        package='lbr_prostate_sweep',
        executable='force_compensation',
        output='log',
        parameters=[{'data_logger_path': log_dir}]
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

    yolo = GroupAction([    
    PushRosNamespace(namespace),

    Node(
        package='microsegnet_inference',
        executable='yolo_predict',
        output='log',
    )

    ])
    microsegnet = GroupAction([    
    PushRosNamespace(namespace),

    Node(
        package='microsegnet_inference',
        executable='microsegnet_inference',
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
    #ld.add_action(helper)
    ld.add_action(force_comp)
    ld.add_action(robot_description)
    ld.add_action(moveit_servo)
    ld.add_action(yolo)
    ld.add_action(microsegnet)

    return ld
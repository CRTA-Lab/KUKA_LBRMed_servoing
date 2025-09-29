from launch_ros.actions import Node, PushRosNamespace

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, Event
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource



import os
import glob

def generate_launch_description():

    dir = '~/KUKA_LBRMed_servoing/logs/'
    dir = os.path.expanduser(dir)
    files = list(filter(os.path.isdir, glob.glob(dir + '*')))
    files.sort(key=os.path.getmtime, reverse=True)
    log_dir = files[0] if len(files) > 0 else dir

    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robots namespace'
    )

    model = DeclareLaunchArgument(
        'model',
        default_value='med7',
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
        launch_arguments={'model': LaunchConfiguration('model'), 'ctrl':'forward_position_controller','sys_cfg':'ros2_control/lbr_system_config_med.yaml'}.items(),
    )

    moveit_servo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('lbr_bringup'), 'launch'),
         '/moveit_servo.launch.py']),
         launch_arguments={'model': LaunchConfiguration('model'), 'mode':LaunchConfiguration('mode')}.items(), 

    )


    
    vert_mot = GroupAction([    
    PushRosNamespace(namespace),

    Node(
        package='patient_motion_sim',
        executable='horisontal_motion',
        output='log',
        parameters=[{'data_logger_path': log_dir}],
    )

    ])

    ld = LaunchDescription([
            namespace_launch_arg,
            model,mode
    ])

    ld.add_action(vert_mot)
    ld.add_action(robot_description)
    ld.add_action(moveit_servo)

    return ld
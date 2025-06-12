from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("iiwa14", package_name="iiwa14_tool_moveit_config").moveit_cpp(file_path=get_package_share_directory("iiwa14_tool_moveit_config") + "/config/planing_pipeline_config.yaml").to_moveit_configs()
    return generate_rsp_launch(moveit_config)

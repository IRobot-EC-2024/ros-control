import os
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PARAM_CONFIG_REQUIREMENTS = [
    'hardware_settings.yaml'
]

def generate_launch_description():
    # 遍历config文件夹下的所有yaml文件作为parameters输入
    param_config_files = list()
    param_config_files_name = set()
    for root, dir, files in os.walk(os.path.join(get_package_share_directory('control_node'), 'config')):
        for file in files:
            if file.endswith('.yaml'):
                param_config_files.append(os.path.join(root, file))
                param_config_files_name.add(file)
    
    # 检查必须的配置文件是否存在，不存在就退出
    for requirement in PARAM_CONFIG_REQUIREMENTS:
        if requirement not in param_config_files_name:
            print(f"[ERROR] [{os.path.basename(__file__)}]: Required configuration file config/{requirement} does not exist")
            sys.exit(1)

    return LaunchDescription(
        [
            Node(
                package='control_node',
                executable='control_node',
                name='control_node',
                output='screen',
                parameters=param_config_files,
            )
        ]
    )

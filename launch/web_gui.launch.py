from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Rosbridge launch
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml'
            )
        )
    )

    # Web GUI node
    web_gui_node = Node(
        package='web_gui',
        executable='web_server',
        name='web_gui_node',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        rosbridge_launch,
        web_gui_node
    ])

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    # Caminho do launch da usb_cam
    usb_cam_launch = os.path.join(
        get_package_share_directory('usb_cam'),
        'launch',
        'camera.launch.py'
    )

    # Caminho do april_tag (seu pacote)
    april_tag_launch = os.path.join(
        get_package_share_directory('pick_place_ned2'),
        'launch',
        'april_tag.launch.py'
    )

    return LaunchDescription([

        # 📷 USB CAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(usb_cam_launch)
        ),

        # 🎯 APRIL TAG
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(april_tag_launch)
        ),

        # 🖼️ RQT Image View
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
            output='screen'
        )
    ])

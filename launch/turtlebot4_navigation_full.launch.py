from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Caminho do mapa
    map_file = "/home/aline/ros2_ws/src/testes_turtlebot/turtlebot4_python_tutorials_testes/maps/siro_lab.yaml"

    # ===============================
    # Localization (AMCL + Map)
    # ===============================
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot4_navigation'),
                'launch',
                'localization.launch.py'
            ])
        ),
        launch_arguments={
            'map': map_file
        }.items()
    )

    # ===============================
    # Nav2 (Planner, Controller, BT, etc.)
    # ===============================
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot4_navigation'),
                'launch',
                'nav2.launch.py'
            ])
        )
    )

    # ===============================
    # RViz (View Navigation)
    # ===============================
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot4_viz'),
                'launch',
                'view_navigation.launch.py'
            ])
        )
    )

    return LaunchDescription([
        localization_launch,
        nav2_launch,
        rviz_launch
    ])


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('mapping_bot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    default_map_file = os.path.join(pkg_share, 'maps', 'peta_ruangan.yaml')
    default_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map_file,
            description='Path absolut ke file peta .yaml'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path absolut ke file konfigurasi Nav2'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Gunakan waktu sistem nyata pada robot fisik'
        ),
        nav2_bringup
    ])

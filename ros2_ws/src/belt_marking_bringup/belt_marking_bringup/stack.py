"""Builds the complete node graph for sim or real; used by all launch files."""

import os

from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

from . import config_loader as cl


def _installed(pkg: str) -> bool:
    return pkg in get_packages_with_prefixes()


def conveyor_end_m(cfg) -> float:
    far = max(cl.station_offsets(cfg) + [cfg['machine']['knife_offset_mm']]) / 1000.0
    return max(0.45, far + 0.25)


def build(cfg_path: str, overlays, use_sim: bool, gazebo: bool, rviz: bool, ui: bool,
          vision: bool, headless: bool = False, kiosk: bool = False):
    cfg = cl.load(cfg_path, overlays)
    offsets = cl.station_offsets(cfg)
    n = len(offsets)
    knife = float(cfg['machine']['knife_offset_mm'])
    desc_share = get_package_share_directory('belt_marking_description')
    urdf = xacro.process_file(
        os.path.join(desc_share, 'urdf', 'belt_marking_machine.urdf.xacro'),
        mappings={
            'station_offsets': ' '.join(f'{v / 1000.0:.4f}' for v in offsets),
            'belt_width': f'{cfg["geometry"]["belt_width_mm"] / 1000.0:.4f}',
            'knife_offset': f'{knife / 1000.0:.4f}',
            'camera_offset': f'{cfg["geometry"]["camera_offset_mm"] / 1000.0:.4f}',
            'laser_field': f'{cfg["laser"]["field_length_mm"] / 1000.0:.4f}',
            'gazebo': 'true' if gazebo else 'false',
        }).toxml()
    x_end = conveyor_end_m(cfg)

    actions = [SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')]
    if use_sim:
        actions.append(Node(package='belt_marking_hardware', executable='sim_hardware_node',
                            output='screen', parameters=[cl.sim_hardware_params(cfg)]))
    else:
        actions.append(Node(package='belt_marking_hardware', executable='serial_bridge_node',
                            output='screen', parameters=[cl.serial_bridge_params(cfg)]))
    actions += [
        Node(package='belt_marking_control', executable='control_node', output='screen',
             parameters=[cl.control_params(cfg, use_sim)]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf}]),
        Node(package='belt_marking_description', executable='joint_state_node',
             parameters=[{'num_stations': n,
                          'knife_stroke_time_s': float(cfg['sim']['knife_stroke_time_s']),
                          'laser_field_m': cfg['laser']['field_length_mm'] / 1000.0}]),
        Node(package='belt_marking_control', executable='rviz_markers_node',
             parameters=[{'station_offsets_mm': offsets, 'knife_offset_mm': knife,
                          'mark_length_mm': float(cfg['geometry']['mark_length_mm']),
                          'x_max_m': x_end,
                          'default_belt_width_mm': float(cfg['geometry']['belt_width_mm'])}]),
    ]
    if rviz:
        actions.append(Node(package='rviz2', executable='rviz2', output='log',
                            arguments=['-d', os.path.join(desc_share, 'rviz', 'machine.rviz')]))
    if gazebo and use_sim:
        gz_share = get_package_share_directory('belt_marking_gazebo')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz_share, 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'headless': 'true' if headless else 'false', 'num_stations': str(n),
                'station_offsets_mm': ' '.join(str(v) for v in offsets),
                'knife_offset_mm': str(knife),
                'belt_width_mm': str(cfg['geometry']['belt_width_mm']),
            }.items()))
    if ui and _installed('belt_marking_ui'):
        actions.append(Node(package='belt_marking_ui', executable='operator_ui',
                            output='screen',
                            parameters=[{'kiosk': kiosk, 'use_sim': use_sim,
                                         'db_path': cfg['control']['db_path'],
                                         'config_path': cfg_path}]))
    if vision and _installed('belt_marking_vision'):
        actions.append(Node(package='belt_marking_vision', executable='vision_qa_node',
                            output='screen',
                            parameters=[{'camera_offset_mm':
                                         float(cfg['geometry']['camera_offset_mm']),
                                         'synthetic': not gazebo,
                                         # Gazebo QA camera: belt band after rotation
                                         'roi_across': [0.34, 0.66],
                                         # Gazebo decals move in steps (twin update rate),
                                         # so allow 3 mm there; the real belt uses 2 mm
                                         'max_offset_mm': 3.0 if gazebo else 2.0,
                                         'mark_length_mm':
                                         float(cfg['geometry']['mark_length_mm']),
                                         'use_sim': use_sim}]))
        actions.append(Node(package='belt_marking_vision', executable='anomaly_node',
                            parameters=[{'db_path': cfg['control']['db_path']}]))
    return actions

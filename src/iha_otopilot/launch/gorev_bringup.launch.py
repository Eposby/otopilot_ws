"""
Görev Bringup Launch — Tüm Katmanları Başlat
===============================================

Kullanım:
    ros2 launch iha_otopilot gorev_bringup.launch.py

Bu launch dosyası aşağıdaki 5 düğümü aynı anda başlatır:
    1. telemetry_monitor    (Algı)
    2. vision_processor     (Gözler)
    3. yolo_detector        (Gözler)
    4. safety_watchdog      (Refleks)
    5. mission_commander    (Beyin)
    6. trajectory_generator (Yörünge)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ═══ Katman 1: Algı ═══
        Node(
            package='iha_otopilot',
            executable='telemetry_monitor',
            name='telemetry_monitor_node',
            output='screen',
        ),
        Node(
            package='iha_otopilot',
            executable='vision_processor',
            name='vision_processor_node',
            output='screen',
        ),

        # ═══ Katman 2: Refleks ═══
        Node(
            package='iha_otopilot',
            executable='safety_watchdog',
            name='safety_watchdog_node',
            output='screen',
        ),

        # ═══ Katman 3: Karar ═══
        Node(
            package='iha_otopilot',
            executable='mission_commander',
            name='mission_commander_node',
            output='screen',
        ),

        # ═══ Katman 4: Yörünge ═══
        Node(
            package='iha_otopilot',
            executable='trajectory_generator',
            name='trajectory_generator_node',
            output='screen',
        ),
    ])

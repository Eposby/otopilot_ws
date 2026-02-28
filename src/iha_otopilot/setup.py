from setuptools import find_packages, setup

package_name = 'iha_otopilot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gorev_bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    package_data={
        'iha_görev': ['models/*.pt', 'models/*.caffemodel', 'models/*.prototxt'],
    },
    zip_safe=True,
    maintainer='mert',
    maintainer_email='mert@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pilot= iha_otopilot.ucus_kontrol:main',
            'gozcu=iha_otopilot.goruntu_isleme:main',
            'pil_durumu=iha_otopilot.pil_durumu:main',
            'hedef_bilgi=iha_otopilot.hedef_durum:main',
            'emir_client=iha_otopilot.komut_client:main',
            'emir_server=iha_otopilot.komut_server:main',
            'camera=iha_otopilot.goruntu_isleme:main',
            'test=iha_otopilot.simple_way:main',
            'point=iha_otopilot.three_point:main',
            'camera_bridge=iha_otopilot_mavros.gz_bridge_camera.py',
            'qr=iha_otopilot_mavros.qr_mission.py'
            'track=iha_otopilot_mavros.track_mission.py',
            'hss_mission=iha_otopilot_mavros.hss_waypoint_mission:main',
            'enemy_plane=iha_otopilot_mavros.enemy_plane_mission:main',
            'dynamic_mission=iha_otopilot_mavros.dynamic_mission_manager:main',
            'telemetry_monitor=iha_görev.telemetry_monitor:main',
            'vision_processor=iha_görev.vision_processor:main',
            'safety_watchdog=iha_görev.safety_watchdog:main',
            'mission_commander=iha_görev.mission_commander:main',
            'trajectory_generator=iha_görev.trajectory_generator:main',
            'qr_vision=iha_görev.qr_vision_node:main',

        ],
    },
)

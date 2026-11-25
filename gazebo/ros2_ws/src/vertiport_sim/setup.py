from glob import glob

from setuptools import setup

package_name = 'vertiport_sim'

setup(
    name=package_name,
    version='1.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/www', glob('www/*.html')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team 이세지',
    maintainer_email='team@example.com',
    description='Smart vertiport safety simulation (PX4 SITL + Gazebo X500)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'commander = vertiport_sim.commander_node:main',
            'telemetry_bridge = vertiport_sim.telemetry_bridge:main',
            'http_bridge = vertiport_sim.http_bridge:main',
            'perception_stub = vertiport_sim.perception_stub:main',
            'scenario_manager = vertiport_sim.scenario_node:main',
            'mock_judge = vertiport_sim.mock_judge_node:main',
            'vp_cmd = vertiport_sim.cli:cmd_main',
            'vp_scenario = vertiport_sim.cli:scenario_main',
        ],
    },
)

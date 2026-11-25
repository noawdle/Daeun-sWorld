"""버티포트 시뮬레이션 ROS2 노드 일괄 실행.

사전 실행: MicroXRCEAgent udp4 -p 8888,  PX4 SITL + Gazebo (scripts/run_sim.sh 가 모두 처리)

  ros2 launch vertiport_sim sim.launch.py spawn_x:=-6.0 spawn_y:=0.0
  ros2 launch vertiport_sim sim.launch.py mock_judge:=true auto_mission:=true
  ros2 launch vertiport_sim sim.launch.py sim:=false          # 실기체(Jetson) 모드: gz 브리지/시나리오 제외

대시보드: http://<이 PC IP>:8080  (API: /api/health, /api/telemetry, /api/command, /api/perception, /api/scene)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    g = lambda k: context.launch_configurations[k]  # noqa: E731
    sim = g('sim').lower() in ('1', 'true', 'yes')
    world = g('world')
    model_inst = f"{g('model')}_{g('instance')}"
    origin = [float(g('spawn_x')), float(g('spawn_y')), float(g('base_height'))]
    cfg = g('config') or os.path.join(get_package_share_directory('vertiport_sim'), 'config', 'vertiport.yaml')
    common = [cfg, {'origin_offset_enu': origin, 'world_name': world}]

    actions = []
    if sim:
        cam = g('camera_gz_prefix') or f'/world/{world}/model/{model_inst}/link/camera_link/sensor/imager'
        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='vp_gz_bridge', output='screen',
            arguments=[
                f'{cam}/image@sensor_msgs/msg/Image[gz.msgs.Image',
                f'{cam}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                f'/world/{world}/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            remappings=[
                (f'{cam}/image', g('camera_topic')),
                (f'{cam}/camera_info', g('camera_info_topic')),
                (f'/world/{world}/clock', '/clock'),
            ],
        ))
        actions.append(Node(package='vertiport_sim', executable='scenario_manager',
                            name='vertiport_scenario', output='screen', parameters=common))

    actions.append(Node(package='vertiport_sim', executable='commander',
                        name='vertiport_commander', output='screen', parameters=common))
    actions.append(Node(package='vertiport_sim', executable='telemetry_bridge',
                        name='vertiport_telemetry_bridge', output='screen',
                        parameters=common + [{'source': 'sim' if sim else 'real',
                                              'tf_stamp_source': 'px4' if sim else 'ros'}]))
    actions.append(Node(package='vertiport_sim', executable='http_bridge',
                        name='vertiport_http_bridge', output='screen',
                        parameters=common + [{'http_port': int(g('http_port')),
                                              'camera_topic': g('camera_topic'),
                                              'camera_info_topic': g('camera_info_topic')}]))
    if g('perception_stub').lower() in ('1', 'true', 'yes'):
        actions.append(Node(package='vertiport_sim', executable='perception_stub',
                            name='vertiport_perception_stub', output='screen', parameters=common))
    if g('mock_judge').lower() in ('1', 'true', 'yes'):
        actions.append(Node(package='vertiport_sim', executable='mock_judge',
                            name='vertiport_mock_judge', output='screen',
                            parameters=common + [{'auto_mission': g('auto_mission').lower() in ('1', 'true', 'yes')}]))
    return actions


def generate_launch_description():
    args = [
        ('sim', 'true', 'true: Gazebo SITL / false: 실기체 (gz 브리지·시나리오 매니저 제외)'),
        ('world', 'vertiport', 'Gazebo world 이름 (PX4_GZ_WORLD 와 동일)'),
        ('model', 'x500_mono_cam_down', 'PX4 Gazebo 모델 (PX4_SIM_MODEL 의 gz_ 뒤 부분)'),
        ('instance', '0', 'PX4 인스턴스 번호 (-i)'),
        ('spawn_x', '-6.0', '스폰 위치 x (World ENU) - PX4_GZ_MODEL_POSE 와 같아야 함'),
        ('spawn_y', '0.0', '스폰 위치 y (World ENU)'),
        ('base_height', '0.24', '지상 정지 시 base_link 높이 [m] (X500)'),
        ('http_port', '8080', 'HTTP Bridge 포트 (대시보드 접속)'),
        ('camera_topic', '/camera/down/image_raw', '하향 카메라 ROS 토픽 (실기체와 동일하게 유지)'),
        ('camera_info_topic', '/camera/down/camera_info', '카메라 내부 파라미터 토픽'),
        ('camera_gz_prefix', '', '카메라 gz 토픽 접두사 직접 지정 (비우면 자동 생성)'),
        ('mock_judge', 'false', 'GT 기반 모의 판단 노드 실행'),
        ('perception_stub', 'false', 'GT 투영 가짜 탐지로 /vertiport/perception 발행 (통합 프로그램 대체용)'),
        ('auto_mission', 'false', 'mock_judge 가 이륙→접근→착륙 자동 수행'),
        ('config', '', '파라미터 YAML 경로 (비우면 패키지 기본값)'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=desc) for n, d, desc in args]
        + [OpaqueFunction(function=_setup)])

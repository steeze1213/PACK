from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory('final_mission_robot')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    turtlebot3_navigation_share = get_package_share_directory('turtlebot3_navigation2')

    # =========================
    # 기존 TurtleBot3 AMCL 파라미터
    # 기존 air_clean_localization.launch.py와 동일하게 사용
    # =========================
    tb3_nav_params = os.path.join(
        turtlebot3_navigation_share,
        'param',
        'humble',
        'waffle_pi.yaml',
    )

    # =========================
    # 기존 TurtleBot3 RViz 설정
    # =========================
    default_rviz = os.path.join(
        turtlebot3_navigation_share,
        'rviz',
        'tb3_navigation2.rviz',
    )

    # =========================
    # 최종 프로젝트 pure controller 파라미터
    # air_clean_pure_controller 전용
    # =========================
    final_params = os.path.join(
        package_share,
        'config',
        'final_params.yaml',
    )

    map_file = LaunchConfiguration('map')
    localization_params_file = LaunchConfiguration('localization_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    # =========================
    # 기존 localization 방식 그대로 사용
    # map_server + amcl + lifecycle 포함
    # =========================
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_share,
                'launch',
                'localization_launch.py',
            )
        ),
        launch_arguments={
            'map': map_file,
            'params_file': localization_params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )

    # =========================
    # RViz 자동 실행
    # 여기서 수동으로 2D Pose Estimate 찍으면 됨
    # =========================
    rviz_node = Node(
        condition=IfCondition(rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz],
        parameters=[
            {
                'use_sim_time': use_sim_time,
            }
        ],
    )

    # =========================
    # 기존 A* + Pure Pursuit 주행 컨트롤러
    # /goal_pose 또는 /air_clean_command를 받아서 /cmd_vel 발행
    # =========================
    pure_controller_node = Node(
        package='final_mission_robot',
        executable='air_clean_pure_controller',
        name='air_clean_pure_controller',
        output='screen',
        parameters=[final_params],
    )

    # =========================
    # 최종 프로젝트 미션 매니저
    # /mission_cmd 수신
    # /goal_pose 발행
    # =========================
    mission_manager_node = Node(
        package='final_mission_robot',
        executable='mission_manager',
        name='mission_manager_node',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='/home/woo/map_cleaned.yaml',
            description='Full path to the map yaml file.',
        ),

        DeclareLaunchArgument(
            'localization_params_file',
            default_value=tb3_nav_params,
            description='TurtleBot3 localization parameter file.',
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock.',
        ),

        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz for manual initial pose estimation.',
        ),

        # 1. 기존 방식 localization 실행
        localization,

        # 2. RViz 자동 실행
        rviz_node,

        # 3. localization이 뜬 뒤 pure_controller + mission_manager 실행
        TimerAction(
            period=5.0,
            actions=[
                pure_controller_node,
                mission_manager_node,
            ],
        ),
    ])

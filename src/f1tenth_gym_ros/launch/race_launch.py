from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def resolve_map_path(raw_map_path: str) -> str:
    """Resolve map_path portably across any username or workspace location."""
    expanded = os.path.expanduser(raw_map_path)
    if os.path.exists(expanded + '.yaml'):
        return expanded
    map_name = os.path.basename(expanded.rstrip('/'))
    share_path = os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'maps', map_name)
    if os.path.exists(share_path + '.yaml'):
        return share_path
    return expanded


def resolve_rviz_config() -> str:
    """Prefer source gym_bridge.rviz so camera/view updates apply immediately without rebuild."""
    share_dir = get_package_share_directory('f1tenth_gym_ros')
    ws_root = os.path.abspath(os.path.join(share_dir, '..', '..', '..', '..'))
    src_rviz = os.path.join(ws_root, 'src', 'f1tenth_gym_ros', 'launch', 'gym_bridge.rviz')
    if os.path.exists(src_rviz):
        return src_rviz
    return os.path.join(share_dir, 'launch', 'gym_bridge.rviz')


def get_rviz_wsl_env() -> dict:
    """On WSL2, force Mesa llvmpipe software rasterizer to prevent d3d12 black map textures."""
    is_wsl = os.path.exists('/mnt/wslg')
    if not is_wsl and os.path.exists('/proc/version'):
        try:
            is_wsl = 'microsoft' in open('/proc/version', 'r').read().lower()
        except Exception:
            pass
    if is_wsl:
        return {'LIBGL_ALWAYS_SOFTWARE': '1', 'GALLIUM_DRIVER': 'llvmpipe'}
    return {}


def generate_launch_description():
    ld = LaunchDescription()

    # 1. Launch arguments
    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='lidar',
        description='Autonomous controller to launch: lidar, qualifier, gap, or none'
    )
    ld.add_action(controller_arg)

    # 2. Config & Map resolution
    config = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'config',
        'sim.yaml'
    )
    config_dict = yaml.safe_load(open(config, 'r'))
    has_opp = config_dict['bridge']['ros__parameters']['num_agent'] > 1
    resolved_map_path = resolve_map_path(config_dict['bridge']['ros__parameters']['map_path'])

    # 3. Simulator backend bridge node
    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[config, {'map_path': resolved_map_path}],
        output='screen'
    )

    # 4. RViz2 visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', resolve_rviz_config()],
        additional_env=get_rviz_wsl_env()
    )

    # 5. Nav2 Map Server & Lifecycle Manager
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': resolved_map_path + '.yaml'},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},
                    {'use_sim_time': False}]
    )
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    # 6. Ego Racecar Robot State Publisher (URDF/TF)
    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'ego_racecar.xacro')])}],
        remappings=[('/robot_description', 'ego_robot_description')]
    )

    # 7. Autonomous Controllers (delayed by 2.0s to allow map server & bridge to initialize)
    lidar_race_node = Node(
        package='controllers',
        executable='lidar_race_node',
        name='lidar_race_node',
        output='screen',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('controller'), "' == 'lidar'"]))
    )
    qualifier_node = Node(
        package='controllers',
        executable='qualifier_node',
        name='qualifier_node',
        output='screen',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('controller'), "' == 'qualifier'"]))
    )
    gap_node = Node(
        package='controllers',
        executable='gap_node',
        name='gap_node',
        output='screen',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('controller'), "' == 'gap'"]))
    )

    delayed_controllers = TimerAction(
        period=2.0,
        actions=[lidar_race_node, qualifier_node, gap_node]
    )

    # Assemble Launch Tree
    ld.add_action(rviz_node)
    ld.add_action(bridge_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(ego_robot_publisher)

    if has_opp:
        opp_robot_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='opp_robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'opp_racecar.xacro')])}],
            remappings=[('/robot_description', 'opp_robot_description')]
        )
        ld.add_action(opp_robot_publisher)

    ld.add_action(delayed_controllers)

    return ld

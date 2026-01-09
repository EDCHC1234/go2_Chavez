import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    my_planner_pkg = FindPackageShare('go2_config')
    map_yaml_file = PathJoinSubstitution([my_planner_pkg, 'maps', 'mapM10.yaml'])

    # 1. Servidor de Mapas
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': map_yaml_file}, {'use_sim_time': True}]
    )

    # 2. Lifecycle Manager (Activa el mapa)
    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        parameters=[{'autostart': True}, {'node_names': ['map_server']}]
    )

    # 3. Transformada Estática (Ajuste según tu .world y .yaml)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # 4. TU NODO PLANIFICADOR dijkstra* (Parte B)
    global_planner = Node(
        package='Global_Planner',
        executable='planner_node',
        output='screen'
    )

    # 5. RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        map_server,
        lifecycle,
        static_tf,
        global_planner,
        rviz
    ])

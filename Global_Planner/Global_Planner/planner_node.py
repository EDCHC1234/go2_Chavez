#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import sys
import os

# --- CONFIGURACIÓN DE RUTAS ---
package_path = os.path.dirname(os.path.realpath(__file__))
if package_path not in sys.path:
    sys.path.append(package_path)

# Mensajes de ROS 2
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped

# Importación de la librería
try:
    from python_motion_planning.global_planner.graph_search.dijkstra import Dijkstra
    from python_motion_planning.utils import Grid, Node as AlgoNode
    
    # SOLUCIÓN AL ERROR 'NOT SUBSCRIPTABLE':
    # Hacemos que el objeto Node se comporte como una lista (node[0], node[1])
    # para que sea compatible con la lógica interna de Dijkstra.
    AlgoNode.__getitem__ = lambda self, index: self.current[index]
    
except ImportError as e:
    from .python_motion_planning.global_planner.graph_search.dijkstra import Dijkstra
    from .python_motion_planning.utils import Grid, Node as AlgoNode
    AlgoNode.__getitem__ = lambda self, index: self.current[index]

class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner_node')

        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile=qos_map)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, '/global_path', 10)

        self.map_msg = None
        self.robot_pose = None
        self.get_logger().info("🚀 Nodo iniciado con parche de compatibilidad subscriptable.")

    def map_callback(self, msg):
        self.map_msg = msg

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def goal_callback(self, msg):
        if self.map_msg is None or self.robot_pose is None:
            self.get_logger().warn("⚠️ Esperando mapa u odometría...")
            return

        self.get_logger().info("🎯 Calculando nueva ruta...")

        # 1. Preparar Grid
        width = self.map_msg.info.width
        height = self.map_msg.info.height
        grid_data = np.array(self.map_msg.data).reshape((height, width))
        
        env = Grid(width, height)
        # Identificar obstáculos
        obstacles = {(x, y) for y in range(height) for x in range(width) if grid_data[y, x] > 50}
        env.update(obstacles)

        # 2. Convertir Mundo -> Grid
        start_px = self.world_to_grid(self.robot_pose.position.x, self.robot_pose.position.y)
        goal_px = self.world_to_grid(msg.pose.position.x, msg.pose.position.y)

        # 3. Crear objetos Node
        # Usamos tuplas para current y parent
        start_node = AlgoNode((start_px[0], start_px[1]), (start_px[0], start_px[1]), 0, -1)
        goal_node = AlgoNode((goal_px[0], goal_px[1]), (goal_px[0], goal_px[1]), 0, -1)
        
        

        # 4. Ejecutar Dijkstra
        try:
            planner = Dijkstra(start_node, goal_node, env)
            cost, path, _ = planner.plan()
            
            if path:
                self.get_logger().info(f"✅ Ruta encontrada. Costo: {cost:.2f}")
                self.publish_path(path)
            else:
                self.get_logger().error("❌ No se encontró ruta.")
        except Exception as e:
            self.get_logger().error(f"Error en el algoritmo: {e}")

    def world_to_grid(self, x, y):
        origin = self.map_msg.info.origin.position
        res = self.map_msg.info.resolution
        gx = int((x - origin.x) / res)
        gy = int((y - origin.y) / res)
        return (gx, gy)

    def grid_to_world(self, gx, gy):
        origin = self.map_msg.info.origin.position
        res = self.map_msg.info.resolution
        wx = (gx * res) + origin.x
        wy = (gy * res) + origin.y
        return wx, wy

    def publish_path(self, path_list):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for pt in path_list:
            pose = PoseStamped()
            # Acceso flexible a coordenadas
            if hasattr(pt, 'x'):
                wx, wy = self.grid_to_world(pt.x, pt.y)
            elif hasattr(pt, 'current'):
                wx, wy = self.grid_to_world(pt.current[0], pt.current[1])
            else:
                wx, wy = self.grid_to_world(pt[0], pt[1])
                
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        self.get_logger().info("📤 Ruta publicada en RViz.")

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

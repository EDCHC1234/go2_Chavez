#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import heapq
import csv  # Librería para manejar archivos CSV
import os   # Para obtener la ruta actual

from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped

class DijkstraPlannerPro(Node):
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
        self.path_pub = self.create_publisher(Path, '/zed/path_map', 10)

        self.map_msg = None
        self.robot_pose = None
        self.obstacles = set()
        
        # Nombre del archivo donde se guardarán los waypoints
        self.filename = "waypoints_generados.csv"
        
        self.get_logger().info("🚀 Nodo Dijkstra Pro con Guardado en CSV listo.")

    def map_callback(self, msg):
        if self.map_msg is None:
            self.map_msg = msg
            width = msg.info.width
            height = msg.info.height
            data = np.array(msg.data).reshape((height, width))
            
            safety_cells = 5 
            temp_obstacles = set()
            y_obs, x_obs = np.where(data > 50) 
            
            self.get_logger().info("⏳ Inflándolos muros para seguridad...")
            for ox, oy in zip(x_obs, y_obs):
                for dx in range(-safety_cells, safety_cells + 1):
                    for dy in range(-safety_cells, safety_cells + 1):
                        temp_obstacles.add((ox + dx, oy + dy))
            
            self.obstacles = temp_obstacles
            self.get_logger().info(f"✅ Mapa inflado. Celdas prohibidas: {len(self.obstacles)}")

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def world_to_grid(self, x, y):
        res = self.map_msg.info.resolution
        origin = self.map_msg.info.origin.position
        return int((x - origin.x) / res), int((y - origin.y) / res)

    def grid_to_world(self, gx, gy):
        res = self.map_msg.info.resolution
        origin = self.map_msg.info.origin.position
        return (gx * res) + origin.x, (gy * res) + origin.y

    def get_neighbors(self, current):
        neighbors = []
        for dx, dy, cost in [(1,0,1), (-1,0,1), (0,1,1), (0,-1,1), (1,1,1.4), (1,-1,1.4), (-1,1,1.4), (-1,-1,1.4)]:
            nx, ny = current[0] + dx, current[1] + dy
            if 0 <= nx < self.map_msg.info.width and 0 <= ny < self.map_msg.info.height:
                if (nx, ny) not in self.obstacles:
                    neighbors.append(((nx, ny), cost))
        return neighbors

    def dijkstra(self, start, goal):
        queue = [(0.0, start, [])]
        visited = {start: 0.0}
        
        while queue:
            (cost, current, path) = heapq.heappop(queue)
            if current == goal:
                return path + [current]

            for neighbor, weight in self.get_neighbors(current):
                new_cost = cost + weight
                if neighbor not in visited or new_cost < visited[neighbor]:
                    visited[neighbor] = new_cost
                    heapq.heappush(queue, (new_cost, neighbor, path + [current]))
        return None

    def goal_callback(self, msg):
        if self.map_msg is None or self.robot_pose is None:
            return

        start_px = self.world_to_grid(self.robot_pose.position.x, self.robot_pose.position.y)
        goal_px = self.world_to_grid(msg.pose.position.x, msg.pose.position.y)

        if goal_px in self.obstacles:
            self.get_logger().error("❌ Destino inválido (dentro de un muro o zona de seguridad)")
            return

        if start_px in self.obstacles:
            self.obstacles.discard(start_px)

        self.get_logger().info(f"🎯 Calculando ruta segura a {goal_px}...")
        path_grid = self.dijkstra(start_px, goal_px)

        if path_grid:
            self.publish_path(path_grid)
            self.save_to_csv(path_grid) # Llamada para guardar los puntos
            self.get_logger().info(f"✨ Trayectoria publicada y guardada en {self.filename}")
        else:
            self.get_logger().error("❌ No hay conexión posible entre los puntos.")

    def save_to_csv(self, path_grid):
        """Guarda la lista de puntos en un archivo CSV."""
        try:
            with open(self.filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                # Encabezados
                writer.writerow(['x', 'y']) 
                for gx, gy in path_grid:
                    wx, wy = self.grid_to_world(gx, gy)
                    writer.writerow([wx, wy])
        except Exception as e:
            self.get_logger().error(f"Error al guardar CSV: {e}")

    def publish_path(self, path_grid):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for gx, gy in path_grid:
            pose = PoseStamped()
            wx, wy = self.grid_to_world(gx, gy)
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.1 
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)

def main():
    rclpy.init()
    node = DijkstraPlannerPro()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

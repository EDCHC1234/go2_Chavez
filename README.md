# Proyecto SLAM y Planificación de Trayectorias con GO2 (ROS 2)

## Descripción del proyecto
Este proyecto implementa un sistema completo de **SLAM** y **planificación de trayectorias** para un robot cuadrúpedo **Unitree GO2** usando **ROS 2**, **Gazebo** y **RViz**. En una primera etapa, el robot realiza el mapeo de un entorno simulado (*bookstore world*) utilizando SLAM. Posteriormente, se desarrolla un nodo propio que genera un **camino (Path)** en RViz empleando el algoritmo de **Dijkstra**, permitiendo al robot planificar trayectorias hacia diferentes objetivos enviados desde la interfaz gráfica.


---

## Algoritmos utilizados

### 1. SLAM
Se utiliza un algoritmo de SLAM disponible en ROS 2 para la construcción del mapa del entorno a partir de sensores del robot (LIDAR). Este algoritmo permite:
- Estimar la posición del robot en tiempo real.
- Construir un mapa 2D del entorno.
- Guardar el mapa generado para su uso posterior en navegación.

**Variables principales:**
- `map`: mapa generado del entorno.
- `odom`: información de odometría del robot.
- `scan`: datos del sensor LIDAR.

---

### 2. Planificación de trayectorias – Dijkstra
Se implementa un nodo propio de planificación que utiliza el algoritmo de **Dijkstra** para encontrar el camino más corto entre la posición actual del robot y un objetivo enviado desde RViz mediante *2D Goal Pose*.

**Descripción del algoritmo:**
- El mapa se representa como una grilla de celdas navegables.
- Cada celda es un nodo del grafo.
- El costo entre nodos adyacentes es uniforme.
- Dijkstra calcula el camino de menor costo desde el nodo inicial hasta el nodo objetivo.

**Variables principales:**
- `start`: posición inicial del robot.
- `goal`: posición objetivo seleccionada en RViz.
- `grid_map`: representación discreta del mapa.
- `path`: lista de puntos que conforman la trayectoria final.

**Modificaciones realizadas:**
- Adaptación del algoritmo para trabajar con mapas de ocupación de ROS 2.
- Publicación del resultado como mensaje `nav_msgs/Path` para su visualización en RViz.

---

## Dependencias

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo
- RViz2
- Paquetes ROS 2 estándar:
  - `nav_msgs`
  - `geometry_msgs`
  - `sensor_msgs`
  - `rclpy`
- Repositorio del robot GO2

---

## Instalación y configuración

### 1. Crear el workspace

```bash
mkdir -p ~/go2_ws/src
cd ~/go2_ws/src
```

### 2. Clonar el repositorio

```bash
git clone https://github.com/EDCHC1234/go2_Chavez.git
```

### 3. Compilar el workspace

```bash
cd ~/go2_ws
colcon build
source install/setup.bash
```

---

## Estructura del paquete ROS 2

```
go2_ws/
└── src/
    └── go2_Chavez/
        ├── go2_config/
        │   ├── launch/
        │   │   ├── gazebo.launch.py
        │   │   └── planner.launch.py
        │   └── config/
        ├── go2_planner/
        │   └── dijkstra_planner.py
        └── README.md
```

### ROS Node Graph
- Nodo de SLAM
- Nodo del planner (Dijkstra)
- Gazebo (simulación)
- RViz (visualización)

---

## Launch files

- **gazebo.launch.py**: Lanza el entorno de Gazebo con el robot GO2 en el mundo *bookstore*.
- **planner.launch.py**: Lanza RViz y el nodo de planificación de trayectorias usando Dijkstra.

---

## Ejecución del proyecto

### Parte A – Mapeo (SLAM)

**Terminal 1:**
```bash
ros2 launch go2_config gazebo.launch.py world:=bookstore
```

Durante esta etapa, el robot se desplaza por el entorno y genera el mapa mediante SLAM.

### Resultados – Parte A

- Imagen del mapa generado:

![Mapa SLAM](https://github.com/EDCHC1234/go2_Chavez/blob/main/configs/go2_config/maps/mapM10.pgm)

- Video demostrativo:

[Enlace al video de SLAM en YouTube]

---

### Parte B – Planificación de trayectorias

**Terminal 1:**
```bash
ros2 launch go2_config gazebo.launch.py world:=bookstore
```

**Terminal 2:**
```bash
ros2 launch go2_config planner.launch.py
```

Desde RViz se envía el objetivo utilizando **2D Goal Pose**, generando automáticamente el path con el algoritmo de Dijkstra.

### Resultados – Parte B

- Imagen del path generado:

![Path Dijkstra](imagenes/path_dijkstra.png)

- Video demostrativo:

[Enlace al video de planificación en YouTube]

---

## Video demostrativo (C2)

Duración aproximada: 1–3 minutos.

Contenido:
- Robot en Gazebo realizando SLAM.
- Visualización del mapa en RViz.
- Envío de *2D Goal Pose*.
- Generación y actualización del Path con nuevos objetivos.

[Enlace al video completo en YouTube]

---

## Autor

**Daniel Chávez**  
Proyecto académico – ROS 2

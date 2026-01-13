# Proyecto SLAM y Planificación de Trayectorias con GO2 (ROS 2)

## Descripción del proyecto
Este proyecto implementa un sistema completo de **SLAM** y **planificación de trayectorias** para un robot cuadrúpedo **Unitree GO2** usando **ROS 2**, **Gazebo** y **RViz**. En una primera etapa, el robot realiza el mapeo de un entorno simulado (*small_house world*) utilizando SLAM. Posteriormente, se desarrolla un nodo propio que genera un **camino (Path)** en RViz empleando el algoritmo de **Dijkstra**, permitiendo al robot planificar trayectorias hacia diferentes objetivos enviados desde la interfaz gráfica.



###  Planificación de trayectorias – Dijkstra
Se implementa un nodo propio de planificación que utiliza el algoritmo de **Dijkstra** para encontrar el camino más corto entre la posición actual del robot y un objetivo enviado desde RViz mediante *2D Goal Pose*.



## 1. Installation

### 1.0 Install ROS-based dependencies:
```bash
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-xacro
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-velodyne
sudo apt install ros-humble-velodyne-gazebo-plugins
sudo apt-get install ros-humble-velodyne-description
```

### 1.1 Clone and install all dependencies:
    
```bash
sudo apt install -y python3-rosdep
rosdep update

mkdir -p go2_ws/src
cd go2_ws/src
git clone https://github.com/EDCHC1234/go2_Chavez.git
cd ~/go2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 1.2 Build your workspace:
```bash
cd ~/go2_ws
colcon build
. go2_ws/install/setup.bash
```

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



Durante esta etapa, el robot se desplaza por el entorno y genera el mapa mediante SLAM.

### Resultados – Parte A


- Video demostrativo:

[Enlace al video de SLAM en YouTube:]: (https://youtu.be/VuV9zSHhyUM)

---



## 📊 Resultados Visuales


| Mapa obtenido con slam |Mapa editado |
| :--- | :--- |
| ![Slam](https://github.com/EDCHC1234/go2_Chavez/blob/main/Screenshot%20from%202026-01-11%2021-39-08.png) | ![Mapa editado](https://github.com/EDCHC1234/go2_Chavez/blob/main/Screenshot%20from%202026-01-12%2021-47-45.png) |

> **Nota:** Por limitaciones de recuso del equipo se loggro obtener un mapa donde se obtuvo el contorno del mundo small house y luego se edito la imagen para tener un mapa mas limpio.


### Parte B – Planificación de trayectorias

**Terminal 1:**
```bash
cd ~/go2_ws
colcon build
source install/setup.bash
ros2 launch go2_config gazebo.launch.py world:=bookstore
```

**Terminal 2:**
```bash
cd ~/go2_ws
source install/setup.bash
ros2 launch go2_config planner.launch.py
```

Desde RViz se envía el objetivo utilizando **2D Goal Pose**, generando automáticamente el path con el algoritmo de Dijkstra.

### Resultados – Parte B
- Ruta generada

  ![Ruta](https://github.com/EDCHC1234/go2_Chavez/blob/main/Screenshot%20from%202026-01-12%2022-10-35.png)

  
- Video demostrativo:

[Enlace al video de planificación en YouTube] (https://youtu.be/gsP_PMtDc3U)

---

### Explicación del código – Parte B

#### planner_node.py (Nodo de planificación global con Dijkstra)

El archivo `planner_node.py` implementa un **nodo ROS 2 personalizado** encargado de calcular una trayectoria global utilizando el algoritmo de **Dijkstra**, a partir de un mapa de ocupación generado previamente por SLAM.

El nodo se suscribe a los siguientes tópicos:
- `/map` (`nav_msgs/OccupancyGrid`): mapa 2D del entorno.
- `/odom` (`nav_msgs/Odometry`): posición actual del robot.
- `/goal_pose` (`geometry_msgs/PoseStamped`): objetivo enviado desde RViz mediante *2D Goal Pose*.

Y publica:
- `/zed/path_map` (`nav_msgs/Path`): trayectoria final calculada.

Funcionamiento general del nodo:
- El mapa de ocupación se convierte en una grilla de celdas navegables.
- Las celdas ocupadas se inflan artificialmente para crear una **zona de seguridad** alrededor de los obstáculos.
- La posición actual del robot y el objetivo se transforman de coordenadas del mundo a coordenadas de grilla.
- Se aplica el algoritmo de **Dijkstra**, considerando movimientos en 8 direcciones (horizontal, vertical y diagonal).
- El camino resultante se convierte nuevamente a coordenadas del mundo.
- La trayectoria se publica en RViz como un mensaje `Path`.
- Adicionalmente, los puntos del camino se guardan en un archivo CSV (`waypoints_generados.csv`) para análisis posterior.

Este nodo permite recalcular dinámicamente el path cada vez que se envía un nuevo objetivo desde RViz.

---

#### planner.launch.py (Lanzamiento del sistema de planificación)

El archivo `planner.launch.py` se encarga de lanzar todos los nodos necesarios para la **Parte B – Planificación de trayectorias**, integrando mapa, transformaciones, planner y visualización.

Incluye los siguientes componentes:
- **Map Server**: carga el mapa previamente generado en formato `.yaml`.
- **Lifecycle Manager**: activa automáticamente el servidor de mapas.
- **Static Transform Publisher**: publica la transformación estática entre los frames `map` y `odom`.
- **Nodo planner (Dijkstra)**: ejecuta el archivo `planner_node.py`.
- **RViz2**: permite visualizar el mapa, la posición del robot y la trayectoria generada.

Este archivo asegura que todos los elementos necesarios estén sincronizados usando tiempo simulado (`use_sim_time`), permitiendo que el usuario envíe objetivos desde RViz y observe la generación del path en tiempo real.


## Autor

**Daniel Chávez**  
Proyecto académico – ROS 2

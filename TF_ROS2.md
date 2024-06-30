

# UNIDAD 1: INTRODUCCION

Manejo de TF  en ros2 humble entorno rviz2 nivel maestria.

En este curso aprenderá:

Qué son los marcos de coordenadas y por qué son necesarios
El papel de las transformaciones
La biblioteca TF2 y cómo ayuda a gestionar los marcos de coordenadas y las transformaciones
Las herramientas de que dispone ROS2 para introspeccionar e interactuar con la biblioteca TF2
Cómo escribir un emisor de transformación estática
Cómo emitir transformaciones dinámicas
Qué es el nodo Robot State Publisher y cuál es su función


## 1.2 Sistemas de coordenadas, marcos de referencia y marcos de coordenadas

## 1.4 Crear un marco de coordenadas
usando el comando:
```sh
ros2 run tf2_ros static_transform_publisher --x 0 --y 4.0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id deepmind_robot1_odom --child-frame-id rock
```
podras ver que se crea uno nuevo sistema de marco en entorno rviz2

## MAS TEORIA

- Unidades de medida estándar y convenciones de coordenadas en : [documentación oficial](https://www.ros.org/reps/rep-0103.html) 

- Marcos de coordenadas para plataformas móviles : [documentación oficial](https://www.ros.org/reps/rep-0105.html) 

- Marcos de coordenadas para robots humanoides : [documentación oficial](https://www.ros.org/reps/rep-0120.html) 

- Marcos de coordenadas para manipuladores industriales en serie : [documentación oficial](https://gavanderhoorn.github.io/rep/rep-0199.html) 

- Una interfaz estándar para vehículos aéreos : [documentación oficial](https://ros.org/reps/rep-0147.html) 




# UNIDAD 2 FUNDAMENTOS DE LA TF
// En esta unidad aprenderá lo siguiente:

Las herramientas de que dispone ROS2 para introspeccionar e interactuar con la biblioteca TF2
Visualización de los marcos de TF mediante árboles de TF
Visualización de fotogramas TF mediante RVIZ2

## 2.3 Ver_cuadros en formato PDF:
ros2 run tf2_tools view_frames

## 2.4 Ver fotogramas TF utilizando rqt_tf_tree:
```sh
ros2 run rqt_tf_tree rqt_tf_tree
``` 
## 2.5 Ver tramas TF en el terminal utilizando tf_echo
Se puede usar los topics :/tf  y /tf_static  para analizar que se estan publicando todos los TF.
Ahora bien dichos topics publican todos los cuadros del robot y es un poco inhumano de tratar de ver toda la informacion.

## Comando para ver los tópicos de `tf`
```sh
ros2 topic echo /tf
  ``` 

Existe una forma practica de filtrar solo 1 cuadro y asi ver al detalle las transformaciones.

```sh
ros2 run tf2_ros tf2_echo rgb_camera_link_frame turtle_chassis
```   
Ahora bien  los parametros que se muestran se puede varias  cuando movemos un robot  con el siguiente comando:

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/turtle_cmd_vel
``` 
## 2.6 Ver_cuadros utilizando RVIZ2
otra forma de ver y analizar de mejor manera es usa la herramienta RVIZ2
en esta plataforma se puede  lograr mchas cosas para el mejor manejo del tema TF



# UNIDAD 3: Emisión y escucha de TF
Crear una emisora TF estática
Crear un receptor de TF
Comprender la relación entre el tiempo y los TF





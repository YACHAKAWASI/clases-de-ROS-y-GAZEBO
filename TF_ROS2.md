


# 2 FUNDAMENTOS DE LA TF
// En esta unidad aprenderá lo siguiente:

Las herramientas de que dispone ROS2 para introspeccionar e interactuar con la biblioteca TF2
Visualización de los marcos de TF mediante árboles de TF
Visualización de fotogramas TF mediante RVIZ2

2.3 Ver_cuadros en formato PDF:
ros2 run tf2_tools view_frames

2.4 Ver fotogramas TF utilizando rqt_tf_tree:



# Clase de ROS 2

## Tareas

- [ ] Completar la instalación de ROS 2
- [ ] Configurar el entorno de desarrollo
- [ ] Crear el primer nodo

## Notas Importantes

- **ROS 2** es una versión mejorada de **ROS**.
- Utiliza DDS como middleware de comunicación.
- Asegúrate de seguir la [documentación oficial](https://docs.ros.org/en/foxy/index.html) para una guía detallada.

## Ejemplo de Código

Aquí tienes un ejemplo de cómo iniciar un nodo en ROS 2 usando Python:

```python
# Ejemplo de código en Python
import rclpy
from rclpy.node import Node

class MiNodo(Node):
    def __init__(self):
        super().__init__('mi_nodo')
        self.get_logger().info('Nodo iniciado')

def main(args=None):
    rclpy.init(args=args)
    nodo = MiNodo()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

## Resumen del curso
Este curso se embarcará en un apasionante viaje al reino de la percepción en robótica utilizando ROS 2. A través de una serie de unidades estructuradas y proyectos prácticos, explorará diversos aspectos del procesamiento de datos de sensores y técnicas de percepción.

## Lo que aprenderás

ROS 2, percepción, procesamiento de imágenes, OpenCV, procesamiento de nubes de puntos, Yolo, técnicas avanzadas de percepción, aprendizaje profundo

# UNIT 1:   Course Intro


# UNIT 2:   Working With Sensor Data in ROS 2


los 3 tipos de mensajes que el robot recibie  al interactura con el mundo exterior
sensor_msgs/LaserScan
sensor_msgs/Imagen
sensor_msgs/PointCloud2



## sensor_msgs/LaserScan

- la estructura del mensaje puede analizarlo aqui : [documentación oficial](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html) 

empecemos a trabajar:

creamos  el medio donde se va desarrollar:
```sh
cd ~/ros2_ws/src/
ros2 pkg create --build-type ament_python basics_sensor_data
cd ~/ros2_ws/
colcon build
source install/setup.bash
mkdir src/basics_sensor_data/rviz
touch src/basics_sensor_data/basics_sensor_data/mona_lisa_approach.py
chmod +x src/basics_sensor_data/basics_sensor_data/mona_lisa_approach.py
```
ademas debe abrir rviz2 y colocar 2 herramientas image, lase, y cloud2 luego guardar en el directorio rviz con el nombre de basic_sensor_data.rviz

pegar el siguiente codigo en  mona_lisa_approach.py


```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class MonaLisaApproach(Node):

    def __init__(self) -> None:
        super().__init__('mona_lisa_approach')
        self.subscription = self.create_subscription(
            LaserScan,
            '/deepmind_robot1/laser_scan',  
            self.laser_callback,
            10
        )

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10) 
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        self.mona_lisa_distance = None

    def laser_callback(self, msg: LaserScan) -> None:
        """Callback function for laser scan data"""

        # Find index corresponding to the direction directly in front of the robot
        middle_index = len(msg.ranges) // 2
        self.mona_lisa_distance = msg.ranges[middle_index]
        

    def timer_callback(self) -> None:
        if self.mona_lisa_distance > 0.5:  
            self.move_forward()
        else:
            self.stop_robot()
            rclpy.shutdown()

    def move_forward(self) -> None:
        """Move the robot forward"""

        print(f'MOVE FORWARD... Mona Lisa is {self.mona_lisa_distance:.2f} meters away')
        twist_msg = Twist()
        twist_msg.linear.x = 0.1
        self.publisher.publish(twist_msg)

    def stop_robot(self) -> None:
        """Stop the robot"""

        print(f'STOP... ')
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        self.publisher.publish(twist_msg)

def main(args=None) -> None:
    rclpy.init(args=args)
    mona_lisa_approach = MonaLisaApproach()
    rclpy.spin(mona_lisa_approach)
    mona_lisa_approach.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## sensor_msgs/Imagen

para ver los componentes de este mensaje:
- la estructura del mensaje puede analizarlo aqui : [documentación oficial](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html) 

crear el archivo color_detector.py y pegar el siguiente codigo

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ColorDetector(Node):

    def __init__(self) -> None:
        super().__init__('color_detector')
        self.subscription = self.create_subscription(
            Image,
            '/deepmind_robot1/deepmind_robot1_camera/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg: Image) -> None:
        """
        Callback function to process image data
        """

        # Retrieve image properties
        height = msg.height
        width = msg.width
        data = msg.data
        byte_depth = 3  # Each pixel consists of 3 bytes (RGB)
 
        # Calculate center pixel index
        center_index = ((height // 2) * width + (width // 2)) * byte_depth

        # Calculate color component indices based on center pixel
        if center_index % byte_depth == 0:
            red_component_index = center_index
            green_component_index = center_index + 1
            blue_component_index = center_index - 1
        if center_index % byte_depth == 1:
            green_component_index = center_index
            blue_component_index = center_index + 1
            red_component_index = center_index - 1
        if center_index % byte_depth == 2:
            blue_component_index = center_index 
            red_component_index = center_index + 1
            green_component_index = center_index - 1

        # Print RGB values of center pixel
        print(f'\nRGB({data[red_component_index]}, {data[green_component_index]}, {data[blue_component_index]}) ')
        
        # Detect colors based on RGB values
        self.detect_color(data[red_component_index], data[green_component_index], data[blue_component_index])

    def detect_color(self, red: int, green: int, blue: int) -> None:
        """
        Detects color based on RGB values
        """

        if (240 <= red <= 255) and (55 <= green <= 80) and (0 <= blue <= 10):
            print(f'ORANGE COLOR DETECTED ....')

        if (240 <= red <= 255) and (210 <= green <= 255) and (0 <= blue <= 10):
            print(f'YELLOW COLOR DETECTED ....')

        if (0 <= red <= 10) and (230 <= green <= 255) and (0 <= blue <= 10):
            print(f'GREEN COLOR DETECTED ....')

        if (10 <= red <= 40) and (90 <= green <= 125) and (130 <= blue <= 185):
            print(f'BLUE COLOR DETECTED ....')


def main(args=None) -> None:
    rclpy.init(args=args)
    color_detector = ColorDetector()
    rclpy.spin(color_detector)
    color_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    ```


## sensor_msgs/PointCloud2


para ver los componentes de este mensaje:
- la estructura del mensaje puede analizarlo aqui : [documentación oficial](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) 
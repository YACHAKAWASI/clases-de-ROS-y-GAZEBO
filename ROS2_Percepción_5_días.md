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



# UNIT 3:   Image Processing

Importancia de OpenCV en el pre procesamiento de imagenes para que de esa manera este liesto para el procesamiento en modelo y aprendizaje , la siguiente  imagen es un esquema donde el paquete  cv_bridge enlaza el entorno de OpenCV con el sistema ROS2.
![Descripción de la imagen](http://wiki.ros.org/cv_bridge?action=AttachFile&do=get&target=cvbridge.png)

tendremos que crear 2 scripts:

blob_point_pub.py: detecta blob y publica su posición.
blob_tracker.py: El rastreador de blobs se encarga de guiar al robot hacia la puerta de salida.
Cree un paquete ROS 2 llamado blob_tracking y los 2 scripts:

creamos un nuevo paquete :
```sh
cd ~/ros2_ws/src/
ros2 pkg create --build-type ament_python blob_tracking
cd ~/ros2_ws/
colcon build
source install/setup.bash
cd src/blob_tracking/blob_tracking
touch blob_point_pub.py blob_tracker.py 
chmod +x *.py
cd ~/ros2_ws/
```


blob_point_pub.py

Este código es un nodo de ROS 2 escrito en Python que se encarga de detectar un objeto de color naranja en una imagen capturada por una cámara, calcular la posición del objeto (blob) y publicar esta posición en un topic de ROS.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2
import numpy as np
from cv_bridge import CvBridge
from typing import Tuple

class BlobPointPublisher(Node):
    def __init__(self):
        super().__init__('blob_point_publisher')
        self.subscription = self.create_subscription(
            Image,
            '/deepmind_robot1/deepmind_robot1_camera/image_raw',  
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Point, '/point_blob', 10)
        self.cv_bridge = CvBridge()

    def image_callback(self, msg: Image) -> None:
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error("Error converting ROS Image to OpenCV format: {0}".format(e))
            return

        # Convert BGR image to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define lower and upper bounds for orange color in HSV
        lower_orange = np.array([3, 0, 233])
        upper_orange = np.array([9, 255, 255])

        # Create a binary mask for orange color
        mask = cv2.inRange(hsv_image, lower_orange, upper_orange)

       # Draw coordinate frame
        self.draw_frame(cv_image)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw a red circle around the blob contour
        for contour in contours:
            self.draw_blob_contour_circle(cv_image, contour)

        # If no contours are found, return
        if not contours:
            return

        # Find the largest contour
        blob_contour = max(contours, key=cv2.contourArea)

        # Calculate centroid of the contour
        moment = cv2.moments(blob_contour)
        if moment["m00"] != 0:             # Area of the object
            blob_x = float(moment["m10"] / moment["m00"])  # x-coordinate of centroid
            blob_y = float(moment["m01"] / moment["m00"])  # y-coordinate of centroid

            # x and y coordinates of the blob are computed relative to the center of the image
            x, y = self.get_blob_relative_position(cv_image, blob_x, blob_y)

            # Publish Blob
            self.publish_blob(x, y)

            # Draw detected blobs
            self.draw_keypoints(cv_image, [(blob_x, blob_y)])

        # Display the image (optional)
        cv2.imshow("Image", cv_image)
        cv2.waitKey(1)

    def draw_frame(self,
                image,
                dimension=0.1,      #- dimension relative to frame size
                line=2              #- line's thickness
        ) -> None:
        """ Draw X Y coordinate frame at the center of the image"""
        
        rows = image.shape[0]
        cols = image.shape[1]
        size = min([rows, cols])
        center_x = int(cols/2.0)
        center_y = int(rows/2.0)
        
        line_length = int(size*dimension)
        
        #-- X
        image = cv2.line(image, (center_x, center_y), (center_x+line_length, center_y), (0,0,255), line)
        #-- Y
        image = cv2.line(image, (center_x, center_y), (center_x, center_y+line_length), (0,255,0), line)
        

    def draw_blob_contour_circle(self, img, contour) -> None:
        """ Draw a red circle around the detected blob """
        # Get the bounding rectangle of the contour
        x, y, w, h = cv2.boundingRect(contour)
        
        # Calculate the center of the rectangle
        center_x = x + w // 2
        center_y = y + h // 2
        
        # Calculate the radius of the circle based on the contour size
        radius = max(w, h) // 2

        # Draw a red circle around the center
        cv2.circle(img, (center_x, center_y), radius, (0, 0, 255), 2)


    def get_blob_relative_position(self, image, x, y) -> Tuple[float, float]:
        """ Get blob position relative to the coordinate frame placed at the center of the image"""
        # The shape attribute of a NumPy array returns a tuple representing the dimensions of the array. 
        # For an image, the shape tuple consists of 3 elements: (height, width, channels).

        rows = float(image.shape[0]) # height
        cols = float(image.shape[1]) # width

        #  Coordinates of the center of the image 
        center_x    = 0.5*cols
        center_y    = 0.5*rows

        # The x and y coordinates of the keypoint are computed relative to the center of the image
        x = (x - center_x)/(center_x)
        y = (y - center_y)/(center_y)

        return x,y
    
    def publish_blob(self, x, y) -> None:
        """ Publish the blob position to /point_blob """
        # Create a Point message
        point_msg = Point()
        point_msg.x = x
        point_msg.y = y

        # Publish the Point message
        self.publisher.publish(point_msg)


    def draw_keypoints(self, img, keypoints) -> None:
        """ Draw the detected blob keypoints in red """
        # Draw red circles at the detected blob keypoints
        for kp in keypoints:
            # Convert keypoints to integers
            kp = (int(kp[0]), int(kp[1]))
            cv2.circle(img, kp, 5, (0, 0, 255), -1)


def main(args=None) -> None:
    rclpy.init(args=args)
    blob_point_publisher = BlobPointPublisher()
    rclpy.spin(blob_point_publisher)
    blob_point_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```



blob_tracker.py

Este código es otro nodo de ROS 2, denominado BlobTracker, que se encarga de seguir un objeto detectado (blob) utilizando la información de posición publicada por el nodo BlobPointPublisher. Aquí tienes una descripción detallada de sus funciones principales:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist

class BlobTracker(Node):
    def __init__(self):
        super().__init__('blob_tracker')
        self.subscription = self.create_subscription(
            Point,
            '/point_blob',
            self.blob_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.angular_gain = 0.3  
        self.no_blob_detected = True
        self.timer = self.create_timer(1.0, self.rotate_continuous)  

    def blob_callback(self, msg):
        # Extract blob coordinates
        blob_x = msg.x
        blob_y = msg.y

        # Define linear velocity
        linear_x = 0.3  

        if blob_x != 0.0 or blob_y != 0.0:
            # Adjust angular velocity based on blob's position
            angular_vel = -self.angular_gain * blob_x
            # Clip angular velocity to [-1, 1] range
            angular_vel = max(min(angular_vel, 1.0), -1.0)

            # Adjust linear velocity based on blob's position
            if blob_y < 0.8:
                linear_vel = linear_x
            else:
                linear_vel = 0.0  # Stop the robot
            self.no_blob_detected = False
        else:
            return

        self.pub_velocities(linear_vel, angular_vel)

    def rotate_continuous(self):
        # Rotate continuously
        if self.no_blob_detected:
            twist_msg = Twist()
            twist_msg.angular.z = 0.3  # Adjust angular velocity as needed
            self.publisher.publish(twist_msg)
            self.get_logger().warn("No blobs detected yet ... Rotating continuously")

    def pub_velocities(self, linear, angular):
        # Create Twist message with linear and angular velocities
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular

        # Publish the Twist message
        self.publisher.publish(twist_msg)
        self.get_logger().info(f"Linear vel: {linear:.2f}, Angular vel: {angular:.2f}")


def main(args=None):
    rclpy.init(args=args)
    blob_tracker = BlobTracker()
    rclpy.spin(blob_tracker)
    blob_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String, Bool, Float32MultiArray
from dynamixel_sdk import *
from cv_bridge import CvBridge
import array
import math

from .KinematicCalculator import KinematicCalculator

import cv2

class PhantomController2(Node):
    def __init__(self):
        super().__init__('phantom_controller2')

        self.movement_mode = 'Lineal'
        self.joints = np.array([0, 0, 0, 0, 0], dtype=float)
        self.position = np.array([55 , 65 ,185 ,  math.radians(-25), 0], dtype=float)
        self.image = None
        self.start = False

        self.movement_mode_subscriber = self.create_subscription(String, '/robot/movement_mode', self.movement_mode_callback, 10)
        self.coordinates_subscriber = self.create_subscription(Float32MultiArray, '/robot/coordinates', self.coordinates_callback, 10)
        self.start_subscriber = self.create_subscription(Bool, '/robot/start', self.start_callback, 10)

        self.joints_publisher = self.create_publisher(Float32MultiArray, '/robot/joints', 10)
        self.position_publisher = self.create_publisher(Float32MultiArray, '/robot/position', 10)
        self.camera_publisher = self.create_publisher(Image, '/robot/camera', 10)

        self.test_publisher = self.create_publisher(JointState, '/coppelia/joint_commands', 10)

        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()    

        self.kinematic_calculator = KinematicCalculator()

        timer_period = 0.1
        self.imagetimer = self.create_timer(timer_period, self.publish_camera_image)
        self.timer = self.create_timer(timer_period, self.publish_robot_info)

        self.bridge = CvBridge()

    def movement_mode_callback(self, msg):
        self.movement_mode = msg.data

    def start_callback(self, msg):
        self.start = msg.data

    def coordinates_callback(self, msg):
        if self.start:
            if self.movement_mode == 'Lineal':
                position = self.position + np.array(msg.data)
                try :
                    self.joints[:4] = np.clip(self.kinematic_calculator.calcRobotJoints(position), -np.pi, np.pi)
                    print('-'*15)
                    print('\n', self.joints, '\n')
                    print('-'*15)
                    self.position = position
                except ValueError as e:
                    print(f"Error al calcular los joints: {e}")
            elif self.movement_mode == 'Articular':
                data = np.array(msg.data)*1.1*np.pi/180
                self.joints = np.clip(self.joints + np.array(data), -np.pi, np.pi)
                self.position = self.kinematic_calculator.calcRobotPosition(self.joints[:4])
                print('\n', self.position, '\n')
            else:
                print('Invalid movement mode')

    def publish_robot_info(self):
        # joints_message = Float32MultiArray()
        print(self.joints)
        print(self.position)

        joints_message = JointState()
        joints_message.name = ["./arm_shoulder_pan_joint", "./arm_shoulder_lift_joint", "./arm_elbow_flex_joint", "./arm_wrist_flex_joint", "./gripperClose_joint"]

        if isinstance(self.joints, list):
            try:
                self.joints = [float(j) for j in self.joints]  # Convertir cada elemento a float
            except ValueError as e:
                print(f"Error al convertir self.joints a floats: {e}")


        joints_message.position = array.array('d', self.joints)
        joints_message.velocity = array.array('d', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        joints_message.effort = array.array('d', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.test_publisher.publish(joints_message)

        joints_message = Float32MultiArray()
        joints_message.data = self.joints.tolist()
        self.joints_publisher.publish(joints_message)

        position_message = Float32MultiArray()
        position_message.data = self.position.tolist()
        self.position_publisher.publish(position_message)

    def publish_camera_image(self):
        ret, frame = self.cap.read()
        if ret:
            image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.camera_publisher.publish(image_message)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()
    
    def sum_arrays(self, a, b):
        return array.array([a + b for a, b in zip(a, b)])

def main(args=None):
    rclpy.init(args=args)
    phantom_controller = PhantomController2()
    rclpy.spin(phantom_controller)
    phantom_controller.destroy_node()
    rclpy.shutdown()
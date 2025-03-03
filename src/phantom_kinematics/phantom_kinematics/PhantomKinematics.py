import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool, Float32MultiArray
import array
import math

from .KinematicCalculator import KinematicCalculator

import cv2

class PhantomKinematics(Node):
    def __init__(self):
        super().__init__('phantom_controller2')

        self.movement_mode = 'Lineal'
        self.joints = np.array([10, 0, 0, 0, 0], dtype=float)
        self.position = np.array([0 , 0 ,442 ,  90, 0], dtype=float)
        # self.position = np.array([55 , 65 ,185 ,  -25, 0], dtype=float)
        self.image = None
        self.start = False

        # Entradas desde el controlador local
        self.movement_mode_subscriber = self.create_subscription(String, '/robot/movement_mode', self.movement_mode_callback, 10)
        self.coordinates_subscriber = self.create_subscription(Float32MultiArray, '/robot/coordinates', self.coordinates_callback, 10)
        self.start_subscriber = self.create_subscription(Bool, '/robot/start', self.start_callback, 10)
        
        # Entrada desde el robot 
        self.joints_subscriber = self.create_subscription(Float32MultiArray, '/robot/joint_states', self.joints_callback, 10)


        # Salidas hacia el controlador local
        self.position_publisher = self.create_publisher(Float32MultiArray, '/robot/position', 10)

        # Salida hacia el robot
        self.command_publisher = self.create_publisher(JointState, '/robot/joint_commands', 10)

        self.kinematic_calculator = KinematicCalculator()


    def movement_mode_callback(self, msg):
        self.movement_mode = msg.data

    def start_callback(self, msg):
        self.start = msg.data

    def coordinates_callback(self, msg):
        if self.start:
            if self.movement_mode == 'Lineal':
                position = self.position + np.array(msg.data)
                try :
                    joints = np.clip(self.kinematic_calculator.calc_robot_joints(position), -150, 150)
                    self.joints = np.append(joints, self.kinematic_calculator.get_gripper_position(msg.data[3]))
                    self.publish_robot_info(self.joints)
                    self.position = position
                    self.joints = joints
                except ValueError as e:
                    print(f"Error al calcular los joints: {e}")
            elif self.movement_mode == 'Articular':
                data = np.array(msg.data)*1.1
                joints =self.joints + np.array(data) #np.clip(self.joints + np.array(data), -150, 150)
                self.joints = joints.copy()
                # self.joints[4] = self.kinematic_calculator.get_gripper_position(msg.data[3])
                self.publish_robot_info(self.joints)
                self.position = self.kinematic_calculator.calc_robot_position(joints[:4])
                
                
            else:
                print('Invalid movement mode')

    def publish_robot_info(self, joints):
        joints_message = JointState()
        joints_message.name = ["./arm_shoulder_pan_joint", "./arm_shoulder_lift_joint", "./arm_elbow_flex_joint", "./arm_wrist_flex_joint", "./gripper_close_joint"]

        if isinstance(joints, list):
            try:
                joints = [float(j) for j in joints]  # Convertir cada elemento a float
            except ValueError as e:
                print(f"Error al convertir joints a floats: {e}")

        joints_message.position = array.array('d', joints)
        joints_message.velocity = array.array('d', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        joints_message.effort = array.array('d', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.command_publisher.publish(joints_message)
        
    def joints_callback(self, msg):
        position_message = Float32MultiArray()
        data = self.kinematic_calculator.calc_robot_position(msg.data[:4])
        position_message.data = data.tolist()
        self.position_publisher.publish(position_message)


    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()
    
    


def main(args=None):
    print("El nodo phantom_kinematics ha sido iniciado")
    rclpy.init(args=args)
    phantom_controller = PhantomKinematics()
    rclpy.spin(phantom_controller)
    phantom_controller.destroy_node()
    rclpy.shutdown()
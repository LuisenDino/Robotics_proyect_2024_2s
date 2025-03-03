import sys
import rclpy
from rclpy.node import Node
import pygame

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QComboBox
from PyQt5.QtGui import QImage, QPixmap

from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Float32MultiArray
from cv_bridge import CvBridge
# import cv2

import threading

class RobotInterface(Node, QWidget):
    def __init__(self):
        Node.__init__(self, 'robot_interface')
        QWidget.__init__(self)

        self.running = True
        self.mode_selector = None
        self.movement_selector = None
        self.joystick = None
        self.joystick_data = [0, 0, 0, 0, 0]
        self.joystick_buttons = []

        self.left_joystick_label = None
        self.right_joystick_label = None
        self.triggers_label = None

        self.camera_display = None  

        self.start = False

        # Publishers
        self.start_publisher = self.create_publisher(Bool, '/robot/start',  10)
        self.coordinates_publisher = self.create_publisher(Float32MultiArray, '/robot/coordinates', 10) # x, y, z, phi, o j1, j2, j3, j4, j5 según el modo
        self.movement_mode = self.create_publisher(String, '/robot/movement_mode', 10)

        # Subscribers
        self.camera_subscriber = self.create_subscription(Image, '/robot/camera', self.camera_callback, 10)
        self.position_subscriber = self.create_subscription(Float32MultiArray, '/robot/position', self.position_callback, 10)
        self.joints_subscriber = self.create_subscription(Float32MultiArray, '/robot/joints', self.joints_callback, 10)

        self.camera_subscriber 
        self.position_subscriber
        self.joints_subscriber

        self.bridge = CvBridge()

        self.ros_thread = threading.Thread(target=self.ros_spin)
        self.ros_thread.start()

        self.init_joystick()
        self.joystick_thread = threading.Thread(target=self.joystick_thread)

        self.joystick_thread.start()

        self.init_ui()
        
        
    def ros_spin(self):
        rclpy.spin(self)

    def init_ui(self):
        layout = QVBoxLayout()
        self.setWindowTitle('Robot Interface')
        
        status_layout = QHBoxLayout()
        self.state_label = QLabel('Estado: desconocido')
        status_layout.addWidget(self.state_label)
        self.mode_label = QLabel('Modo: Manual')
        status_layout.addWidget(self.mode_label)
        layout.addLayout(status_layout)


        self.mode_selector = QComboBox()
        self.mode_selector.addItems(['Manual', 'Automatico'])
        layout.addWidget(self.mode_selector)
        self.mode_selector.currentIndexChanged.connect(self.change_mode)

        self.movement_selector = QComboBox()
        self.movement_selector.addItems(["Lineal", "Articular"])
        layout.addWidget(self.movement_selector)

        status_layout = QHBoxLayout()
        self.start_button = QPushButton("Iniciar")
        self.start_button.clicked.connect(self.start_robot)
        status_layout.addWidget(self.start_button)
        

        self.stop_button = QPushButton("Detener")
        self.stop_button.clicked.connect(self.stop_robot)
        status_layout.addWidget(self.stop_button)
        layout.addLayout(status_layout)

        #Camara
        self.camera_label = QLabel("Cámara")
        layout.addWidget(self.camera_label)
        self.camera_display = QLabel()
        layout.addWidget(self.camera_display)

        # Información del robot
        status_layout = QHBoxLayout()
        self.position_label = QLabel("Posición:\n x: Desconocido\n y: Desconocido\n z: Desconocido\n phi: Desconocido\n gripper: Desconocido")
        status_layout.addWidget(self.position_label)

        self.joints_label = QLabel("Articulaciones:\n J1: Desconocido\n J2: Desconocido\n J3: Desconocido\n J4: Desconocido\n J5: Desconocido")
        status_layout.addWidget(self.joints_label)

        layout.addLayout(status_layout)

        self.setLayout(layout)

        joystick_layout = QVBoxLayout()
        self.joystick_label = QLabel("Joystick:")
        joystick_layout.addWidget(self.joystick_label)
        hor_layout = QHBoxLayout()
        self.left_joystick_label = QLabel("Left Joystick:\n x: 0.0\n y: 0.0")
        hor_layout.addWidget(self.left_joystick_label)
        self.right_joystick_label = QLabel("Right Joystick:\n x: 0.0\n y: 0.0")
        hor_layout.addWidget(self.right_joystick_label)
        self.triggers_label = QLabel("Triggers:\n R2: 0.0\t L2: 0.0")
        layout.addLayout(joystick_layout)
        joystick_layout.addLayout(hor_layout)
        layout.addWidget(self.triggers_label)

        self.show()

    def joystick_thread(self):
        while rclpy.ok() and self.running:
            self.read_joystick()

    def init_joystick(self):
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            print('No se detectaron joysticks')
            self.joystick = None
            return

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print('Joystick detectado:', self.joystick.get_name())
        


    def read_joystick(self):
        if self.mode_selector is not None and self.mode_selector.currentText() == 'Manual' and self.joystick is not None:            
            
            pygame.event.pump()
            deadzone = 0.05
            left_x = self.joystick.get_axis(0) if abs(self.joystick.get_axis(0)) > deadzone else 0.0
            left_y = -1*self.joystick.get_axis(1) if abs(self.joystick.get_axis(1)) > deadzone else 0.0
            right_x = self.joystick.get_axis(3) if abs(self.joystick.get_axis(3)) > deadzone else 0.0
            right_y = -1*self.joystick.get_axis(4) if abs(self.joystick.get_axis(4)) > deadzone else 0.0
            l_2 = (self.joystick.get_axis(2)+1)/2 if abs(self.joystick.get_axis(2)) > deadzone else 0.0 # normalizar los botones entre 0 y 1
            r_2 = (self.joystick.get_axis(5)+1)/2 if abs(self.joystick.get_axis(5)) > deadzone else 0.0 # normalizar los botones entre 0 y 1

            gripper = r_2 - l_2
            self.joystick_data = [left_x, left_y, right_x, right_y, gripper]
            self.joystick_buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]

            

            if self.joystick_buttons[10]:
                if self.start:
                    self.stop_robot()
                else:
                    self.start_robot()

            self.send_command()

            if (self.left_joystick_label is not None and self.right_joystick_label is not None and self.triggers_label is not None):
                self.left_joystick_label.setText(f"Left Joystick:\n x: {left_x:.2f}\n y: {left_y:.2f}")
                self.right_joystick_label.setText(f"Right Joystick:\n x: {right_x:.2f}\n y: {right_y:.2f}")
                self.triggers_label.setText(f"Triggers:\n R2: {r_2:.2f}\n L2: {l_2:.2f}")

            pygame.time.wait(100)

        
    def send_command(self):
        if self.movement_selector is None:
            return
        
        msg = String()
        if self.movement_selector.currentText() == 'Lineal':
            phi, z, x, y, gripper = self.joystick_data[:5]
            msg.data = 'Lineal'
            self.movement_mode.publish(msg)
            self.coordinates_publisher.publish(Float32MultiArray(data=[x, y, z, phi, gripper]))
        else:
            j1, j2, j3, j4, j5 = self.joystick_data[:5]
            msg.data = 'Articular'
            self.movement_mode.publish(msg)
            self.coordinates_publisher.publish(Float32MultiArray(data=[j1, j2, j3, j4, j5]))

    def camera_callback(self, data):
        if self.camera_display is None:
            return
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        height, width, channels = cv_image.shape
        max_height = 480
        max_width = 640
        if height > max_height or width > max_width:
            scaling_factor = min(max_width / width, max_height / height)
            width = int(width * scaling_factor)
            height = int(height * scaling_factor)
            cv_image = cv2.resize(cv_image, (width, height))
        bytes_per_line = channels * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        self.camera_display.setPixmap(QPixmap.fromImage(q_image))

    def position_callback(self, data):
        x, y, z, phi, gripper = data.data
        self.position_label.setText(f"Posición:\n x: {x:.2f}\n y: {y:.2f}\n z: {z:.2f}\n phi: {phi:.2f}, gripper: {gripper:.2f}")

    def joints_callback(self, data):
        j1, j2, j3, j4, j5 = data.data
        self.joints_label.setText(f"Articulaciones:\n J1: {j1:.2f}\n J2: {j2:.2f}\n J3: {j3:.2f}\n J4: {j4:.2f}\n J5: {j5:.2f}")

    def change_mode(self):
        mode = self.mode_selector.currentText()
        if mode == 'Manual':
            self.mode_label.setText('Modo: Manual')
        else:
            self.mode_label.setText('Modo: Automatico')

    def start_robot(self):
        self.state_label.setText('Estado: Iniciando')
        self.start = True
        msg = Bool()
        msg.data = True
        self.start_publisher.publish(msg)
    
    def stop_robot(self):
        self.state_label.setText('Estado: Deteniendo')
        self.start = False
        msg = Bool()
        msg.data = False
        self.start_publisher.publish(msg)
        

    def closeEvent(self, event):
        self.running = False
        # rclpy.shutdown()
        event.accept()

    def destroy_node(self):
        self.running = False
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    node = RobotInterface()
    
    # Ejecutar la interfaz y el nodo
    app.exec_()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':  
    main()

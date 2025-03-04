import json
import os
import rclpy
import cv2


from dynamixel_sdk import *  # Importar la SDK de Dynamixel
from rclpy.node import Node  # Importar la clase Node de rclpy
from sensor_msgs.msg import JointState, Image  # Importar el mensaje JointState e Image de sensor_msgs
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge  # Importar la clase CvBridge de cv_bridge

ADDR_MX_TORQUE_ENABLE = 24  # Dirección de la dirección de habilitación del torque
ADDR_MX_GOAL_POSITION = 30  # Dirección de la posición objetivo
ADDR_MX_PRESENT_POSITION = 36  # Dirección de la posición actual
ADDR_TEMPERATURE_LIMIT = 11  # Dirección del límite de temperatura
ADDR_MAX_TORQUE = 34  # Dirección del torque máximo

BAUDRATE = 1000000  # Velocidad de transmisión de datos en bps
PROTOCOL_VERSION = 1.0  # Versión del protocolo utilizado
DEVICE_NAME = 'COM19'  # Puerto USB al que está conectado el controlador de Dynamixel


class PhantomController(Node):

    def __init__(self, device_name, baudrate, protocol_version):
        super().__init__('phantom_controller')
        
        # Configuración inicial del robot
        self.device_name = device_name
        self.baudrate = baudrate
        self.protocol_version = protocol_version

        self.portHandler = PortHandler(self.device_name)
        self.packetHandler = PacketHandler(self.protocol_version)

        if self.portHandler.openPort():
            print("Puerto Abierto con éxito")
        else:
            print("No se pudo abrir el puerto")
            quit()
        
        if self.portHandler.setBaudRate(self.baudrate):
            print("Baudrate configurada con éxito")
        else:
            print("No se pudo configurar el baudrate")
            quit()

                # Obtener la ruta del paquete instalado
        package_share_directory = get_package_share_directory('phantom_controller')

        # Ruta completa al archivo de configuración
        config_path = os.path.join(package_share_directory, 'config', 'config.json')
        
        self.config = None
        with open(config_path, 'r') as config_file:
            self.config = json.load(config_file)
            print(self.config)  
        print("Archivo de configuración cargado con éxito")

        for (joint_name, joint_information) in self.config.items():
            joint_id = joint_information['id']
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, joint_id, ADDR_MX_TORQUE_ENABLE, 1)
            self.check_configuration('torque', joint_name, dxl_comm_result, dxl_error)

            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, joint_id, ADDR_TEMPERATURE_LIMIT, joint_information['max_temperature'])
            self.check_configuration('límite de temperatura', joint_name, dxl_comm_result, dxl_error)

            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, joint_id, ADDR_MAX_TORQUE, joint_information['max_torque'])
            self.check_configuration('torque máximo', joint_name, dxl_comm_result, dxl_error)

            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, joint_id, ADDR_MX_GOAL_POSITION, self.degrees_to_dxl_position(joint_information['initial_angle']))
            self.check_configuration('posición inicial', joint_name, dxl_comm_result, dxl_error)

        # Inicialización de la cámara
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()    


        # Confifuración del nodo
        self.joints_publisher = self.create_publisher(JointState, 'robot/joint_states', 10)
        self.joints_subscriber = self.create_subscription(JointState, 'robot/joint_commands', self.joints_callback, 10)
        self.camera_publisher = self.create_publisher(Image, '/robot/camera', 10)
        self.create_timer(0.1, self.publish_camera_image)
        self.create_timer(0.1, self.publish_joints)


    def check_configuration(self, info, joint_name, result, error):
        if result != COMM_SUCCESS:
            print(f"Error al establecer {info} del motor {joint_name}: {self.packetHandler.getTxRxResult(result)}")
        elif error != 0:
            print(f"Error al establecer {info} del motor {joint_name}: {self.packetHandler.getRxPacketError(error)}")
        else:
            print(f"{info} del motor {joint_name} configurado con éxito")

    def move_to_position(self, joint_name, position):
        joint_id = self.config[joint_name]['id']
        position = self.degrees_to_dxl_position(position)
        self.packetHandler.write4ByteTxRx(self.portHandler, joint_id, ADDR_MX_GOAL_POSITION, position)
        

    def degrees_to_dxl_position(self, degrees):
        return int(degrees * 1023 / 300)

    def dxl_position_to_degrees(self, dxl_position):
        return dxl_position * 300 / 1023

    def joints_callback(self, msg):
        for (joint_name, position) in zip(msg.name, msg.position):
            if 'gripper_joint' in self.config[joint_name] and self.config[joint_name]['gripper_joint']:
                position = max(0, min(1, position))
                self.move_to_position(joint_name, position)  # El gripper solo puede ir de 0 a 1
            else:
                self.move_to_position(joint_name, position + 150)  # los 150 grados se añaden porque es la que se define como posición 0 en la cinemática directa

    def publish_joints(self):
        joint_state = JointState()
        joint_state.name = []
        joint_state.position = []
        joint_state.velocity = []
        joint_state.effort = []

        for joint_name in self.config.keys():
            joint_id = self.config[joint_name]['id']
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, joint_id, ADDR_MX_PRESENT_POSITION)
            
            joint_state.name.append(joint_name)
            joint_state.position.append(self.dxl_position_to_degrees(dxl_present_position))

        self.joints_publisher.publish(joint_state)

    def publish_camera_image(self):
        ret, frame = self.cap.read()
        if ret:
            image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.camera_publisher.publish(image_message)

    def destroy_node(self):
        self.cap.release()
        for joint_name in self.config.keys():
            joint_id = self.config[joint_name]['id']
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, joint_id, ADDR_MX_TORQUE_ENABLE, 0)
            self.check_configuration('torque', joint_name, dxl_comm_result, dxl_error)
        self.portHandler.closePort()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    phantom_controller = PhantomController(DEVICE_NAME, BAUDRATE, PROTOCOL_VERSION)
    rclpy.spin(phantom_controller)
    phantom_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
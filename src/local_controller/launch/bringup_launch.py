from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Obtener ruta del launch de Coppelia en el Paquete B
    coppelia_launch_path = os.path.join(
        get_package_share_directory('phantom_coppelia'),  # Nombre del paquete que abre Coppelia
        'launch',
        'bringup_launch.py'  # Nombre del archivo launch que abre Coppelia
    )

    return LaunchDescription([
        # Incluir el launch que abre Coppelia
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(coppelia_launch_path),
        ),

        # Lanzar la interfaz gráfica del Paquete B
        Node(
            package='local_controller',
            executable='local_controller',
            name='local_controller',
            output='screen'
        ),

        # # Lanzar el nodo del Paquete A para comunicarse con Coppelia
        Node(
            package='phantom_kinematics',
            executable='phantom_kinematics',
            name='phantom_kinematics',
            output='screen'
        ),
    ])

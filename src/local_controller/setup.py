from setuptools import find_packages, setup

package_name = 'local_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs', 'sensor_msgs', 'PyQt5', 'pygame', 'cv_bridge'],
    zip_safe=True,
    maintainer='Luis Enrique Carmona',
    maintainer_email='lucarmonaa@unal.edu.co',
    description='Este paquete permite leer la información de un joystic de PS5 y envía datos de movimiento, modo y status. Tiene una GUI para mostrar información del robot',
    license='MIT license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'local_controller = local_controller.RobotInterface:main',
        ],
    },
)

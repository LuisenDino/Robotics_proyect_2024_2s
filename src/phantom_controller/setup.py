from setuptools import find_packages, setup

package_name = 'phantom_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'cv_bridge', 'rclpy', 'sensor_msgs', 'std_msgs', 'dynamixel_sdk', 'numpy', 'roboticstoolbox-python'],
    zip_safe=True,
    maintainer='luisen',
    maintainer_email='luisen@todo.todo',
    description='TODO: Package description',
    license='MIT-License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'phantom_controller = phantom_controller.PhantomController:main',
        ],
    },
)

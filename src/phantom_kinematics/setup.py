from setuptools import find_packages, setup

package_name = 'phantom_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs', 'sensor_msgs', 'numpy', 'roboticstoolbox-python'],
    zip_safe=True,
    maintainer='luisen',
    maintainer_email='lucarmonaa@unal.edu.co',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'phantom_kinematics = phantom_kinematics.PhantomKinematics:main',
        ],
    },
)

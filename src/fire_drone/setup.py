from glob import glob

from setuptools import setup

package_name = 'fire_drone'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phantrungtien',
    maintainer_email='phantrungtien1034@gmail.com',
    description='ROS 2 + MAVROS cho drone ArduPilot cứu hỏa tự hành',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'takeoff_test = fire_drone.guided:main',
            'marker_detector = fire_drone.marker_detector:main',
            'fire_mission = fire_drone.fire_mission:main',
            'ground_range = fire_drone.ground_range:main',
        ],
    },
)

from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'cooperation_ned2_turtlebot4'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aline Almeida',
    maintainer_email='alineglalmeida@gmail.com',
    description='Pacote ROS2 para cooperacao entre NED2 e TurtleBot4.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'algoritmo_cooperacao = cooperation_ned2_turtlebot4.algoritmo_cooperacao:main',
            'pose_object_camera = cooperation_ned2_turtlebot4.pose_object_camera:main',
            'reposicionar_turtlebot = cooperation_ned2_turtlebot4.reposicionar_turtlebot:main',
            'sem_algoritmo = cooperation_ned2_turtlebot4.sem_algoritmo:main',
            'turtlebot_nav = cooperation_ned2_turtlebot4.turtlebot_nav:main',
        ],
    },
)

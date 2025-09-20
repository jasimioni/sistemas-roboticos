from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'jasimioni'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joao Andre Simioni',
    maintainer_email='jasimioni@gmail.com',
    description='Exercícios Sistemas Robóticos Inteligentes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_subscriber_printer = jasimioni.pose_subscriber_printer:main',
            'draw_square = jasimioni.draw_square:main',
            'move_to_pos = jasimioni.move_to_pos:main',
            'lidar_2d_plot = jasimioni.lidar_2d_plot:main',
        ],
    },
)

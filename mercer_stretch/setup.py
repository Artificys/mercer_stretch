from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'mercer_stretch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob('rviz/*')),
        ('share/' + package_name + '/config', glob('config/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hello-robot',
    maintainer_email='hello-robot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop = mercer_stretch.keyboard_listener:main',
            'mercer_node = mercer_stretch.mercer_node:main',
            'mercer_aruco = mercer_stretch.mercer_aruco:main',
            'aruco_to_nav_pose = mercer_stretch.aruco_to_nav_pose:main',
            'mercer_nav = mercer_stretch.mercer_nav:main',
            'mercer_audio = mercer_stretch.mercer_audio:main',
            'save_pointcloud = mercer_stretch.save_pointcloud:main',
        ],
    },
)

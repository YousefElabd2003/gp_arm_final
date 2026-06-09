import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Index entry for the package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Include package.xml
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include all files in the config directory, ensuring ros2_controllers.yaml is included
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yousef_elabd',
    maintainer_email='Yousef.Elabd@hotmail.com',
    description='Control configuration package for the 6-joint robotic arm.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_move = my_robot_control.test_move:main',
        ],
    },
)
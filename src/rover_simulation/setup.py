import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'rover_simulation'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),

        
        (
            'share/' + package_name + '/config/comparison',
            glob('config/comparison/*.yaml')
        ),        
        
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*'),
        ),
        (
            'share/' + package_name + '/config/experiments',
            glob('config/experiments/*.yaml')
        ),        
        
    ],
    install_requires=['setuptools','PyYAML'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='liorshema@gmail.com',
    description='Gazebo simulation and ROS integration for the agricultural rover',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nominal_kinematics_node = rover_simulation.nominal_kinematics_node:main',
            'straight_test = rover_simulation.straight_test:main',
            'state_comparator = rover_simulation.state_comparator:main',
            'experiment_runner = rover_simulation.experiment_runner:main',
        ],
    },
)
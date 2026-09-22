from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robot_models'

setup(
    name=package_name,
    version='0.0.1',

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
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],

    install_requires=[
        'setuptools',
        'numpy',
    ],

    zip_safe=True,

    maintainer='lior',
    maintainer_email='liorshema@gmail.com',

    description=(
        'Mathematical models for the mobile base, robotic arm, camera, '
        'target motion, and whole-body mobile manipulator system.'
    ),

    license='Apache-2.0',

    tests_require=['pytest'],
)
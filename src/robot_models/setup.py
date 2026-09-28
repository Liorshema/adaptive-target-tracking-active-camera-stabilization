from setuptools import find_packages, setup


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
    ],

    install_requires=[
        'setuptools',
        'numpy',
    ],

    zip_safe=True,

    maintainer='lior',
    maintainer_email='liorshema@gmail.com',

    description=(
        'Mathematical models for generic robot bases, manipulators, '
        'camera geometry, and whole-body systems.'
    ),

    license='Apache-2.0',

    tests_require=['pytest'],
)

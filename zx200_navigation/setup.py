import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'zx200_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'map'),glob('map/*.pgm')),
        (os.path.join('share', package_name, 'map'),glob('map/*.yaml')),
        (os.path.join('share', package_name, 'params'),glob('params/*.yaml')),
        (os.path.join('share', package_name, 'params'),glob('params/*.xml')),
        (os.path.join('share', package_name, 'parameters'),glob('parameters/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'scripts'),glob('scripts/*.py')),
        (os.path.join('share', package_name, 'srv'),glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ws',
    maintainer_email='ws@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'poseStamped2Odometry = zx200_navigation.poseStamped2Odometry:main',
            'odom_broadcaster = zx200_navigation.odom_broadcaster:main',
            'map_generator = zx200_navigation.map_generator:main',
            'fixed_odom_publisher = zx200_navigation.fixed_odom_publisher:main',
            'fixed_jointstates_publisher = zx200_navigation.fixed_jointstates_publisher:main'
        ],
    },
)

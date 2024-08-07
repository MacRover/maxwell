import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='veerash',
    maintainer_email='veerash@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_controller.py = drive.drive_controller:main',
            'sim_command_converter.py = drive.sim_command_converter:main',
            'vesc_controller.py = drive.vesc_controller:main',
            'keyboard_drive.py = drive.keyboard_drive:main',
            'heartbeat.py = drive.heartbeat:main',
            'vesc_status.py = drive.vesc_status:main',
            'xbox_drive.py = drive.xbox_drive:main',
            'xbox_drive_steer_test.py = drive.xbox_drive_steer_test:main',
        ],
    },
)

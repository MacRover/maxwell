from setuptools import setup
import os
from glob import glob

package_name = 'h264_decoder'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A ROS 2 package for decoding H.264 compressed images',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'h264_decoder_node = h264_decoder.h264_decoder_node:main',
        ],
    },
)

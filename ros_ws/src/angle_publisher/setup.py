from setuptools import find_packages, setup

package_name = 'angle_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nehan',
    maintainer_email='nehanmohammed06@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "keyboard_input.py = angle_publisher.keyboard_input:main",
            "angle_subscriber.py = angle_publisher.angle_subscriber:main",
        ],
    },
)

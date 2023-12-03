from setuptools import find_packages, setup

package_name = "keyboard_controller_sim"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="veerash",
    maintainer_email="veeresh.532000@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "keyboard_controller = keyboard_controller_sim.keyboard_drive_control:main",
            "xbox_controller = keyboard_controller_sim.xbox_drive_control:main",
        ],
    },
)

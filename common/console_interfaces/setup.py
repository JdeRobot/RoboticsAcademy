import os
from setuptools import setup
from glob import glob

package_name = "console_interfaces"

# Add here the package resources
data_files = [
    ("share/" + package_name, ["package.xml"]),
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
]

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Javier Izquierdo",
    maintainer_email="javizqh@gmail.com",
    description="JdeRobot console interface package",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)

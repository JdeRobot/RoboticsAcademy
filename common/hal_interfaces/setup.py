import os
from setuptools import setup
from glob import glob

package_name = "hal_interfaces"

# Add here the package resources
data_files = [
    ("share/" + package_name, ["package.xml"]),
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    (os.path.join("share", package_name, "models"), glob("models/*")),
]

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Óscar Martínez",
    maintainer_email="oscar.robotics@tutanota.com",
    description="JdeRobot hardware abstraction layer package",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)

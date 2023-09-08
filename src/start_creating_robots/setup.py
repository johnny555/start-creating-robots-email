from setuptools import find_packages, setup
from os.path import join
from glob import glob

package_name = "start_creating_robots"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (join('share', package_name, 'launch'), glob('launch/*launch.py')),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="john@soundelve.com",
    description="Example Robot System",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)

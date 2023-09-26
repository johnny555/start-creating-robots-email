from setuptools import find_packages, setup
from os.path import join
from glob import glob, iglob

package_name = "start_creating_robots"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/" + package_name, ["package.xml"]),

        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        
        (join('share', package_name, 'launch'), glob('launch/*launch.py')),
        
        (join('share', package_name, 'config'), glob('config/*.yaml')),
        (join('share', package_name, 'config'), glob('config/*.rviz')),
        
        (join('share', package_name, 'worlds_and_models'), glob('worlds_and_models/*.*')),

        (join('share', package_name, 'worlds_and_models/Cafe'), glob('worlds_and_models/Cafe/*.*')),
        (join('share', package_name, 'worlds_and_models/Cafe/materials/textures'), 
                glob('worlds_and_models/Cafe/materials/textures/*.jpg')),
        (join('share', package_name, 'worlds_and_models/Cafe/meshes'), 
                glob('worlds_and_models/Cafe/meshes/*.dae')),

        (join('share', package_name, 'worlds_and_models/krytn'), glob('worlds_and_models/krytn/*.*')),
        (join('share', package_name, 'worlds_and_models/krytn/materials/textures'), 
                glob('worlds_and_models/krytn/materials/textures/*.jpg')),
        (join('share', package_name, 'worlds_and_models/krytn/materials/scripts'), 
                glob('worlds_and_models/krytn/materials/scripts/*.material')),
        (join('share', package_name, 'worlds_and_models/krytn/meshes'), 
                glob('worlds_and_models/krytn/meshes/*.dae')),
        


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

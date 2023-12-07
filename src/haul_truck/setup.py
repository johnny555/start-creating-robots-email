from setuptools import find_packages, setup
from os.path import join
from glob import glob

package_name = 'haul_truck'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (join('share', package_name, 'worlds_and_models/haul_truck'), glob('worlds_and_models/haul_truck/*.*')),
        (join('share', package_name, 'worlds_and_models/haul_truck/meshes'), 
                glob('worlds_and_models/haul_truck/meshes/*.*')),
        (join('share', package_name, 'worlds_and_models'), glob('worlds_and_models/*.*')),
        (join('share', package_name, 'worlds_and_models/haul_truck/materials/textures/'), 
                glob('worlds_and_models/haul_truck/materials/textures/*.png')),
        (join('share', package_name, 'config'), glob('config/*.yaml')),

        (join('share', package_name, 'worlds_and_models/falling_rock1'), glob('worlds_and_models/falling_rock1/*.*')),        
        (join('share', package_name, 'worlds_and_models/falling_rock1/meshes'), 
                glob('worlds_and_models/falling_rock1/meshes/*.*')),
        (join('share', package_name, 'worlds_and_models/falling_rock1/materials/textures/'), 
                glob('worlds_and_models/falling_rock1/materials/textures/*.jpg')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='badwolf.johnnyv@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'haul_truck = haul_truck.haul_truck:main'
        ],
    },
)

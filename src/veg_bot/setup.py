from setuptools import find_packages, setup
from os.path import join, dirname
from glob import glob

def get_all_files_in_path(glob_string, path_string):
    return [ (join(path_string, dirname(g)), [g]) for g in glob(glob_string, recursive=True)]


package_name = 'veg_bot'

def create_datafiles():
    df = [ ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
            (join('share/' , package_name,"launch"), glob("launch/*.*")),
            (join('share/',  package_name, "config"), glob("config/*.*")),
            ('share/' + package_name, ['package.xml']),
            ]
    res = get_all_files_in_path(glob_string="worlds_and_models/**/*.*", path_string=join("share",package_name))
    df.extend(res) 
    return df

#df = df + res

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=create_datafiles(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='badwolf.johnnyv@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'veg_bot = veg_bot.veg_bot:main'
        ],
    },
)

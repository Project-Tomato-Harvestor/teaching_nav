from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'teaching_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kennybot',
    maintainer_email='kennybot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recorder_node = teaching_nav.recorder_node:main',
            'bag_player = teaching_nav.bag_player:main',
            'simple_planner = teaching_nav.simple_planner:main',
        ],
    },
)

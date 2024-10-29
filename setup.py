from setuptools import find_packages, setup

package_name = 'teaching_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/learning_mode_launch.py']),
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
            'recorder_node = learning_mode.recorder_node:main',
            'bag_player = learning_mode.bag_player:main',
            'simple_planner = learning_mode.simple_planner:main',
        ],
    },
)

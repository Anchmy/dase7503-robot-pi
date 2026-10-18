from glob import glob

from setuptools import find_packages, setup

package_name = 'game_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rsi-bd-11',
    maintainer_email='todo@todo.todo',
    description='Mission state machine for the DASE7503 logistics game',
    license='MIT',
    entry_points={
        'console_scripts': [
            'game_manager = game_manager.game_manager_node:main',
        ],
    },
)

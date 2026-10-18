from glob import glob

from setuptools import find_packages, setup

package_name = 'qr_scanner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rsi-bd-11',
    maintainer_email='todo@todo.todo',
    description='QR code scanner node for the DASE7503 logistics robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'qr_node = qr_scanner.qr_node:main',
        ],
    },
)

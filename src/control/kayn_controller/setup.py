from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'kayn_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(include=['kayn_controller', 'kayn_controller.*']),
    zip_safe=True,
    maintainer='Mohammed Azab',
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('lib', package_name), ['scripts/kayn_node']),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'kayn_node = kayn_controller.kayn_node:main',
        ],
    },
)

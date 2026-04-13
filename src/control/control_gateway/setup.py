import os

from setuptools import find_packages, setup
from glob import glob 

package_name = 'control_gateway'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fam Shihata',
    maintainer_email='fam@awadlouis.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        "console_scripts": [
            "control_gateway = control_gateway.control_gateway:main",
            "teleop_switcher = control_gateway.teleop_switcher:main",
        ],
    },
)

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'opp_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fam Shihata',
    maintainer_email='fam@awadlouis.com',
    description='Opponent tracking from object detection bounding boxes for F1TENTH autonomous racing',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'opp_tracker_node = opp_tracker.opp_tracker_node:main',
        ],
    },
)

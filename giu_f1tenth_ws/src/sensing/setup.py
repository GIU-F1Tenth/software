from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'sensing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add launch files if any
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='F1TENTH sensing package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lidar_pub = sensing.lidar_publisher.lidar_pub:main',
            'lidar_objdet = sensing.lidar_subscriber.lidar_objdet:main',
        ],
    },
)
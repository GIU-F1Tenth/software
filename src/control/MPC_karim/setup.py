import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'MPC_karim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'casadi', 'numpy', 'pyyaml'],
    zip_safe=True,
    maintainer='Karim Shousha',
    maintainer_email='karim.shousha.ks@gmail.com',
    description='Model Predictive Controller (MPC) for F1TENTH racing.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpc_karim_node = MPC_karim.mpc_karim_node:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'trajectory_planning'

setup(
    name=package_name,
    version='0.2.3',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Karim',
    maintainer='Fam Shihata',
    maintainer_email='fam@awadlouis.com',
    description='GIU F1Tenth Car\'s trajectory planning package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit_node_v1 = pure_pursuit_node.pure_pursuit_v1:main',
            'pure_pursuit_node_v2 = pure_pursuit_node.pure_pursuit_v2:main',
        ],
    },
)

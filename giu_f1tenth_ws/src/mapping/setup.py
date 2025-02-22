from setuptools import find_packages, setup

package_name = 'mapping'

setup(
    name=package_name,
    version='0.2.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fam',
    maintainer_email='fam@awadlouis.com',
    description='GIU F1Tenth Car\'s mapping package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mapping_node = frontier_search_node.frontier_exploration:main',
        ],
    },
)

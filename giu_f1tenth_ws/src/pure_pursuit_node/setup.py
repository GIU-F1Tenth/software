from setuptools import find_packages, setup

package_name = 'pure_pursuit_node'

setup(
    name=package_name,
    version='0.0.1',
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
    description='GIU F1Tenth Car\'s pure puruit calculation node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit_node = pure_pursuit_node.pure_pursuit_node:main',
        ],
    },
)

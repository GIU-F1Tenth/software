import os
from glob import glob

from setuptools import find_packages, setup

package_name = "reactive_gap_follower"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Karim Shousha",
    maintainer_email="karim.shousha.ks@gmail.com",
    description="Disparity-extender reactive gap follower for F1TENTH.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "reactive_gap_follower_node = reactive_gap_follower.reactive_gap_follower_node:main",
        ],
    },
)

from setuptools import find_packages, setup
import os
from glob import glob

package_name = "frame_publisher"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Rafael Farias",
    maintainer_email="rafaelfariasm@live.com",
    description="PRISM frame throttle + publisher node",
    license="MIT",
    entry_points={
        "console_scripts": [
            "frame_publisher_node = frame_publisher.node:main",
        ],
    },
)

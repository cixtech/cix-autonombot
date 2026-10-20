from setuptools import setup

package_name = "autonombot_mapping"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CIX Robotics team",
    maintainer_email="noreply@cixtech.com",
    description="Autonombot mapping with LDR SLAM",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mapping_with_known_poses = autonombot_mapping.mapping_with_known_poses:main",
        ],
    },
)

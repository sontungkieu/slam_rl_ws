from setuptools import setup

package_name = "rl_resource_manager"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "psutil"],
    zip_safe=True,
    maintainer="student",
    maintainer_email="student@example.com",
    description="RL-based resource manager demo for ROS2 nodes (OS course).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cpu_hog = rl_resource_manager.cpu_hog_node:main",
            "rl_resource_manager = rl_resource_manager.rl_resource_manager_node:main",
        ],
    },
)

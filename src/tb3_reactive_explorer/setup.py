from setuptools import setup

package_name = "tb3_reactive_explorer"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="student",
    maintainer_email="student@example.com",
    description="Minimal package for SLAM exploration + RL resource management (course project).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            'reactive_explorer = tb3_reactive_explorer.reactive_explorer_node:main'
        ],
    },
)

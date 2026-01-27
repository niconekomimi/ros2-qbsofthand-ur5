from setuptools import setup

package_name = "yolo_center_detector"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/yolo_center.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="ROS2 YOLO detector that returns bbox center pixel positions filtered by target list.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "yolo_center_node = yolo_center_detector.yolo_center_node:main",
        ],
    },
)

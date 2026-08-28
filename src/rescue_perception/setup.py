from setuptools import find_packages, setup

package_name = "rescue_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch",
         ["launch/detect_target.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wheeltec",
    maintainer_email="wheeltec@todo.todo",
    description="救援场景目标感知：YOLO 检测 + 深度投影输出 map 系目标位姿",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "detect_target = rescue_perception.detect_target:main",
        ],
    },
)

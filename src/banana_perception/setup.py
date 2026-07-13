from setuptools import find_packages, setup

package_name = "banana_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/perception.yaml"]),
        ("share/" + package_name + "/launch", ["launch/perception.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="vanillaturtlechips",
    maintainer_email="wkdqkdgud@gmail.com",
    description="YOLO 바나나 익음 4단계 감지 노드",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "perception = banana_perception.node:main",
            "fake_camera = banana_perception.fake_camera:main",
        ],
    },
)

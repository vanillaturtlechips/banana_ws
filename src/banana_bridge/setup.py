from setuptools import find_packages, setup

package_name = "banana_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/bridge.yaml"]),
        ("share/" + package_name + "/launch", ["launch/bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="vanillaturtlechips",
    maintainer_email="wkdqkdgud@gmail.com",
    description="웹 게이트웨이: Starlette WS + LLM + rclpy + aiortc",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # uvicorn으로 직접 띄우는 게 기본이지만, ros2 run 진입점도 원하면:
            # "bridge = banana_bridge.__main__:main",
        ],
    },
)

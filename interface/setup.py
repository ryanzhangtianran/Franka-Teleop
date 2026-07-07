from setuptools import setup, find_packages

setup(
    name="interface",
    version="0.0.1",
    description="LeRobot Franka integration",
    author="Zhaolong Shen, Ryan Zhang",
    author_email="shenzhaolong@buaa.edu.cn, ryanzhangtianran@gmail.com",
    packages=find_packages(),
    install_requires=[
        "pyrealsense2",
        "scipy",
        "zerorpc",
    ],
    python_requires=">=3.10",
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
)

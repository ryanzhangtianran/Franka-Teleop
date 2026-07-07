from setuptools import setup, find_packages

setup(
    name="teleoperation",
    version="0.0.1",
    description="LeRobot teleoperator integration",
    author="Zhaolong Shen, Ryan Zhang",
    author_email="shenzhaolong@buaa.edu.cn, ryanzhangtianran@gmail.com",
    packages=find_packages(),
    python_requires=">=3.10",
    install_requires=[
        "easyhid",
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
)

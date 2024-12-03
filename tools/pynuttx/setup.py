from setuptools import find_packages, setup

setup(
    name="pynuttx",
    version="0.0.1",
    packages=find_packages(include=["nxgdb", "nxelf"]),
    install_requires=[],
    description="NuttX python development tools",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: Apache Software License",
    ],
    requires=["matplotlib", "numpy", "pyelftools", "debugpy"],
)

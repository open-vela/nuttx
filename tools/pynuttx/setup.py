from setuptools import find_packages, setup

setup(
    name="pynuttxtest",
    version="0.0.1",
    packages=find_packages(include=["nx*"]),
    install_requires=[],
    description="NuttX python development tools",
    entry_points={
        "console_scripts": [
            "nxstub = nxstub.__main__:main",
        ],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: Apache Software License",
    ],
    requires=["construct", "debugpy", "lief", "matplotlib", "numpy", "pyelftools"],
)

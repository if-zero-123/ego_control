from setuptools import find_packages, setup


setup(
    name="uav_protocol_gateway",
    version="0.2.0",
    packages=find_packages("src"),
    package_dir={"": "src"},
)

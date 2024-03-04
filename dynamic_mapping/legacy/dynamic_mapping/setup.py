from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["dynamic_mapping_py"],
    package_name="dynamic_mapping_py",
    package_dir={"": "include"},
)

setup(**d)

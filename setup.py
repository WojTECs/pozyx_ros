from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['pozyx_ros'],
    scripts=['scripts/pozyx_ros_node.py'],
    package_dir={'': 'src'}
)

setup(**d)
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["simple_navigation_goals"], package_dir={"": "src"}
)

setup(**d)

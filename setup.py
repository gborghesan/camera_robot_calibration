## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
d = generate_distutils_setup(
    packages=['camera_robot_calib'],
    package_dir={'': 'src'},
    scripts=['scripts/camera_robot_calibration.py']
)

setup(**d)

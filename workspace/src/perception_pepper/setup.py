from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['perception_utils', 'ml_pose'],
    #packages=['perception_utils', 'object_detection', 'object_tracker', 'pose_detection', 'colors_detection'],
    package_dir={'': 'scripts'}
)
setup(**d)
import os
import sys
from setuptools import setup, find_packages

print("Installing IK Tele-op.")

if sys.version_info.major != 3:
    print("This Python is only compatible with Python 3, but you are running "
          "Python {}. The installation will likely fail.".format(sys.version_info.major))
    
def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()

setup(
    name='ik_teleop',
    version='1.0.0',
    packages=find_packages(),
    description='Tools for recording demonstrations with allegro hand in sim and on the robot.',
    long_description=read('README.md'),
    url='https://github.com/NYU-robot-learning/Allegro-Inverse-Kinematics',
    author='sridhar, sneha',
)


import os
import sys
#Find directory path of current file
current = os.path.dirname(os.path.realpath(__file__))
#Find directory path of parent folder and add to sys path
parent = os.path.dirname(current)
sys.path.append(parent)

from kinematics.kinematic_funcs import IKSpace, FKSpace
import modern_robotics as mr
import numpy as np


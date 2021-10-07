from kinematics.kinematic_funcs import FKSpace, IKSpace
from trajectory_generation.traj_gen import TrajGen
from classes import Robot
import modern_robotics as mr

#Define our robot



###Position control, no obstacles###
# 1. User inserts a start- and end-configuration (either in the space frame or joint space)
# 2. Generate trajectory
# 3. Calculate joint angles and derivatives at every timestep

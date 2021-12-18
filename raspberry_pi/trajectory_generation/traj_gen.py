import os
import sys
#Find directory path of current file
current = os.path.dirname(os.path.realpath(__file__))
#Find directory path of parent folder and add to sys path
parent = os.path.dirname(current)
sys.path.append(parent)

from classes import Robot, DimensionError, IKAlgorithmError
from robot_init import robot as Pegasus
from kinematics.kinematic_funcs import IKSpace
import modern_robotics as mr
import numpy as np
#import matplotlib.pyplot as plt
from typing import Union, List, Tuple

def JointTrajLims(thetaStart: Union[List, np.ndarray], thetaEnd: 
                  Union[List, np.ndarray], lims: List, Tf: float, 
                  N: int, method: int):
    """Computes the shortest trajectory based on a given sampling 
    polynomial in joint space that respects joint limits.
    :param thetaStart: List/array of joint starting joint angles.
    :param thetaEnd: List/array of desired final joint angles.
    :param lims: List of limits of the type [lower, upper], with all 
                 possible configurations in that interval.
    :param Tf: Total time between thetaStart and thetaEnd
    :param N: Number of sampling points of the trajectory.
    :param method: Type of sampling polynomial: 3 for cubic, 5 for 
                   quintic.
    :return traj: A matrix of joint angles, with the ith column 
                  describing the trajectory of the ith joint. 
    
    Example input:
    thetaStart = np.array([0,0,0,0,0])
    thetaEnd = np.array([np.pi, -0.8*np.pi, 0.5*np.pi, 0.7*np.pi, 0.4*np.pi])
    lims = [[-0.1*np.pi, -0.7*np.pi] for i in range(len(thetaStart))]
    Tf = 5
    N = 4
    method = 5
    Output:
    [[0.         0.         0.         0.         0.        ]
     [0.65934661 0.79121593 0.3296733  0.46154262 0.26373864]
     [2.48224605 2.97869526 1.24112302 1.73757223 0.99289842]
     [3.14159265 3.76991118 1.57079633 2.19911486 1.25663706]]

    NOTE: Credits to Thomas Sjerps (TU Delft) for writing the majority
    of this code.
    """
    thetaStart = np.array(thetaStart)
    thetaEnd = np.array(thetaEnd)
    lims = np.array(lims)
    for i in range(thetaStart.size):
        start = thetaStart[i]
        end = thetaEnd[i]
        lim = lims[i]
        #Transform angles to [0,2pi]
        start = np.mod(start, 2*np.pi)
        end = np.mod(end, 2*np.pi)
        #long & short path: Increasing or decreasing angle?
        if end - start > np.pi:
            short = -2*np.pi #decreasing
            long = 0 #increasing
        else:
            short = 0 
            long = 2*np.pi
        lim = np.mod(lim, 2*np.pi)
        lim = np.sort(lim) #Redefine lower- and upper
        #Rotate all angles w.r.t the lower limit
        startL = np.mod((start - lim[0]), 2*np.pi)
        endL = np.mod((end - lim[0]), 2*np.pi)
        limL = lim.copy()
        limL[1] = np.mod((lim[1] - lim[0]), 2*np.pi)
        #Are the start- or end configuration in the limit?
        if startL < limL[1] or endL < limL[1]:
            raise ValueError("At least one starting- or ending joint " +
            "angle is inside of joint limits")
        #Does the short path go through the limit?
        elif abs(startL - endL) > np.pi:
            #Take the long path
            thetaEnd[i] = end+long
        else:
            #Take the short path
            thetaEnd[i] = end+short
    traj = mr.JointTrajectory(thetaStart, thetaEnd, Tf, N, method)
    return traj

def TrajGen(robot: Robot, startConfig: Union[np.ndarray, List[float]], endConfig: 
    Union[np.ndarray, List[float]], vMax: float,
    omgMax: float, dt: float, method: str="joint", timeScaling: int=5) -> \
    Tuple[np.ndarray, np.ndarray]:
    """Calculates a straight-line trajectory between two 
    configurations, eitherin screw-, joint-, or cartesian space.
    :param robot: Robot object mathematically representing the robot.
    :param startConfig: The starting configuration, either as a list of
                        joint angles or an SO(3) matrix.
    :param endConfig: The end configuration, either as a list of joint
                      angles or an SO(3) matrix.
    :param vMax: The maximum continuous linear velocity of the 
                 end-effector.
    :param omgMax: The maximum continuous rotational speed of the motor
                   output shafts.
    :param dt: Desired time between each sub configuration, for 
               resolution purposes.
    :param method: Space in which the trajectory should be calculated.
                   Choice between 'screw', 'joint', and 'cartesian'.
    :param timeScaling: The order of the time polynomial which is 
                        followed for the trajectory. Choice between
                        cubic (3) or quintic (5).
    :return traj: List of configurations evolving linearly in the 
                  desired space (determined by 'method').

    Example input:
    (robot arg initalisation is omitted for the sake of brevity)
    robot = Robot(joints, links, TsbHome)
    startConfig = [0,0,0,0,0]
    endConfig = [0.2*np.pi for i in range(len(startConfig))]
    vMax = 0.2
    omgMax = 0.25*np.pi
    dt = 0.2
    method = 'joint'
    timeScaling = 5
    Output:
    [[0.         0.         0.         0.         0.        ]
    [0.03639221 0.03639221 0.03639221 0.03639221 0.03639221]
    [0.19945343 0.19945343 0.19945343 0.19945343 0.19945343]
    [0.4288651  0.4288651  0.4288651  0.4288651  0.4288651 ]
    [0.59192632 0.59192632 0.59192632 0.59192632 0.59192632]
    [0.62831853 0.62831853 0.62831853 0.62831853 0.62831853]]
    NOTE: Only joint trajectories are ensured to respect joint limits!
    """
    if timeScaling != 3 and timeScaling != 5:
                print("Invalid timeScaling; defaulting to quintic.")
                timeScaling = 5

    if method == "joint" or method == "Joint":
        if not isinstance(startConfig, np.ndarray) or \
           not isinstance(endConfig, np.ndarray):
                raise SyntaxError("To make a trajectory in the joint space," +
                                  " please input an array of joint angles.")
        elif startConfig.size != endConfig.size:
            raise DimensionError("start- and end joint angle lists" +
                                 " are not of the same length: " +
                                 f"({startConfig.size, endConfig.size}")
        else:
            thetaMaxPos = max(np.subtract(endConfig, startConfig))
            thetaMaxNeg = min(np.subtract(endConfig, startConfig))
            thetaMax = max(abs(thetaMaxNeg), thetaMaxPos)
            tTot = 1.5*(thetaMax / omgMax) #Scaling factor might require tweaking
            if tTot == 0:
                tTot = 0.1 #To avoid division by zero errors
            nSubConfigs = int(tTot/dt + 2)
            traj = JointTrajLims(startConfig, 
            endConfig, robot.limList, tTot, nSubConfigs, timeScaling)
    else:
        if not mr.TestIfSE3(startConfig) or not mr.TestIfSE3(endConfig):
            raise SyntaxError("Ensure that both the start- and end " +
                              "configuration are part of the SE(3) " + 
                              "manifold.")
        else:
            pStart = startConfig[0:3,3]
            pEnd = endConfig[0:3,3]
            distTot = np.linalg.norm(pStart-pEnd)
            tTot = 1.5*(distTot/vMax) #Scaling factor might require tweaking
            nSubConfigs = int(tTot/dt)
            if method == "screw" or method == "Screw":
                traj = np.array(mr.ScrewTrajectory(startConfig, endConfig, tTot, 
                nSubConfigs, timeScaling))
            elif method == "cartesian" or method == "Cartesian":
                traj = np.array(mr.CartesianTrajectory(startConfig, endConfig, 
                tTot, nSubConfigs, timeScaling))
            else:
                raise SyntaxError("Invalid method input. Please choose " + 
                                  "between 'joint', 'screw', or 'cartesian'")
    return traj

def TrajDerivatives(traj: Union[List[np.ndarray], List[List[float]]], 
                    method: str, robot: Robot, dt: float) \
                    -> Tuple[np.ndarray]:
    """Calculates the time derivatives of a sequential list of 
    configurations.
    :param traj: List of sequential configurations in either the joint- 
                 or work space.
    :param method: The space in which the trajectory linearly evolves
                   over time. Either 'joint', 'screw', or 'cartesian'.
    :param robot: Robot class object describing the robot in question.
    :param dt: Time between each configuration.
    :return trajTheta: The trajectory in joint space.
    :return trajVel: The joint velocities during the trajectory.
    :return trajAcc: The join accelerations during the trajectory.
    Example input:
    (Init of joints & links not included for sake of brevity)
    robot = Robot(joints, links, TsbHome)
    sConfig = [0,0,0,0,0]
    fConfig = [1,1,1,1,1]
    vMax = 1
    omgMax = 1
    dt = 0.02
    method = 'joint'
    traj = TrajGen(sConfig, fConfig, vMax, omgMax, dt, method)[0]

    Output:
    [[0.00000000e+00 0.00000000e+00 0.00000000e+00 0.00000000e+00
    0.00000000e+00]
    [7.31859478e-05 7.31859478e-05 7.31859478e-05 7.31859478e-05
    7.31859478e-05]...]

    [[-1.00000000e+01 -1.00000000e+01 -1.00000000e+01 -1.00000000e+01
    -1.00000000e+01]
    [ 7.31859478e-04  7.31859478e-04  7.31859478e-04  7.31859478e-04
    7.31859478e-04]...]

    [[-1.00000000e+02 -1.00000000e+02 -1.00000000e+02 -1.00000000e+02
    -1.00000000e+02]
    [ 1.00007319e+02  1.00007319e+02  1.00007319e+02  1.00007319e+02
    1.00007319e+02]...]
    """
    trajTheta = np.zeros((len(traj),len(robot.joints)))
    trajVel = np.zeros((len(traj),len(robot.joints)))
    trajAcc = np.zeros((len(traj),len(robot.joints)))
    if method != "joint" and method != "Joint":
        # robot.TllList[-1] = TsbHome, the end-effector frame described in
        # the space frame at the home configuration of the robot.
        lims = [robot.joints[i].lims for i in range(len(robot.joints))]
        for i in range(len(traj)): 
            trajTheta[i], success = IKSpace(robot.TllList[-1], traj[i], 
            robot.screwAxes, lims, eRad=0.03, eLin=0.03)
            if not success:
                """A straight path in end-effector space might move 
                through configurations that fall outside of the 
                C-space of the joints, as the C-space of the end-
                effector is likely non-convex. """
                raise IKAlgorithmError()
    else:
        trajTheta = traj
    #Discrete differentiation
    for i in range(len(trajTheta)-1):
        trajVel[i] = (trajTheta[i+1] - trajTheta[i])/dt
        trajAcc[i] = (trajVel[i+1] - trajVel[i])/dt 
    return trajTheta, trajVel, trajAcc

if __name__ == "__main__":
    startConfig = [0,0,0,0,0]
    endConfig = [0.2*np.pi for i in range(len(startConfig))]
    vMax = 0.2
    omgMax = 0.25*np.pi
    dt = 0.01
    method = 'joint'
    timeScaling = 5
    traj = TrajGen(Pegasus, startConfig, endConfig, vMax, omgMax, dt, method="joint")
    trajTheta, trajVel, trajAcc = TrajDerivatives(traj, robot=Pegasus, method="joint", dt=0.1)
    valList = trajTheta[:,0]
    timeList = [dt*n for n in range(trajTheta[:,0].size)]
    #plt.plot(timeList, valList)
    #plt.show()
    thetaStart = np.array([0,0,0,0,0])
    thetaEnd = np.array([np.pi, -0.8*np.pi, 0.5*np.pi, 0.7*np.pi, 0.4*np.pi])
    lims = [[-0.1*np.pi, -0.7*np.pi] for i in range(len(thetaStart))]
    Tf = 5
    N = 4
    method = 5

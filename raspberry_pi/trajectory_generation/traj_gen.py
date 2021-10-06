import sys
sys.path.append('C:\\DeKUT_Internship\\Robot_Arm\\PegasusArmOS\\raspberry_pi\\')
from classes import Robot, DimensionError
from kinematics.kinematic_funcs import IKSpace
import modern_robotics as mr
import numpy as np
from typing import Union, List, Tuple

def TrajGen(startConfig: Union[np.ndarray, List[float]], endConfig: 
    Union[np.ndarray, List[float]], vMax: float,
    omgMax: float, dt: float, method: str="screw", timeScaling: int=5) -> \
    Tuple[np.ndarray, List[float]]:
    """Calculates a straight-line trajectory between two 
    configurations, eitherin screw-, joint-, or cartesian space.
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
                   Choice between 'screw', 'joint', and 'cartesian' 
                   (space).
    :param timeScaling: The order of the time polynomial which is 
                        followed for the trajectory. Choice between
                        cubic (3) or quintic (5).
    """
    if timeScaling != 3 | timeScaling != 5:
                print("Invalid timeScaling; defaulting to quintic.")
                timeScaling = 5
    if method == "joint" or method == "Joint":
        if not isinstance(startConfig, list) or \
           not isinstance(endConfig, list):
            raise SyntaxError("To make a trajectory in the joint space," +
                              " please input a list of joint angles.")
        elif len(startConfig) != len(endConfig):
            raise DimensionError("start- and end joint angle lists" +
                                 " are not of the same length: " +
                                 f"({len(startConfig),len(endConfig)}")
        else:
            thetaMaxPos = max(np.subtract(endConfig, startConfig))
            thetaMaxNeg = min(np.subtract(endConfig, startConfig))
            thetaMax = max(abs(thetaMaxNeg), thetaMaxPos)
            tTot = thetaMax / omgMax
            nSubConfigs = int(tTot/dt + 2)
            traj = mr.JointTrajectory(np.array(startConfig), 
            np.array(endConfig), tTot, nSubConfigs, timeScaling)
    else:
        if not isinstance(startConfig, np.ndarray) or \
           not isinstance(endConfig, np.ndarray):
            raise SyntaxError("To make a trajectory in SE(3), input SE(3) " + 
                              "numpy arrays as start- and end configurations.")
        elif not mr.TestIfSE3(startConfig) or not mr.TestIfSE3(endConfig):
            raise SyntaxError("Ensure that both the start- and end " +
                              "configuration are part of the SE(3) " + 
                              "manifold.")
        else:
            pStart = startConfig[0:3,3]
            pEnd = endConfig[0:3,3]
            distTot = np.linalg.norm(pStart-pEnd)
            tTot = distTot/vMax
            nSubConfigs = int(tTot/dt)
            if method == "screw" or method == "Screw":
                traj = mr.ScrewTrajectory(startConfig, endConfig, tTot, 
                nSubConfigs, timeScaling)
            elif method == "cartesian" or method == "Cartesian":
                traj = mr.CartesianTrajectory(startConfig, endConfig, 
                tTot, nSubConfigs, timeScaling)
            else:
                raise SyntaxError("Invalid method input. Please choose " + 
                                  "between 'joint', 'screw', or 'cartesian'")
    timeList = [dt * (tTot/(nSubConfigs-1)) for dt in range(0, nSubConfigs)]
    return traj, timeList

def trajDerivatives(traj: Union[List[np.ndarray], List[List[float]]], 
                    method: str, robot: Robot, deltaT: float) \
                    -> Tuple[List[List[float]]]:
    if method != "joint" and method != "Joint":
        trajTheta = [IKSpace(robot.TsbHome, traj[i], robot.screwAxes, 
                     robot.lims)[0].tolist() for i in range(len(traj))]
    else:
        trajTheta = traj
    trajVel = [(trajTheta[i] - trajTheta[i-1])/deltaT 
                for i in range(1, len(trajTheta))]
    trajAcc = [(trajVel[i] - trajVel[i-1])/deltaT 
               for i in range(1, len(trajTheta))]
    return trajTheta, trajVel, trajAcc
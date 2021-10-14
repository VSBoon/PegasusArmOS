import sys
sys.path.append('C:\\DeKUT_Internship\\Robot_Arm\\PegasusArmOS\\raspberry_pi\\')
from classes import Robot, Joint, Link, DimensionError
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
    :return traj: List of configurations evolving linearly in the 
                  desired space (determined by 'method').
    :return timeList: List of timestamps for each configuration in 
                      'traj'.
    Example input:

    Output:

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

def TrajDerivatives(traj: Union[List[np.ndarray], List[List[float]]], 
                    method: str, robot: Robot, deltaT: float) \
                    -> Tuple[List[List[float]]]:
    """Calculates the time derivatives of a sequential list of 
    configurations.
    :param traj: List of sequential configurations in either the joint- 
                 or work space.
    :param method: The space in which the trajectory linearly evolves
                   over time. Either 'joint', 'screw', or 'cartesian'.
    :param robot: Robot class object describing the robot in question.
    :param deltaT: Time between each configuration.
    :return trajTheta: The trajectory in joint space.
    :return trajVel: The joint velocities during the trajectory.
    :return trajAcc: The join accelerations during the trajectory.
    Example input:

    Output:
    
    TODO: FINISH DOCSTRING
    """
    if method != "joint" and method != "Joint":
        # robot.links[-1].Tsi = TsbHome, the end-effector frame described in
        # the space frame at the home configuration of the robot.
        trajTheta = [IKSpace(robot.links[-1].Tsi, traj[i], robot.screwAxes, 
                     robot.lims)[0].tolist() for i in range(len(traj))]
    else:
        trajTheta = traj
    trajVel = [(trajTheta[i] - trajTheta[i-1])/deltaT 
                for i in range(1, len(trajTheta))]
    trajAcc = [(trajVel[i] - trajVel[i-1])/deltaT 
               for i in range(1, len(trajTheta))]
    return trajTheta, trajVel, trajAcc

if __name__ == "__main__":
    #Inertia matrices
    iMat1 = np.array([[]])
    massList = [4.99, 0.507, 0.420, 0.952, 0.952]
    #Transformation matrices from CoM of links with principle axes of
    #inertia to the space frame (Tsi):
    Tsi1 = np.array([[0.3804 ,-0.9215,0.0786 ,-0.0103],
                     [0.8774 ,0.3864 ,0.2843 ,-0.0292],
                     [-0.2924,-0.0392,0.9555 ,0.0642 ],
                     [0      ,0      ,0      ,1      ]])
    Tsi2 = np.array([[-0.0002,0.0008 ,-1.0000,0.0350 ],
                     [0.4553 ,0.8903 ,0.0007 ,-0.0083],
                     [0.8903 ,-0.4553,-0.0006,0.1666 ],
                     [0      ,0      ,0      ,1      ]])
    Tsi3 = np.array([[-0.9966,0.0819 ,-0.0003,0.0814 ],
                     [-0.0819,-0.9966,0.0008 ,-0.0083],
                     [-0.0002,0.0008 ,1.0000 ,0.3550 ],
                     [0      ,0      ,0      ,1      ]])
    Tsi45 = np.array([[-0.0326,0.0002 ,-0.9995,0.2640 ],
                      [-0.0004,1.0000 ,0.0002 ,-0.0059],
                      [-0.9995,0.0004 ,-0.0326,0.3504 ],
                      [0      ,0      ,0      ,1      ]])
    #Screw axes in the Space Frame {s}
    S1 = np.array([0,0,1,0,0,0])
    S2 = np.array([0,1,0,-0.0035,0,0.126])
    S3 = np.array([0,1,0,-0.0035,0,0.335])
    S4 = np.array([0,1,0,-0.234,0,0.355])
    S5 = np.array([0,0,1,-0.234,-0.016,0])
    gearRatioList = [19.7*50, 19.7*50, (65.5*20)/9, (65.5*20)/9, (127.7*32)/9]
    J1 = Joint()
    J2 = Joint()
    J3 = Joint()
    J4 = Joint()
    J5 = Joint()
    J6 = Joint()
    L1 = Link()
    L2 = Link()
    L3 = Link()
    L4 = Link()
    L5 = Link()
    Pegasus = Robot()


    startConfig = [0,0,0,0,0]
    endConfig = [0.5*np.pi, 0.2*np.pi, 0.1*np.pi, 0.3*np.pi, 0.4*np.pi, 0]
    vMax = 0.5
    omgMax = 0.2
    dt = 0.1
    traj, timeList = TrajGen(startConfig, endConfig, vMax, omgMax, dt, method="joint", timeScaling=5)
    trajTheta, trajVel, trajAcc = TrajDerivatives(traj, method="joint", Pegasus)
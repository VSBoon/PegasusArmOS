import os
import sys
#Find directory path of current file
current = os.path.dirname(os.path.realpath(__file__))
#Find directory path of parent folder and add to sys path
parent = os.path.dirname(current)
sys.path.append(parent)

from kinematics.kinematic_funcs import IKSpace, FKSpace
from classes import Joint, Link, Robot
from typing import List
import modern_robotics as mr
import numpy as np


def LossComp(tauComm: float, dtheta: float, dthetaPrev: float, tauStat: float, 
            bVisc: float, tauKin: float, gRatio: float, eff: float) -> float:
    """Adds friction according to a model utilizing static, viscous, 
    and kinetic friction coefficients. 
    :param tauComm: Desired torque based on inverse dynamics at the 
                    output shaft.
    :param dtheta: Desired joint velocity.
    :param dthetaPrev: Desired joint velocities in the 
                       previous time step.
    :param tauStat: Torque required to overcome static friction 
                    (occurs if dthetaPrev == 0).
    :param bVisc: Viscous friction coefficient in (N*m^2*s)/rad
    :param tauKin: Torque required to overcome kinetic friction.
                   NOTE: tauKin should be smaller than tauStat!
    :param gRatio: Gearbox ratio (>1).
    :param eff: Efficiency (heat dissipation, gearbox losses, etc.)
    :return tauComm: Desired output torque, including friction- and
                     efficiency compensation.
    NOTE: All variables are taken w.r.t the output shaft, so after 
    the internal & external gearbox!
    """
    if np.isclose(dthetaPrev, 0, atol=1e-04) and \
        not tauComm == 0: #Static friction
        tauComm += tauStat*np.sign(dtheta)
    elif not tauComm == 0: #Kinetic & viscous friction
        tauComm += tauKin*np.sign(dtheta) + \
                   np.multiply(bVisc, dtheta)
    return tauComm/eff

def FeedForward(robot: Robot, thetaList: List[float], dthetaList: List[float], 
                ddthetaList: List[float], Ftip: np.ndarray[float]) -> \
                np.ndarray[float]:
    """Computes the feed-forward torque based on the inverse dynamics 
    of the robot.
    NOTE: This function is used for robots with a shared fifth link. 
    Robots having a 'pure' serial configuration can simply use the 
    modern_robotics.InverseDynamics() function.
    :param robot: A Robot-class instance containing all characteristic
                  information of the robot.
    :param thetaList: List of currently desired joint angles.
    :param dthetaList: List of currently desired joint velocities.
    :param ddthetaList: List of currently desired joint accelerations.
    :param Ftip: 6-vector representing the desired end-effector forces 
                 and torques in the end-effector frame(!).
    :return FFTorque: Array of feed-forward joint torques.
    """
    #TODO: checks for lists, checks for Ftip.
    g = np.array([0, 0, -9.81]) #gravity-vector in-line with the space frame!
    """Calculate FF torque for all joints except the last, 
    due to shared link w/ joint 4 (which causes problematic dynamics).
    """
    FFTorque = mr.InverseDynamics(thetaList[:-1], dthetaList[:-1], 
                                  ddthetaList[:-1], g, Ftip, \
                                  robot.TllList[:-1], robot.GiList[:-1], \
                                  robot.screwAxes[:-1])
    """Remove joint 4 from the model to calculate final joint torque
    NOTE: Might be inefficient, as the torques for motor 0 through 3 
    are calculated twice.
    """
    thetaList.pop(-2)
    dthetaList.pop(-2)
    ddthetaList.pop(-2)
    TllFinal = robot.TllList[0:-2] + [robot.TllList[-1]]
    GiFinal = robot.GiList[0:-2] + [robot.GiList[-1]]
    screwFinal = robot.screwAxes[0:-2] + [robot.screwAxes[-1]]
    lastTorque = mr.InverseDynamics(thetaList, dthetaList, ddthetaList, g, \
                                    Ftip, TllFinal, GiFinal, \
                                    screwFinal)[-1]
    FFTorque = np.append(FFTorque, lastTorque)
    return FFTorque
    
#TODO: Add 'CheckTorque' func to abide limits.

def ForwardDyn():
    """"""
    mr.ForwardDynamics()
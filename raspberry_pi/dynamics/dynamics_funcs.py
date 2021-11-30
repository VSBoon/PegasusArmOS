import os
import sys
#Find directory path of current file
current = os.path.dirname(os.path.realpath(__file__))
#Find directory path of parent folder and add to sys path
parent = os.path.dirname(current)
sys.path.append(parent)

from classes import Robot
from typing import List, Tuple
import modern_robotics as mr
import numpy as np

""" NOTE: IF NOT TESTED ON RPi, ALL MENTIONS OF RPi LIBRARY SHOULD BE
    REMOVED FROM ALL IMPORTS BEFORE RUNNING THESE TESTS!
"""

def LossComp(tauComm: float, dtheta: float,  tauStat: float, 
            bVisc: float, tauKin: float, eff: float) -> float:
    """Adds friction according to a model utilizing static, viscous, 
    and kinetic friction coefficients. 
    :param tauComm: Desired torque based on inverse dynamics at the 
                    output shaft.
    :param dtheta: Desired joint velocity.
    :param tauStat: Torque required to overcome static friction 
                    (occurs if dthetaPrev == 0).
    :param bVisc: Viscous friction coefficient in (N*m^2*s)/rad
    :param tauKin: Torque required to overcome kinetic friction.
                   NOTE: tauKin should be smaller than tauStat!
    :param eff: Efficiency (heat dissipation, gearbox losses, etc.)
    :return tauComm: Desired output torque, including friction- and
                     efficiency compensation.
    NOTE: All variables are taken w.r.t the output shaft, so after 
    the internal & external gearbox!
    """
    if np.isclose(dtheta, 0, atol=1e-04) and \
        not tauComm == 0: #Static friction
        if dtheta != 0:
            dir = np.sign(dtheta)
        else:
            dir = np.sign(tauComm) #Reasonable approximation
        tauComm += tauStat*dir
    elif not tauComm == 0: #Kinetic & viscous friction
        tauComm += tauKin*np.sign(dtheta) + \
                   bVisc*dtheta
    return tauComm/eff

def FeedForward(robot: Robot, theta: List, dtheta: List, ddtheta: List, 
                g: np.ndarray, FTip: np.ndarray) -> np.ndarray:
    """Optimized feed-forward for the Pegasus arm, only going through 
    the Newton-Euler inverse dynamics algorithm once. Based on the
    Modern Robotics Python Library.
    :param robot: A Robot class describing the robot mathematically.
    :param thetaList: List of desired joint angles.
    :param dthetaList: List of desired joint velocities.
    :param ddthetaList: List of desired joint accelerations.
    :param g: 3-dimensional gravity vector in [m/s^2].
    :param FTip: 6-dimensional End-effector wrench.
    :return tau: A list of required joint torques at the output shaft.

    Example input:
    (Joints and links init not included for brevity)
    Pegasus = Robot(joints, links)
    thetaList = [0,0.5*np.pi, 0.25*np.pi, 0.25*np.pi, 0]
    dthetaList = [0,1,0,0,0]
    ddtheta = [0,1,1,1,1]
    g = np.array([0,0,-9.81])
    FTip = np.array([1,1,1,1,1,1])
    Output:
    [ 0.81  -1.343  0.767  1.754  0.958]
    """
    #Initialization
    #DEBUG: REMOVE INVERSE FOR TLLLIST
    #Note: 6,shape is used to obtain column vectors for calculations.
    n = len(theta) #Number of joints
    Tsi = np.eye(4)
    AdHis = np.zeros((n+1,6,6))
    screwS = np.c_[robot.screwAxes[0], robot.screwAxes[1]]
    for i in range(2,n):
        screwS = np.c_[screwS, robot.screwAxes[i]]
    screwA = np.zeros((6, n))
    expT = np.zeros((n,4,4))
    AdTiiN = np.zeros((n+1,6,6))
    """The end-effector frame (TllList[n]) is rigidly connected
    to the frame of the previous link --> constant T_(n-1,n)"""
    AdTiiN[n] = mr.Adjoint(mr.TransInv(robot.TllList[n]))
    V = np.zeros((6, n+1))
    dV = np.zeros((6, n+1))
    dV[3:,0] = -g #Gravity is equivalent to upward acceleration
    #This way, the gravity gets 'carried on' in forward iterations.
    F = np.zeros((6, n+1))
    F[:, n] = FTip
    tau = np.zeros(n)
    #Forward iterations
    for i in range(n):
        if i != n-1:
            #Adjoint of the transformation matrix {i} to {s} in the home config
            #For some reason, Tsi = robot.links[i].Tsi gives the wrong answer...
            Tsi = np.dot(Tsi, robot.links[i].Tii)
            AdHis[i] = mr.Adjoint(mr.TransInv(Tsi))
            #Screw axis in frame {i}. np.array([]) to go from 1D to 2D
            screwA[:,i] = np.dot(AdHis[i],screwS[:,i])
            #Exponential of A with angle theta
            expT[i] = mr.MatrixExp6(mr.VecTose3(screwA[:,i]*-theta[i]))
            #Adjoint of T_(i,i-1), given current config:
            AdTiiN[i] = mr.Adjoint(np.dot(expT[i],mr.TransInv(robot.TllList[i])))
            #Twist due to joint velocity + twist due to prev. link
            V[:,i+1] = screwA[:,i]*dtheta[i] + np.dot(AdTiiN[i], V[:,i])
            #Acceleration due to joint acceleration + vel-prod. term + prev link
            dV[:,i+1] = screwA[:,i]*ddtheta[i] + np.dot(AdTiiN[i], dV[:,i]) + \
                    np.dot(mr.ad(V[:,i+1]), screwA[:,i]*dtheta[i])
        elif i == n-1:
            AdHis[i] = mr.Adjoint(mr.TransInv(Tsi))
            screwA[:,i] = np.dot(AdHis[i],screwS[:,i])
            expT[i] = mr.MatrixExp6(mr.VecTose3(screwA[:,i]*-theta[i]))
            AdTiiN[i] = mr.Adjoint(np.dot(expT[i],mr.TransInv(robot.TllList[i])))
            #NOTE: Prev twist == V[:,i-1] & Prev acc == dV[:,i-1]
            V[:,i+1] = screwA[:,i]*dtheta[i] + np.dot(AdTiiN[i], V[:,i-1])
            dV[:,i+1] = screwA[:,i]*ddtheta[i] + np.dot(AdTiiN[i], dV[:,i-1]) + \
                    np.dot(mr.ad(V[:,i+1]), screwA[:,i]*dtheta[i])
    #Backward iterations
    for i in range(n-1, -1, -1):
        if i != n-2:
            """Wrench due to wrench next link + link acceleration - 
            Coriolis/centripetal terms"""
            F[:,i] = np.dot(AdTiiN[i+1].T, F[:,i+1]) + \
                np.dot(robot.GiList[i], dV[:,i+1]) - \
                np.dot(mr.ad(V[:,i+1]).T, np.dot(robot.GiList[i], V[:,i+1]))
        elif i == n-2: #Pegasus arm specific mechanics
            #NOTE: Same wrench as F[:,n-1], as it is the same link
            F[:,i] = F[:,i+1]
        """Obtain torques by projection through screwA, effectively a
        column of the Jacobian between wrenches/twists in {i} and 
        joint space"""
        tau[i] = np.dot(F[:,i].T, screwA[:,i]) 
    return tau

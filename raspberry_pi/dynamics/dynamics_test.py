import os
import sys

#Find directory path of current file
current = os.path.dirname(os.path.realpath(__file__))
#Find directory path of parent folder and add to sys path
parent = os.path.dirname(current)
sys.path.append(parent)

import numpy as np
import modern_robotics as mr
from classes import Robot, Link, Joint
from dynamics.dynamics_funcs import LossComp, FeedForward

np.set_printoptions(precision=3)

tauComm = 5
dtheta = 1
tauStat = 0.02
bVisc = 0.04
tauKin = 0.015
eff = 0.8

def test_LossCompNoLoss():
    """Checks functionality if loss model is nullified"""
    tauStat = 0
    bVisc = 0
    tauKin = 0
    eff = 1
    tau = LossComp(tauComm, dtheta, tauStat, bVisc, tauKin, eff)
    assert tau == tauComm

def test_LossCompEff():
    """Checks efficiency calculation"""
    tauStat = 0
    bVisc = 0
    tauKin = 0
    eff = 0.8
    tau = LossComp(tauComm, dtheta, tauStat, bVisc, tauKin, eff)
    assert tau == tauComm/eff

def test_LossCompStat():
    """Observe if static & kinetic friction (w/o viscous) works."""
    tauStat = 1
    dtheta = 0
    eff = 1
    bVisc = 0.04
    tau = LossComp(tauComm, dtheta, tauStat, bVisc, tauKin, eff)
    assert tau == tauComm + tauStat
    dtheta = 1
    bVisc = 0
    tau = LossComp(tauComm, dtheta, tauStat, bVisc, tauKin, eff)
    assert tau == tauComm + tauKin

def test_LossCompDyn():
    """Checks if full functionality works"""
    tau = LossComp(tauComm, dtheta, tauStat, bVisc, tauKin, eff)
    assert tau == (tauComm + tauKin + bVisc*dtheta)/eff

def test_LossCompSign():
    """See if LossComp correctly handles velocity signes"""
    dtheta = -1
    eff = 1
    tau = LossComp(tauComm, dtheta, tauStat, bVisc, tauKin, eff)
    assert tau == tauComm - tauKin + bVisc*dtheta     

###ROBOT INITIALISATION###
#Inertia matrices
iMat0 = np.diag([0.03947, 0.03362, 0.04886])
iMat1 = np.diag([0.00393, 0.00237, 0.00172])
iMat2 = np.diag([0.00294, 0.00210, 0.0029])
iMat34 = np.diag([0.00041, 0.00348, 0.00364])
massList = [5.13, 0.507, 0.420, 0.952, 0.952]
#Transformation matrices from CoM of links with principle axes of
#inertia to the space frame (Tsi):
Tsi0 = np.array([[ 0.397, 0.838,-0.375, 0.0284],
                 [-0.909, 0.416,-0.033,-0.0413],
                 [ 0.129, 0.354, 0.926, 0.0522],
                 [0    , 0     , 0    , 1     ]])
Tsi1 = np.array([[ 0.000, 0.455, 0.890, 0.0015],
                 [ 0.001, 0.890,-0.455, 0.0026],
                 [-1.000, 0.007, 0.001, 0.0039],
                 [0    , 0     , 0    , 1     ]])
Tsi2 = np.array([[-0.003, 0.082, 0.997, 0.0009],
                 [ 0.001, 0.997,-0.082, 0.0021],
                 [-1.000, 0.001,-0.003, 0.0029],
                 [0    , 0     , 0    , 1     ]])
Tsi34 = np.array([[-0.999, 0.000, -0.035, 0.0076],
                  [0.000, -1.000, -0.000,-0.0159],
                  [-0.035, -0.000, 0.999, 0.5840],
                  [0    , 0     , 0    , 1     ]])
TsbHome = np.array([[1,0,0, 0.1474],
                    [0,1,0,-0.0168],
                    [0,0,1, 0.5853],
                    [0,0,0, 1     ]])
#Screw axes in the Space Frame {s}
S0 = np.array([0,0,1,0,0,0])
S1 = np.array([0,1,0,-0.125,0,0.0035])
S2 = np.array([0,1,0,-0.355,0,0.0035])
S3 = np.array([0,1,0,-0.585,0,0.0030])
S4 = np.array([1,0,0,0,0.585,0.016])
lims0 = [-0.945*np.pi, 0.945*np.pi] #+/- 170 deg
lims1 = [-0.25*np.pi, 0.5*np.pi] #-45 deg, + 90 deg
lims2 = [-np.pi, 0.25*np.pi] #-180 deg, + 45 deg.
lims3 = [-np.pi, 0.25*np.pi] #-180 deg, +45 deg.
lims4 = [-np.pi, np.pi] #+/- 180 deg.
gearRatioList = [19.7*50, 19.7*50, (65.5*20)/9, (65.5*20)/9, (127.7*32)/9]
cpr = 512
L0 = Link(iMat0, massList[0], None, Tsi0)
L1 = Link(iMat1, massList[1], L0, Tsi1)
L2 = Link(iMat2, massList[2], L1, Tsi2)
L34 = Link(iMat34, massList[3], L2, Tsi34)
links = [L0, L1, L2, L34, L34]
km = [22.7*10**(-3), 22.7*10**(-3), 22.7*10**(-3), 22.7*10**(-3),
          22.7*10**(-3), 22.7*10**(-3), 9.2*10**(-3)] 
J0 = Joint(S0, [None, L0], gearRatioList[0], km[0], cpr, lims0)
J1 = Joint(S1, [L0, L1], gearRatioList[1], km[1], cpr, lims1)
J2 = Joint(S2, [L1, L2], gearRatioList[2], km[2], cpr, lims2)
J3 = Joint(S3, [L2,L34], gearRatioList[3], km[3], cpr, lims3)
J4 = Joint(S4, [L2,L34], gearRatioList[4], km[4], cpr, lims4)
joints = [J0, J1, J2, J3, J4]
robot = Robot(joints, links, TsbHome)
###END OF ROBOT INIT###

def test_FFOptZero():
    """Check if FF gives no required torques if no forces are applied."""
    theta = np.array([0, 0, 0, 0, 0])
    dtheta = np.array([0, 0, 0, 0, 0])
    ddtheta = np.array([0, 0, 0, 0, 0])
    g = np.array([0, 0, 0])
    FTip = np.array([0,0,0,0,0,0])
    tau = FeedForward(robot, theta, dtheta, ddtheta, g, FTip)
    assert all(tau) == False
    #Change configuration: Shouldn't change anything
    theta = np.array([0.05*np.pi, 0.5*np.pi, 0.25*np.pi, 0, np.pi])
    tau = FeedForward(robot, theta, dtheta, ddtheta, g, FTip)
    assert all(tau) == False

def test_FFOptGrav():
    theta = np.array([0,0,0,0,0])
    dtheta = np.array([0,0,0,0,0])
    ddtheta = np.array([0,0,0,0,0])
    g = np.array([0,0,-9.81])
    FTip = np.array([0,0,0,0,0,0])
    tau = FeedForward(robot, theta, dtheta, ddtheta, g, FTip)
    print(tau)
    """Increasing the angle of the second joint (downward) should
    make the torque for this joint (and the third) significantly larger.
    Additionally: The torque on the fourth joint should decrease, as 
    the gripper is now pointing downward."""
    theta = np.array([0,0.5*np.pi,0,0,0])
    tau2 = FeedForward(robot, theta, dtheta, ddtheta, g, FTip)
    assert abs(tau2[1]) > abs(tau[1])
    assert abs(tau2[2]) > abs(tau[2])
    assert abs(tau2[3]) < abs(tau[3])


if __name__ == "__main__":
    test_FFOptGrav()
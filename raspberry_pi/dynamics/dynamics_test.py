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

def InverseDynamicsDEBUG(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, \
                    Glist, Slist):
    """Computes inverse dynamics in the space frame for an open chain robot

    :param thetalist: n-vector of joint variables
    :param dthetalist: n-vector of joint rates
    :param ddthetalist: n-vector of joint accelerations
    :param g: Gravity vector g
    :param Ftip: Spatial force applied by the end-effector expressed in frame
                 {n+1}
    :param Mlist: List of link frames {i} relative to {i-1} at the home
                  position
    :param Glist: Spatial inertia matrices Gi of the links
    :param Slist: Screw axes Si of the joints in a space frame, in the format
                  of a matrix with axes as the columns
    :return: The n-vector of required joint forces/torques
    This function uses forward-backward Newton-Euler iterations to solve the
    equation:
    taulist = Mlist(thetalist)ddthetalist + c(thetalist,dthetalist) \
              + g(thetalist) + Jtr(thetalist)Ftip

    Example Input (3 Link Robot):
        thetalist = np.array([0.1, 0.1, 0.1])
        dthetalist = np.array([0.1, 0.2, 0.3])
        ddthetalist = np.array([2, 1.5, 1])
        g = np.array([0, 0, -9.8])
        Ftip = np.array([1, 1, 1, 1, 1, 1])
        M01 = np.array([[1, 0, 0,        0],
                        [0, 1, 0,        0],
                        [0, 0, 1, 0.089159],
                        [0, 0, 0,        1]])
        M12 = np.array([[ 0, 0, 1,    0.28],
                        [ 0, 1, 0, 0.13585],
                        [-1, 0, 0,       0],
                        [ 0, 0, 0,       1]])
        M23 = np.array([[1, 0, 0,       0],
                        [0, 1, 0, -0.1197],
                        [0, 0, 1,   0.395],
                        [0, 0, 0,       1]])
        M34 = np.array([[1, 0, 0,       0],
                        [0, 1, 0,       0],
                        [0, 0, 1, 0.14225],
                        [0, 0, 0,       1]])
        G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
        G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
        G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
        Glist = np.array([G1, G2, G3])
        Mlist = np.array([M01, M12, M23, M34])
        Slist = np.array([[1, 0, 1,      0, 1,     0],
                          [0, 1, 0, -0.089, 0,     0],
                          [0, 1, 0, -0.089, 0, 0.425]]).T
    Output:
        np.array([74.69616155, -33.06766016, -3.23057314])
    """
    n = len(thetalist)
    Mi = np.eye(4)
    Ai = np.zeros((6, n))
    AdTi = [[None]] * (n + 1)
    Vi = np.zeros((6, n + 1))
    Vdi = np.zeros((6, n + 1))
    Vdi[:, 0] = np.r_[[0, 0, 0], -np.array(g)]
    AdTi[n] = mr.Adjoint(mr.TransInv(Mlist[n]))
    Fi = np.array(Ftip).copy()
    taulist = np.zeros(n)
    for i in range(n):
        Mi = np.dot(Mi,Mlist[i])
        Ai[:, i] = np.dot(mr.Adjoint(mr.TransInv(Mi)), np.array(Slist)[:, i])
        AdTi[i] = mr.Adjoint(np.dot(mr.MatrixExp6(mr.VecTose3(Ai[:, i] * \
                                            -thetalist[i])), \
                                 mr.TransInv(Mlist[i])))
        Vi[:, i + 1] = np.dot(AdTi[i], Vi[:,i]) + Ai[:, i] * dthetalist[i]
        Vdi[:, i + 1] = np.dot(AdTi[i], Vdi[:, i]) \
                       + Ai[:, i] * ddthetalist[i] \
                       + np.dot(mr.ad(Vi[:, i + 1]), Ai[:, i]) * dthetalist[i]
    for i in range (n - 1, -1, -1):
        Fi = np.dot(np.array(AdTi[i + 1]).T, Fi) \
             + np.dot(np.array(Glist[i]), Vdi[:, i + 1]) \
             - np.dot(np.array(mr.ad(Vi[:, i + 1])).T, \
                      np.dot(np.array(Glist[i]), Vi[:, i + 1]))
        taulist[i] = np.dot(np.array(Fi).T, Ai[:, i])
    return taulist

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
    """Increasing the angle of the second joint (downward) should
    make the torque for this joint (and the third) significantly larger.
    Additionally: The torque on the fourth joint should decrease, as 
    the gripper is now pointing downward."""
    theta = np.array([0,0.5*np.pi,0,0,0])
    tau2 = FeedForward(robot, theta, dtheta, ddtheta, g, FTip)
    assert abs(tau2[1]) > abs(tau[1])
    assert abs(tau2[2]) > abs(tau[2])
    assert abs(tau2[3]) < abs(tau[3])

def test_FFOptdtheta():
    theta = np.array([0,0,0,0,0])
    dtheta = np.array([0,0,0,0,0])
    ddtheta = np.array([0,0,0,0,0])
    g = np.array([0,0,-9.81])
    FTip = np.array([0,0,0,0,0,0])
    tau = FeedForward(robot, theta, dtheta, ddtheta, g, FTip)
    theta = np.array([0,0,0,0,0])
    dtheta = np.array([np.pi,0,0,0,0])
    ddtheta = np.array([0,0,0,0,0])
    g = np.array([0,0,-9.81])
    FTip = np.array([0,0,0,0,0,0])
    tau2 = FeedForward(robot, theta, dtheta, ddtheta, g, FTip)
    assert np.allclose(tau2, tau2, atol=1e-03)

    theta = np.array([0,0,0,0])
    dtheta = np.array([np.pi,0,0,0])
    ddtheta = np.array([0,0,0,0])
    Mlist = [robot.links[i].Tii for i in range(len(theta))]
    Mlist.append(robot.TsbHome)
    Glist = [robot.links[i].Gi for i in range(len(theta))]
    Slist = np.c_[robot.screwAxes[0], robot.screwAxes[1]]
    for i in range(2, len(theta)):
        Slist = np.c_[Slist, robot.screwAxes[i]]
    tauMRFirst = InverseDynamicsDEBUG(theta, dtheta, ddtheta, g, FTip, Mlist,Glist, Slist)
    assert np.allclose(tauMRFirst, tau2[0:-1])

    Slist[:, -1] = robot.screwAxes[-1]
    tauMRLast = InverseDynamicsDEBUG(theta, dtheta, ddtheta, g, FTip, Mlist,Glist, Slist)
    assert np.allclose(tauMRLast, np.r_[tau2[0:-2], tau[-1:]], atol=1e-03)

def test_FFOptddtheta():
    theta = np.array([0,0,0,0,0])
    dtheta = np.array([0,0,0,0,0])
    ddtheta = np.array([0,0,0,0,0])
    g = np.array([0,0,-9.81])
    FTip = np.array([0,0,0,0,0,0])
    tau = FeedForward(robot, theta, dtheta, ddtheta, g, FTip)
    ddtheta = np.array([0,0.25*np.pi,0,0,0])
    tau2 = FeedForward(robot, theta, dtheta, ddtheta, g, FTip)
    #Expectation: Torque is higher for at least joint 2
    assert tau2[1] > tau[1]

def test_FFOptFTip():
    theta = np.array([0,0,0,0,0])
    dtheta = np.array([0,0,0,0,0])
    ddtheta = np.array([0,0,0,0,0])
    g = np.array([0,0,-9.81])
    FTip = np.array([0,0,0,0,0,0]) 
    tau = FeedForward(robot, theta, dtheta, ddtheta, g, FTip)
    FTip = np.array([1,0,0,0,0,0]) #Pure x rotation in body frame
    tau2 = FeedForward(robot, theta, dtheta, ddtheta, g, FTip)
    print(tau)
    print(tau2)
    #Expectation: Mainly joint five will have to give additional torque
    assert tau2[-1] > tau[-1]
    FTip = np.array([0,1,0,0,0,0]) #Pure y rotation in body frame
    tau3 = FeedForward(robot, theta, dtheta, ddtheta, g, FTip)
    print(tau3)
    """Expectation: At least joint four, three, and two will have to 
                    give additional torque
    Expectation: Joint five should be barely affected
    """
    assert all(np.greater(tau3[1:-2], tau[1:-2]))
    assert np.isclose(tau3[-1], tau[-1], atol=1e-3)
    FTip = np.array([0,0,0,1,0,0])#Pure linear force in x
    tau4 = FeedForward(robot, theta, dtheta, ddtheta, g, FTip)
    print(tau4)
    """Expectation: Mainly joint two, three, and four will have to 
                    give additional torque in the home configuration
        Expectation: Joint five should be barely affected
    """
    assert all(np.greater(tau3[1:-2], tau[1:-2]))
    assert np.isclose(tau3[-1], tau[-1], atol=1e-3)



if __name__ == "__main__":
    test_FFOptFTip()
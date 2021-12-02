import os
import sys

#Find directory path of current file
current = os.path.dirname(os.path.realpath(__file__))
#Find directory path of parent folder and add to sys path
parent = os.path.dirname(current)
sys.path.append(parent)

import numpy as np
from traj_gen import TrajGen, TrajDerivatives, JointTrajLims
from classes import Link, Joint, Robot, IKAlgorithmError
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


sConfigJoint = [0, 0, 0, 0, 0]
fConfigJoint = [0.5, 0.4, 0.3, 0.2, 0.1]
sConfigOther = np.array([[1,0,0,0],
                         [0,1,0,0],
                         [0,0,1,0],
                         [0,0,0,1]])
fConfigOther = np.array([[1,0,0,0.05],
                         [0,1,0,0],
                         [0,0,1,0],
                         [0,0,0,1]])
sConfigOtherNotSE3 = np.array([[1,0,0,0],
                               [0,1,0,0],
                               [0,0,1,0],
                               [0,0,0,1]])
omgMax = 0.5
vMax = 0.5
dt = 0.01

def test_TrajLimsShort():
    thetaStart = [0,0,0,0,0]
    thetaEnd = [0.4*np.pi, 0.5*np.pi, -0.5*np.pi, 0.2*np.pi, -0.45*np.pi]
    lims = [[-0.6*np.pi, 0.6*np.pi] for i in range(5)]
    Tf = 10
    N = 10
    method = 5
    traj = JointTrajLims(thetaStart, thetaEnd, lims, Tf, N , method)
    assert np.all(traj[-1,:] == np.array(thetaEnd))

def test_TrajLimsLong():
    lims = [[-(1/4)*np.pi, (5/4)*np.pi] for i in range(5)]
    thetaStart = [-0.1*np.pi, -0.05*np.pi, 0, -0.95*np.pi, 1.05*np.pi]
    thetaEnd = [1.05*np.pi, 1*np.pi, 1.05*np.pi, 1.9*np.pi, -0.1*np.pi]
    Tf = 10
    N = 10
    method = 5
    traj = JointTrajLims(thetaStart, thetaEnd, lims, Tf, N , method)
    assert np.all(traj[:,-1] == traj[:,-2])
    assert traj[5,0] > 0 #Go through long route, in this case positive
    assert traj[5,1] > 0
    assert traj[5,2] > 0

def test_TrajLimsErr():
    lims = [[0, np.pi] for i in range(5)]
    thetaStart = [0,0,0,0,0]
    thetaEnd = [0.5*np.pi for i in range(5)]
    Tf = 10
    N = 10
    method = 5
    try:
        traj = JointTrajLims(thetaStart, thetaEnd, lims, Tf, N , method)
    except ValueError:
        assert True

def test_TrajLimsNoLims():
    lims = [[0, 0] for i in range(5)]
    thetaStart = np.random.rand(5)
    thetaEnd = np.random.rand(5)
    Tf = 10
    N = 10
    method = 5
    traj = JointTrajLims(thetaStart, thetaEnd, lims, Tf, N , method)
    assert True #No ValueError

#Tested, do not let it get in the way of other tests
robot.limList = [[0,0] for i in range(len(robot.limList))]

def test_TrajGenScrewNormal():
    trajList= TrajGen(robot, sConfigOther, fConfigOther, omgMax, 
    vMax, dt, method='screw')
    assert isinstance(trajList[0], np.ndarray)
    assert (trajList[0] == sConfigOther).all()
    assert (trajList[-1] == fConfigOther).all()

def test_TrajGenCartNormal():
    trajList = TrajGen(robot, sConfigOther, fConfigOther, omgMax, 
    vMax, dt, method='cartesian')
    assert isinstance(trajList[0], np.ndarray)
    assert (trajList[0] == sConfigOther).all()
    assert (trajList[-1] == fConfigOther).all()

def test_TrajGenJointNormal():
    trajList = TrajGen(robot, sConfigJoint, fConfigJoint, omgMax, 
    vMax, dt, method='joint')
    assert isinstance(trajList[0], np.ndarray)
    assert (trajList[0] == sConfigJoint).all()
    assert (trajList[-1] == fConfigJoint).all()

def test_TrajGenScrewNotSE3():
    try:
        trajList = TrajGen(robot, sConfigOtherNotSE3, fConfigOther, omgMax, 
        vMax, dt, method='cartesian') 
    except SyntaxError as e:
        if "input SE(3)" in e.msg:
            assert True 

def test_TrajGenCartNotNpArr():
    try:
        trajList = TrajGen(robot, sConfigJoint, fConfigOther, omgMax, 
        vMax, dt, method='cartesian') 
    except SyntaxError as e:
        if "numpy arrays" in e.msg:
            assert True 

def test_TrajGenJointNotList():
    try:
        trajList = TrajGen(robot, sConfigJoint, fConfigOther, vMax, 
        omgMax, dt, method='joint', timeScaling=5) 
    except SyntaxError as e:
        if "list of joint angles" in e.msg:
            assert True 

def test_TrajDerJoint():
    traj = TrajGen(robot, sConfigJoint, fConfigJoint, vMax, omgMax, dt, 'joint')
    trajTheta, trajV, trajA = TrajDerivatives(traj, 'joint', robot, dt)
    assert trajTheta.all() == traj.all()
    assert trajV[-1].all() == ((trajTheta[-1] - trajTheta[-2])/dt).all()
    assert trajA[-1].all() == ((trajV[-1] - trajTheta[-2])/dt).all()

def test_TrajDerScrew():
    traj = TrajGen(robot, sConfigOther, fConfigOther, vMax, omgMax, dt, 'screw')
    trajTheta, trajV, trajA = TrajDerivatives(traj, 'screw', robot, dt)
    assert np.allclose(trajV[-1], ((trajTheta[-1] - trajTheta[-2])/dt))
    assert np.allclose(trajA[-1], ((trajV[-1] - trajV[-2])/dt))

def test_TrajDerCart():
    traj = TrajGen(robot, sConfigOther, fConfigOther, vMax, omgMax, dt, 'cartesian')
    trajTheta, trajV, trajA = TrajDerivatives(traj, 'cartesian', robot, dt)
    assert np.allclose(trajV[-1], ((trajTheta[-1] - trajTheta[-2])/dt))
    assert np.allclose(trajA[-1], ((trajV[-1] - trajV[-2])/dt))

def test_TrajDerIKUnreachable():
    fConfigOther = np.array([[1,0,0,10],
                             [0,1,0,0],
                             [0,0,1,0],
                             [0,0,0,1]])
    traj = TrajGen(robot, sConfigOther, fConfigOther, vMax, omgMax, dt, 'cartesian')
    try:
        trajTheta, trajV, trajA = TrajDerivatives(traj, 'cartesian', robot, dt)
        assert False
    except IKAlgorithmError as e:
        assert True
import os
import sys

#Find directory path of current file
current = os.path.dirname(os.path.realpath(__file__))
#Find directory path of parent folder and add to sys path
parent = os.path.dirname(current)
sys.path.append(parent)

import numpy as np
from traj_gen import TrajGen, TrajDerivatives, JointTrajLims
from classes import IKAlgorithmError
from robot_init import robot


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
    assert np.all(trajTheta == traj)
    assert np.all(trajV[-2] == ((trajTheta[-1] - trajTheta[-2])/dt))
    assert np.all(trajA[-2] == ((trajV[-1] - trajV[-2])/dt))
    assert np.all(trajV[-1] == np.zeros(trajV.shape[1]))
    assert np.all(trajA[-1] == np.zeros(trajA.shape[1]))

def test_TrajDerScrew():
    traj = TrajGen(robot, sConfigOther, fConfigOther, vMax, omgMax, dt, 'screw')
    trajTheta, trajV, trajA = TrajDerivatives(traj, 'screw', robot, dt)
    assert np.allclose(trajV[-2], ((trajTheta[-1] - trajTheta[-2])/dt))
    assert np.allclose(trajA[-2], ((trajV[-1] - trajV[-2])/dt))
    assert np.all(trajV[-1] == np.zeros(trajV.shape[1]))
    assert np.all(trajA[-1] == np.zeros(trajA.shape[1]))

def test_TrajDerCart():
    traj = TrajGen(robot, sConfigOther, fConfigOther, vMax, omgMax, dt, 'cartesian')
    trajTheta, trajV, trajA = TrajDerivatives(traj, 'cartesian', robot, dt)
    assert np.allclose(trajV[-2], ((trajTheta[-1] - trajTheta[-2])/dt))
    assert np.allclose(trajA[-2], ((trajV[-1] - trajV[-2])/dt))

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
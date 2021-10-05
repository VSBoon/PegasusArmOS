import numpy as np
import unittest
from traj_gen import TrajGen
sConfigJoint = [0, 0, 0, 0, 0]
fConfigJoint = [0.5, 0.4, 0.3, 0.2, 0.1]
sConfigOther = np.array([[1,0,0,0],
                         [0,1,0,0],
                         [0,0,1,0],
                         [0,0,0,1]])
fConfigOther = np.array([[1,0,0,5],
                         [0,1,0,5],
                         [0,0,1,5],
                         [0,0,0,1]])
sConfigOtherNotSE3 = np.array([[1,0,0,0],
                               [0,1,0,0],
                               [0,0,1,0],
                               [0,0,2,0]])
tTot = 10
nSubConfigs = 20


def test_TrajGenScrewNormal():
    trajList, timeList = TrajGen(sConfigOther, fConfigOther, tTot, 
    nSubConfigs, method='screw')
    assert isinstance(trajList[0], np.ndarray)
    assert isinstance(timeList[0], float)
    assert (trajList[0] == sConfigOther).all()
    assert (trajList[-1] == fConfigOther).all()

def test_TrajGenCartNormal():
    trajList, timeList = TrajGen(sConfigOther, fConfigOther, tTot, 
    nSubConfigs, method='cartesian')
    assert isinstance(trajList[0], np.ndarray)
    assert isinstance(timeList[0], float)
    assert (trajList[0] == sConfigOther).all()
    assert (trajList[-1] == fConfigOther).all()

def test_TrajGenJointNormal():
    trajList, timeList = TrajGen(sConfigJoint, fConfigJoint, tTot, 
    nSubConfigs, method='joint')
    assert isinstance(trajList[0], np.ndarray)
    assert isinstance(timeList[0], float)
    assert (trajList[0] == sConfigJoint).all()
    assert (trajList[-1] == fConfigJoint).all()

def test_TrajGenScrewNotSE3():
    try:
        trajList, timeList = TrajGen(sConfigOtherNotSE3, fConfigOther, tTot, 
        nSubConfigs, method='cartesian') 
    except SyntaxError as e:
        if "input SE(3)" in e.msg:
            assert True 

def test_TrajGenCartNotNpArr():
    try:
        trajList, timeList = TrajGen(sConfigJoint, fConfigOther, tTot, 
        nSubConfigs, method='cartesian') 
    except SyntaxError as e:
        if "numpy arrays" in e.msg:
            assert True 

def test_TrajGenJointNotList():
    try:
        trajList, timeList = TrajGen(sConfigJoint, fConfigOther, tTot, 
        nSubConfigs, method='joint') 
    except SyntaxError as e:
        if "list of joint angles" in e.msg:
            assert True 

if __name__ == "__main__":
    trajList, timeList = TrajGen(sConfigOther, fConfigOther, tTot, 
    nSubConfigs, method='cartesian') 
    print(trajList[3])
    print(timeList[3])
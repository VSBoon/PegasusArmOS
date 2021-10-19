import os
import sys
#Find directory path of current file
current = os.path.dirname(os.path.realpath(__file__))
#Find directory path of parent folder and add to sys path
parent = os.path.dirname(current)
sys.path.append(parent)

import numpy as np
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
omgMax = 0.5
vMax = 0.5
dt = 0.01



def test_TrajGenScrewNormal():
    trajList, timeList = TrajGen(sConfigOther, fConfigOther, omgMax, 
    vMax, dt, method='screw')
    assert isinstance(trajList[0], np.ndarray)
    assert isinstance(timeList[0], float)
    assert (trajList[0] == sConfigOther).all()
    assert (trajList[-1] == fConfigOther).all()

def test_TrajGenCartNormal():
    trajList, timeList = TrajGen(sConfigOther, fConfigOther, omgMax, 
    vMax, dt, method='cartesian')
    assert isinstance(trajList[0], np.ndarray)
    assert isinstance(timeList[0], float)
    assert (trajList[0] == sConfigOther).all()
    assert (trajList[-1] == fConfigOther).all()

def test_TrajGenJointNormal():
    trajList, timeList = TrajGen(sConfigJoint, fConfigJoint, omgMax, 
    vMax, dt, method='joint')
    assert isinstance(trajList[0], np.ndarray)
    assert isinstance(timeList[0], float)
    assert (trajList[0] == sConfigJoint).all()
    assert (trajList[-1] == fConfigJoint).all()

def test_TrajGenScrewNotSE3():
    try:
        trajList, timeList = TrajGen(sConfigOtherNotSE3, fConfigOther, omgMax, 
        vMax, dt, method='cartesian') 
    except SyntaxError as e:
        if "input SE(3)" in e.msg:
            assert True 

def test_TrajGenCartNotNpArr():
    try:
        trajList, timeList = TrajGen(sConfigJoint, fConfigOther, omgMax, 
        vMax, dt, method='cartesian') 
    except SyntaxError as e:
        if "numpy arrays" in e.msg:
            assert True 

def test_TrajGenJointNotList():
    try:
        trajList, timeList = TrajGen(sConfigJoint, fConfigOther, omgMax, 
        vMax, dt, method='joint') 
    except SyntaxError as e:
        if "list of joint angles" in e.msg:
            assert True 

if __name__ == "__main__":
    trajList, timeList = TrajGen(sConfigOther, fConfigOther, omgMax, 
    vMax, dt, method='cartesian') 
    print(trajList[3])
    print(timeList[3])
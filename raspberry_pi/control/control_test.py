import os
import sys
#Find directory path of current file
current = os.path.dirname(os.path.realpath(__file__))
#Find directory path of parent folder and add to sys path
parent = os.path.dirname(current)
sys.path.append(parent)

import numpy as np
from control import PosControl
from classes import SerialData, PID
from robot_init import robot, robotFric
from util import LimDamping

"""NOTE: To run these tests, remove any notion of the localMu object in """

def test_PosCtrlNoErrors():
    serial = SerialData(6, [0,0,0,0,0,0], [1,1,1,1,1,1], [0.1,0.1,0.1,0.1,0.1,0.1], robot.joints)
    sConfig = serial.currAngle[:-1]
    eConfig = [0.2,0.2,0.2,0.2,0.2]
    dt = 0.1
    vMax = 0.25
    omgMax = 0.25
    kP = 1
    kI = 0.01
    kD = 0.01
    PIDObj = PID(kP, kI, kD, ILim=np.array([3,3,3,3,3]))
    dtComm = 0.5
    dtPID = 0.02
    PosControl(sConfig, eConfig, robot, serial, dt, vMax, omgMax, PIDObj, dtComm, dtPID)
    assert True

def test_LimDamping():
    theta = np.array([1,-1,0])*np.pi
    dtheta = np.array([0,0,0])
    limList = [[-np.pi,np.pi], [-0.5*np.pi, 0.5*np.pi], [0,0]]
    dtheta1 = LimDamping(theta, dtheta, limList)
    assert np.array_equal(dtheta1, dtheta)

    theta = np.array([1,-1,0])*np.pi
    dtheta = np.array([1,1,1])
    limList = [[-np.pi,np.pi], [-0.5*np.pi, 0.5*np.pi], [0,0]]
    dtheta1 = LimDamping(theta, dtheta, limList)
    assert dtheta1[0] < dtheta[0]
    assert dtheta1[1] == dtheta[1]
    assert dtheta1[2] == 0

    theta = np.array([1,-1,0])*np.pi
    dtheta = np.array([-0.9*np.pi, 0.45*np.pi, 0.2*np.pi])
    limList = [[-np.pi,np.pi], [-0.5*np.pi, 0.5*np.pi], [0,0]]
    dtheta1 = LimDamping(theta, dtheta, limList)
    assert dtheta1[0] == dtheta[0]
    assert dtheta1[1] == dtheta[1]
    assert dtheta1[2] == 0

if __name__ == "__main__":
    test_LimDamping()

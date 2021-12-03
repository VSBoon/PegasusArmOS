import os
import sys
#Find directory path of current file
current = os.path.dirname(os.path.realpath(__file__))
#Find directory path of parent folder and add to sys path
parent = os.path.dirname(current)
sys.path.append(parent)

import numpy as np
from control import PosControl
from classes import SerialData
from robot_init import robot, robotFric

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
    dtComm = 0.5
    dtPID = 0.02
    PosControl(sConfig, eConfig, robot, serial, dt, vMax, omgMax, kP, kI, kD, dtComm, dtPID)
    assert True

if __name__ == "__main__":
    test_PosCtrlNoErrors()
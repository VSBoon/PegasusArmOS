import os
import sys
#Find directory path of current file
current = os.path.dirname(os.path.realpath(__file__))
#Find directory path of parent folder and add to sys path
parent = os.path.dirname(current)
sys.path.append(parent)
from classes import PID
import numpy as np

ref = np.array([0, 1, 2, 3, 4])
Fdb = np.array([0, 1.1, 1.9, 3.5, 4])
kP = 1
kI = 0.1
kD = 0.02
ILim = np.array([5.,5.,5.,5.,5.])
dt = 0.01
PIDObj = PID(kP, kI, kD, ILim)
def test_PIDNormal():
    """Checks PID functionality"""
    PIDVal = PIDObj.Execute(ref, Fdb, dt)
    assert PIDVal[0] == 0
    assert PIDVal[1] < 0
    assert PIDVal[2] > 0
    

def test_PIDZeros():
    """Checks if PID function does not interfere when kP, kI, kD 
    are set to zero."""
    kP = 0
    kI = 0
    kD = 0
    PIDObj = PID(kP, kI, kD, ILim)
    PIDVal = PIDObj.Execute(ref, Fdb, dt)
    assert np.allclose(PIDVal, np.zeros(5))

def test_PIDInts():
    """Checks if errors occur if np.ndarray's are with ints"""
    ref = np.array([0, 1, 2, 3, 4])
    Fdb = np.array([0, 1, 1, 3, 4])
    kP = 1
    kI = 1
    kD = 2
    ILim = np.array([5,5,5,5,5])
    dt = 0.01
    PIDObj = PID(kP, kI, kD, ILim)
    assert True #No errors occurred

def test_PIDILim():
    """Check if anti-integral windup works as anticipated."""
    kP = 0
    kI = 0
    kD = 0
    ILim = np.array([5,5,5,5,5])
    ref = np.zeros(5)
    Fdb = np.zeros(5)
    PIDObj = PID(kP, kI, kD, ILim)
    PIDObj.termI = np.array([4.,6.,6.,6.,6.])
    PIDVal = PIDObj.Execute(ref, Fdb, dt)
    assert np.allclose(PIDObj.termI, np.array([4.,5.,5.,5.,5.]))

def test_PIDDerivative():
    """Ensures the proper 'direction' of the derivative control."""
    kP = 0
    kI = 0
    PIDObj = PID(kP, kI, kD, ILim)
    PIDVal = PIDObj.Execute(ref, Fdb, dt)
    assert PIDVal[1] < 0
    assert PIDVal[2] > 0



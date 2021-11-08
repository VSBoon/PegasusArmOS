import os
import sys
#Find directory path of current file
current = os.path.dirname(os.path.realpath(__file__))
#Find directory path of parent folder and add to sys path
parent = os.path.dirname(current)
sys.path.append(parent)
from util import PID
import numpy as np

ref = np.array([0, 1, 2, 3, 4])
Fdb = np.array([0, 1.1, 1.9, 3.5, 4])
kP = 1*np.eye(5)
kI = 0.1*np.eye(5)
kD = 0.02*np.eye(5)
ILim = np.array([5.,5.,5.,5.,5.])
dt = 0.01
errPrev = np.array([0.,0.,0.,0.,0.])

def test_PIDNormal():
    """Checks PID functionality"""
    termI = np.array([0.,0.,0.,0.,0.])
    refWPID, termI, err = PID(ref, Fdb, kP, kI, kD, termI, ILim, dt, errPrev)
    assert refWPID[0] == 0
    assert refWPID[1] < ref[1]
    assert refWPID[2] > ref[2]
    

def test_PIDZeros():
    """Checks if PID function does not interfere when kP, kI, kD 
    are set to zero."""
    kP = 0
    kI = 0
    kD = 0
    termI = np.array([0.,0.,0.,0.,0.])
    refWPID, termI, err = PID(ref, Fdb, kP, kI, kD, termI, ILim, dt, errPrev)
    assert all(refWPID == ref)

def test_PIDInts():
    """Checks if errors occur if np.ndarray's are with ints"""
    ref = np.array([0, 1, 2, 3, 4])
    Fdb = np.array([0, 1, 1, 3, 4])
    kP = 1*np.eye(5)
    kI = 1*np.eye(5)
    kD = 2*np.eye(5)
    ILim = np.array([5,5,5,5,5])
    dt = 0.01
    errPrev = np.array([0,0,0,0,0])
    assert True #No errors occurred

def test_PIDILim():
    """Check if anti-integral windup works as anticipated."""
    termI = np.array([4.,6.,6.,6.,6.])
    refWPID, termI, err = PID(ref, Fdb, kP, kI, kD, termI, ILim, dt, errPrev)
    assert all(termI == np.array([4.,5.,5.,5.,5.]))

def test_PIDDerivative():
    """Ensures the proper 'direction' of the derivative control."""
    termI = np.array([0.,0.,0.,0.,0.])
    kP = 0
    kI = 0
    refWPID, termI, err = PID(ref, Fdb, kP, kI, kD, termI, ILim, dt, errPrev)
    assert (refWPID - ref)[1] < 0
    assert (refWPID - ref)[2] > 0


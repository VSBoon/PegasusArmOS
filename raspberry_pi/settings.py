import numpy as np
from classes import PID
"""This document contains all settings that are unrelated to the robot 
model. For the robot model, see robot_init.py. If you desire to change 
settings related to main.py, kindly do so here. """
sett = dict()
sett['dtFrame'] = 0.1
sett['dtPID'] = 0.1
sett['dtComm'] = 0.1 #Ensure equal to one in Teensy code.
sett['dtFF'] = 0.1
sett['dtPosConfig'] = sett['dtFF']
sett['dtHold'] = sett['dtFF']
sett['vMax'] = 0.2
sett['wMax'] = 0.3*np.pi
sett['kP'] = np.diag(np.array([8,15,0,0,0]))
sett['kI'] = np.diag(np.array([0,0,0,0,0]))
sett['kD'] = np.diag(np.array([0,0,0,0,0]))
#sett['kP2'] = np.diag(np.array([5,5,10,0,0]))
#sett['kI2'] = np.diag(np.array([0.02,0.02,0.02,0,0]))
#sett['kD2'] = np.diag(np.array([1,1,1,0,0]))
sett['PID'] = PID(sett['kP'], sett['kI'], sett['kD'], ILim=np.array([5,5,5,5,5]))
#sett['PIDVel'] = PID(sett['kP2'], sett['kI2'], sett['kD2'], ILim=np.array([5,5,5,5,5]))
sett['jIncr'] = 0.05*np.pi
sett['eIncrLin'] = 0.05
sett['eIncrRot'] = 0.05
sett['errThetaHold'] = 0.02*np.pi
sett['forceDamp'] = 100
sett['M'] = 0.2*np.eye(6)
sett['B'] = 10*np.eye(6)
sett['Kx'] = 100*np.eye(3)
sett['Ka'] = 10*np.eye(3)

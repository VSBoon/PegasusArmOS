import numpy as np
from classes import PID
"""This document contains all settings that are unrelated to the robot 
model. For the robot model, see robot_init.py. If you desire to change 
settings related to main.py, kindly do so here. """
sett = dict()
sett['dtFrame'] = 0.03
sett['dtPID'] = 0.01
sett['dtComm'] = 0.01 #Ensure equal to one in Teensy code.
sett['dtFF'] = 0.1
sett['dtPosConfig'] = sett['dtFF']
sett['dtHold'] = sett['dtFF']
sett['vMax'] = 0.2
sett['wMax'] = 0.3*np.pi
sett['kP'] = 100
sett['kI'] = 0
sett['kD'] = 5
sett['PID'] = PID(sett['kP'], sett['kI'], sett['kD'], ILim=np.array([5,5,5,5,5]))
sett['jIncr'] = 0.05*np.pi
sett['eIncrLin'] = 0.05
sett['eIncrRot'] = 0.05
sett['errThetaHold'] = 0.02*np.pi
sett['forceDamp'] = 100
sett['M'] = 0.2*np.eye(6)
sett['B'] = 10*np.eye(6)
sett['Kx'] = 100*np.eye(3)
sett['Ka'] = 10*np.eye(3)
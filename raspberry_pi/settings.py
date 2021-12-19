import numpy as np
from classes import PID
"""This document contains all settings that are unrelated to the robot 
model. For the robot model, see robot_init.py. If you desire to change 
settings related to main.py, kindly do so here and in robot_init.py."""
sett = dict()
#Time between refreshing frames for the UI [s].
sett['dtFrame'] = 0.05
#Time between PID updates. Note: higher dtPID leads to more instability! [s].
sett['dtPID'] = 0.05
#Data communication interval with Teensy [s].
sett['dtComm'] = 0.05 #Ensure equal to one in Teensy code [s].
#Time between FF updates [s].
sett['dtFF'] = 0.05
#Time interval between trajectory sub-configurations [s].
sett['dtPosConfig'] = 0.05
#Interval for updating the FF & PID while holding a position [s].
sett['dtHold'] = sett['dtFF']
#Maximum linear velocity of the end-effector in position control [m/s].
sett['vMax'] = 0.01
#Maximum rotational velocity of the joints in position control [rad/s].
sett['wMax'] = 0.04*np.pi
#Proportional gain for position control [(N/m)/rad].
sett['kPP'] = np.diag(np.array([14,28,26,6,6]))
#Integral gain for position control [(N/m)*s/rad].
sett['kIP'] = np.diag(np.array([4,6,12,3,3]))
#Derivative gain for position control [(N/m)/(rad*s)].
sett['kDP'] = np.diag(np.array([2,2,4,1,1]))
#Proportional gain for velocity control [(N/m)/(rad/s)].
sett['kPV'] = np.diag(np.array([0,0,0,0,0]))
#Integral gain for velocity control [(N/m)*s/(rad/s)].
sett['kIV'] = np.diag(np.array([0,0,0,0,0]))
#Derivative gain for velocity control [(N/m)/(rad)].
sett['kDV'] = np.diag(np.array([0,0,0,0,0]))
#PID object for position control.
sett['PIDP'] = PID(sett['kPP'], sett['kIP'], sett['kDP'], ILim=np.array([2,2,2,2,2]))
#PID object for velocity control
sett['PIDV'] = PID(sett['kPV'], sett['kIV'], sett['kDV'], ILim=np.array([5,5,5,5,5]))
#Joint speed increase for joint velocity control when pressing the 
#'increase' button [rad/s].
sett['jIncr'] = 0.02*np.pi
#Linear end-effector speed increase for end-effector velocity control
#when pressing the 'increase' button [m/s].
sett['eIncrLin'] = 0.05
#Rotational end-effector speed increase for end-effector velocity 
#control when pressing the 'increase' button [rad/s]
sett['eIncrRot'] = 0.05
#Maximum allowed joint angle error for position control [rad].
sett['errThetaHold'] = 0.017*np.pi
#Virtual damper for force control to avoid high end-effector velocities
#when the end-effector cannot push against an object [(N/m)/(rad/s)].
sett['forceDamp'] = 100
#Virtual spatial inertia matrix (inertia+mass) matrix at the end-effector 
#for impedance control [kg*m^2 & kg].
sett['M'] = 0.2*np.eye(6)
#Virtual end-effector damper for impedance control [(N/m)/(rad/s) & N/(m/s)]
sett['B'] = 10*np.eye(6)
#Virtual linear spring for impedance control [N/m]
sett['Kx'] = 100*np.eye(3)
#Virtual rotational spring for impedance control [(N/m)/rad]
sett['Ka'] = 10*np.eye(3)
#Additional virtual damping coefficient to avoid high end-effector 
#velocities [(N/m)/(rad/s)]
sett['D'] = 4

import os
import sys
#Find directory path of current file
current = os.path.dirname(os.path.realpath(__file__))
#Find directory path of parent folder and add to sys path
parent = os.path.dirname(current)
sys.path.append(parent)

from typing import Union, List
import numpy as np
import time
import serial
from kinematics.kinematic_funcs import IKSpace
from trajectory_generation.traj_gen import TrajGen, TrajDerivatives
from dynamics.dynamics_funcs import FeedForward
from serial_comm.serial_comm import SReadAndParse
from classes import Robot, SerialData, IKAlgorithmError
from util import PID, Tau2Curr, Curr2MSpeed

def PosControl(sConfig: Union[np.ndarray, List], eConfig: Union[np.ndarray, List], robot: Robot, serial: SerialData, dt: float, vMax: float, omgMax: float, kP: float, kI: float, kD: float, dtComm: float, dtPID: float): #Removed localMu for testing
    print("Checking position inputs...")
    #Is position defined in SE3 or theta?
    if isinstance(sConfig, np.ndarray):
        if isinstance(eConfig, np.ndarray) and sConfig.shape == eConfig.shape:
            if sConfig.shape == (4,4): #Translate both to joints space 
                sConfig, successS = IKSpace(robot.TsbHome, sConfig)
                eConfig, successE = IKSpace(robot.TsbHome, eConfig)
                if not successS or not successE:
                    raise IKAlgorithmError()
        else:
            raise SyntaxError("Start- and end configurations are" +\
                              " not of the same shape or object type.")
    elif isinstance(eConfig, np.ndarray):
        raise SyntaxError("Start- and end configurations are" +\
                              " not of the same shape or object type.")
    elif not len(eConfig) == len(sConfig):
        raise SyntaxError("Start- and end configurations are" +\
                              " not of the same shape or object type.")
    #Obtain trajectory in joint space, for safety
    method = "joint"
    print("Generating trajectory...")
    traj = TrajGen(robot, sConfig, eConfig, vMax, omgMax, dt, method, 
    timeScaling=5)
    print(f"Total estimated time for trajectory: {round(dt*traj[:,0].size, 2)} s")
    traj, velTraj, accTraj = TrajDerivatives(traj, method, robot, dt)
    g = np.array([0,0,-9.81])
    FTip = np.zeros(6) #Position control, --> assume no end-effector force.
    err = np.zeros(traj[0,:].size)
    termI = np.zeros(traj[0,:].size)
    ILim = np.array([1,1,1,1,1])
    tauPID = np.zeros(traj[0,:].size)
    n = -1 #iterator
    startTime = time.perf_counter()
    lastCheck = time.perf_counter()
    lastWrite = time.perf_counter()
    lastPID = time.perf_counter()
    print("Starting trajectory...")
    while time.perf_counter() - startTime < dt*traj[:,0].size: #Trajectory loop
        nPrev = n
        n = round((time.perf_counter()-startTime)/dt)
        if n == traj[:,0].size:
            break
        if n != nPrev:
            tauFF = FeedForward(robot, traj[n], velTraj[n], accTraj[n], g, FTip)
        if time.perf_counter() - lastPID >= dtPID:
            thetaCurr = serial.currAngle[:-1] #Exclude gripper data
            errPrev = err
            tauPID, termI, err = PID(traj[n], thetaCurr, kP, kI, kD, termI, 
                                     ILim, dtPID, errPrev)
            lastPID = time.perf_counter()
        
        tau = tauFF + tauPID
        I = [Tau2Curr(tau[i], robot.joints[i].gearRatio, robot.joints[i].km, 
                      currLim=2) for i in range(len(robot.joints))]
        #TODO: Confirm conversion factor I --> PWM
        PWM = [round(Curr2MSpeed(current)) for current in I]
        serial.mSpeed[:-1] = PWM
        #TODO: Add current / PWM for gripper

        #Take care of communication on an interval basis:
        #lastCheck = SReadAndParse(serial, lastCheck, dtComm, localMu) #REMOVED FOR TESTING
        if (time.perf_counter() - lastWrite >= dtComm):
            serial.rotDirDes = [np.sign(velTraj[n,i]) if 
                                np.sign(velTraj[n,i]) == 1 else 0 for i in 
                                range(serial.lenData-1)]
            for i in range(serial.lenData-1): #TODO: Add Gripper function
                serial.dataOut[i] = f"{serial.mSpeed[i]}|"+\
                                    f"{serial.rotDirDes[i]}"
                #TODO: Replace last entry w/ gripper commands
                serial.dataOut[-1] = f"{0|0}"
                #localMu.write(f"{serial.dataOut}\n".encode('utf-8')) REMOVED FOR TESTING
                print(f"tauFF:\n{tauFF}\ntauPID:\n{tauPID}\ncurr:\n{I}\nPWM:\n{PWM}") #DEBUG
                lastWrite = time.perf_counter()
    print("Finished trajectory!")
    return None
    


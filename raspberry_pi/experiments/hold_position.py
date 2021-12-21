import os
import sys
#Find directory path of current file
current = os.path.dirname(os.path.realpath(__file__))
#Find directory path of parent folder and add to sys path
parent = os.path.dirname(current)
sys.path.append(parent)

import numpy as np
import time
import pygame
import matplotlib.pyplot as plt
from dynamics.dynamics_funcs import FeedForward
from robot_init import robot as Pegasus
from settings import sett
from typing import Tuple
from classes import SerialData, Robot, PID
from util import Tau2Curr, Curr2MSpeed, LimDamping
from serial_comm.serial_comm import StartComms, SReadAndParse
from control.control import PosControl
from main import HoldPos

dataMat = np.zeros((1,8)) #First three for encoder readings, second three for output torques
serial = SerialData(6, Pegasus.joints)
#port = FindSerial(askInput=True)[0]
Teensy = StartComms('COM13') #TEMPORARY, REPLACE WITH port
try:
    PIDObj = sett['PIDP']
    g = np.array([0,0,-9.81])
    FTip = np.zeros(6)
    PIDObj = sett['PIDP']
    dtAct = sett['dtFF']*0.5
    dtComm = sett['dtComm']
    D = sett['D'] #Absolute rotational damper
    SReadAndParse(serial, Teensy)
    thetaStart = np.array(serial.currAngle[:-1])
    #thetaDes = np.array([0*np.pi,0,0.5*np.pi,-0.5*np.pi,0.5*np.pi])
    thetaDes = np.array([0.*np.pi,0.*np.pi,0*np.pi,0*np.pi,0])
    pygame.init()
    screen = pygame.display.set_mode([700, 500])
    background = pygame.image.load(os.path.join(parent,'control_overview.png'))
    PosControl(thetaStart, thetaDes, Pegasus, serial, 0.1, 0.1, 0.1, PIDObj, dtComm, dtAct, sett['dtFrame'], Teensy, screen, background)
    lastHold = -100 #Last hold should run first
    lastComm = time.perf_counter()
    lastFrame = time.perf_counter()
    dtHold = sett['dtFF']
    dtFrame = sett['dtFrame']
    PWM = serial.mSpeed[:-1]
    newRow = np.zeros((1,8)) #TEMP
    print("Start holding")
    while True:
        if time.perf_counter() - lastHold >= dtHold:
            lastComm, lastFrame, tau = HoldPos(serial, Teensy, Pegasus, PIDObj, thetaDes, lastComm, lastFrame, dtComm, dtHold, dtFrame, screen, background)
            lastHold = time.perf_counter()
            newRow[0,0:4] = np.array([np.array(serial.currAngle[1:-1]) - thetaDes[1:]])
            newRow[0,4:] = tau[1:]
            dataMat = np.vstack((dataMat, newRow))
finally:
    np.savetxt("PIDDataLowAngle4.csv", dataMat, fmt="%.4f", delimiter=',')
    serial.dataOut = [f"{0|0}" for i in range(serial.lenData)]
    Teensy.write(f"{serial.dataOut}\n".encode('utf-8'))
    Teensy.__del__()

# dtTot = float(input("Desired total run-time [s]: "))
# name = input("Desired name of csv file (including '.csv'): ")
# start = time.perf_counter()
# while time.perf_counter() - start <= dtTot:
#     try:
#         if time.perf_counter() - lastAct >= dtAct:
#             SReadAndParse(serial, Teensy)
#             tauFF = FeedForward(Pegasus, thetaDes, dthetaDes, ddthetaDes, g, FTip)
#             thetaCurr = np.array(serial.currAngle[:-1]) #Exclude gripper data
#             thetaPrev = np.array(serial.prevAngle[:-1])
#             dthetaCurr = (thetaCurr - thetaPrev)/dtAct
#             tauPID = PIDObj.Execute(thetaDes, thetaCurr, dtAct)
#             lastPID = time.perf_counter()
#             tau = tauFF + tauPID
#             tauFric = np.zeros(5)
#             for i in range(tau.size):
#                 """Compute joint friction based on friction model of joint."""
#                 if not np.isclose(tauPID[i], 0, atol=1e-03) and np.isclose(dthetaCurr[i], 0, atol=1e-01):
#                     tauStat = Pegasus.joints[i].fricPar['stat']
#                     tauFric[i] = tauStat*np.sign(tau[i])
#                 elif np.isclose(tauPID[i], 0, atol=1e-03) and not np.isclose(dthetaCurr[i],0,atol=1e-01):
#                     tauKin = Pegasus.joints[i].fricPar['kin']
#                     tauFric[i] = tauKin*np.sign(tau[i]) 
#             tau += tauFric
#             #Diff-drive properties:
#             tauJ4 = tau[3]
#             tauJ5 = tau[4]
#             tau[3] = (-tauJ4 - tauJ5)
#             tau[4] = tauJ4 - tauJ5
#             # I = [Tau2Curr(tau[i], Pegasus.joints[i].gearRatio, Pegasus.joints[i].km, 
#             #               currLim=2) for i in range(len(Pegasus.joints))]
#             PWM = tau*33.78 #EXPERIMENTAL
#             serial.mSpeed[:-1] = PWM
#             PWM = LimDamping(serial.currAngle[:-1], PWM, Pegasus.limList, k=20)
#             PWM = [round(val) for val in PWM]
#             print(PWM)
#             serial.mSpeed[:-1] = PWM
#             serial.rotDirDes = [1 if np.sign(speed) == 1 else 0 for 
#                                         speed in serial.mSpeed]
#             for i in range(serial.lenData-1): 
#                 serial.dataOut[i] = f"{abs(serial.mSpeed[i])}|"+\
#                                     f"{serial.rotDirDes[i]}"
#             serial.dataOut[-1] = f"{0|0}"
#             Teensy.write(f"{serial.dataOut}\n".encode('utf-8')) 
#             lastAct = time.perf_counter()
#     except KeyboardInterrupt:
#         serial.dataOut = [f"{0|0}" for i in range(serial.lenData)]
#         Teensy.write(f"{serial.dataOut}\n".encode('utf-8'))
#         Teensy.__del__()
# serial.dataOut = [f"{0|0}" for i in range(serial.lenData)]
# Teensy.write(f"{serial.dataOut}\n".encode('utf-8'))
# Teensy.__del__()
# err = np.hstack((errJ4, errJ5))
# np.savetxt(name, err, delimiter=",")
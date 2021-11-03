import os
import sys
#Find directory path of current file
current = os.path.dirname(os.path.realpath(__file__))
#Find directory path of parent folder and add to sys path
parent = os.path.dirname(current)
sys.path.append(parent)

print("Importing local modules...")
from classes import SerialData, Joint, Link, Robot, Homing
from serial_comm.serial_comm import SReadAndParse, FindSerial, StartComms
print("Importing independant modules...")
import numpy as np
import time
import pygame

def SpeedUp(SPData: SerialData, nJoint: int, rotDir: int, mSpeed: int):
    """Sets motor speed for a given joint.
    :param SPData: SerialData instance for connection with the 
                   local microcontroller.
    :param nJoint: In iterator integer value, in the 
                   range [0, SPData.lenData)
    :param rotDir: Indicates desired CW (0) or CCW (1) 
                   rotation.
    :param mSpeed: PWM value indicating motor speed / current.
    """
    SPData.mSpeed[nJoint] = mSpeed
    SPData.rotDirDes[nJoint] = rotDir

def Break(SPData: SerialData, nJoint: int):
    """Commands the motors to stop turning.
    :param SPData: SerialData instance for connection with the 
                   local microcontroller.
    :param nJoint: In iterator integer value, in the 
                   range [0, SPData.lenData)
    """
    #TODO: ADD minSpeed to counter gravity
    SPData.mSpeed[nJoint] = 0

def ChangeSpeed(mSpeedSel: int, mSpeedMin: int, mSpeedMax: int, dSpeed: int, 
                incr: bool):
    """Changes the set motor speed iteratively.
    All values should be an integer in the range [0, 255].
    :param mSpeedSel: The current selected motor speed.
    :param mSpeedMin: The minimal motor speed.
    :param mSpeedMax: The maximal motor speed.
    :param dSpeed: Change in speed per click.
    :param incr: Increase (True) or Decrease (False) speed.
    """
    if incr:
        if (mSpeedMax - mSpeedMin) < dSpeed:
            mSpeedSel = mSpeedMax
        else:
            mSpeedSel += dSpeed
    else:
        if (mSpeedSel - mSpeedMin) < dSpeed:
            mSpeedSel = mSpeedMin
        else:
            mSpeedSel -= dSpeed
    print(f"speed: {mSpeedSel}")
    return mSpeedSel

def PegasusManualControl():
    ### INTANTIATE ROBOT INSTANCE ###
    #Inertia matrices
    iMat0 = np.diag([0.03584238, 0.02950513, 0.04859042])
    iMat1 = np.diag([0.00393345, 0.00236823, 0.00171701])
    iMat2 = np.diag([0.0092912, 0.00210452, 0.0029424])
    iMat34 = np.diag([0.00363966, 0.00347835, 0.00041124])
    massList = [4.99, 0.507, 0.420, 0.952, 0.952]
    #Transformation matrices from CoM of links with principle axes of
    #inertia to the space frame (Tsi):
    Tsi0 = np.array([[0.3804 ,-0.9215,0.0786 ,-0.0103],
                     [0.8774 ,0.3864 ,0.2843 ,-0.0292],
                     [-0.2924,-0.0392,0.9555 ,0.0642 ],
                     [0      ,0      ,0      ,1      ]])
    Tsi1 = np.array([[-0.0002,0.0008 ,-1.0000,0.0350 ],
                     [0.4553 ,0.8903 ,0.0007 ,-0.0083],
                     [0.8903 ,-0.4553,-0.0006,0.1666 ],
                     [0      ,0      ,0      ,1      ]])
    Tsi2 = np.array([[-0.9966,0.0819 ,-0.0003,0.0814 ],
                     [-0.0819,-0.9966,0.0008 ,-0.0083],
                     [-0.0002,0.0008 ,1.0000 ,0.3550 ],
                     [0      ,0      ,0      ,1      ]])
    Tsi34 = np.array([[-0.0326,0.0002 ,-0.9995,0.2640 ],
                      [-0.0004,1.0000 ,0.0002 ,-0.0059],
                      [-0.9995,0.0004 ,-0.0326,0.3504 ],
                      [0      ,0      ,0      ,1      ]])
    #Screw axes in the Space Frame {s}
    S0 = np.array([0,0,1,0,0,0])
    S1 = np.array([0,1,0,-0.0035,0,0.126])
    S2 = np.array([0,1,0,-0.0035,0,0.335])
    S3 = np.array([0,1,0,-0.234,0,0.355])
    S4 = np.array([0,0,1,-0.234,-0.016,0])
    #Joint limits in home configuration, of the form [lower, upper]:
    lims0 = [-0.945*np.pi, 0.945*np.pi] #+/- 170 deg
    lims1 = [-0.25*np.pi, 0.5*np.pi] #-45 deg, + 90 deg
    lims2 = [-np.pi, 0.25*np.pi] #-180 deg, + 45 deg.
    lims3 = [-np.pi, 0.25*np.pi] #-180 deg, +45 deg.
    lims4 = [-np.pi, np.pi] #+/- 180 deg.
    limsTest = [-0.1*np.pi, 0.1*np.pi]
    cpr = 1440
    gearRatioList = [19.7*50, 19.7*50, (65.5*20)/9, (65.5*20)/9, (127.7*32)/9]
    L0 = Link(iMat0, massList[0], None, Tsi0)
    L1 = Link(iMat1, massList[1], L0, Tsi1)
    L2 = Link(iMat2, massList[2], L1, Tsi2)
    L34 = Link(iMat34, massList[3], L2, Tsi34)
    J0 = Joint(S0, [None, L0], gearRatioList[0], cpr, limsTest) #REMOVE LIMSTEST FOR LIMS0!
    J1 = Joint(S1, [L0, L1], gearRatioList[1], cpr, lims1)
    J2 = Joint(S2, [L1, L2], gearRatioList[2], cpr, lims2)
    J3 = Joint(S3, [L2,34], gearRatioList[3], cpr, lims3)
    J4 = Joint(S4, [L2,L34], gearRatioList[4], cpr, lims4)
    Pegasus = Robot([J0, J1, J2, J3, J4], [L0, L1, L2, L34])
    ### END OF ROBOT INITIATION ###

    ### SETUP SERIAL COMMUNICATION ###
    baudRate = 115200
    lenData = len(Pegasus.joints)
    desAngles = [0 for i in range(lenData)]
    maxDeltaAngles = [np.pi for i in range(lenData)]
    tolAngles = [0.001*np.pi for i in range(lenData)]
    SPData = SerialData(lenData, desAngles, maxDeltaAngles, tolAngles, Pegasus.joints)
    dtComm = 0.005 #Make sure this aligns with dtComm in C++ code.
    dtPrint = 1
    dtInput = 0.03 #approximately 30 fps, also avoids window crashing
    port, warning = FindSerial(askInput=True)
    localMu = StartComms(port, baudRate)
    encAlg = "utf-8"
    ### END OF SERIAL COMMUNICATION SETUP ###
    homingPins = [3, 5, 7, 29, 31, 26]
    homeObj = Homing(homingPins)

    mSpeedMax = int(input("mSpeedMax (0 - 255): "))
    if mSpeedMax < 0 or mSpeedMax > 255:
        print("Invalid entry, putting mSpeedMax at 120.")
        mSpeedMax = 120
    mSpeedMin = int(input("mSpeedMin (0 - 255): "))
    if mSpeedMin > mSpeedMax:
        print("mSpeedMin must be smaller than mSpeedMax.") 
        if mSpeedMax - 10 > 0:
            print(f"Setting mSpeedMin to {mSpeedMax - 10}.")
            mSpeedMin = mSpeedMax - 10
        else:
            print("Setting mSpeedMin to 0")
            mSpeedMin = 0
    
    mSpeedSel = mSpeedMin

    pygame.init()
    screen = pygame.display.set_mode([500, 500])
    print("Starting main loop.\n Type Ctrl+C to stop")
    print("Press 'C' to change from fast to slow speed.")
    try:
        lastCheck = time.time()
        lastWrite = time.time()
        lastPrint = time.time()
        lastInput = time.time()
        print(f"speed: {mSpeedSel}")
        while True:
            lastCheck = SReadAndParse(SPData, lastCheck, dtComm, 
                                      localMu, homeObj, encAlg)[0]
            if (time.time() - lastWrite >= dtComm):
                #Check for each motor if the current move is allowed 
                #within the joint limits
                SPData.CheckJointLim()
                for i in range(SPData.lenData):
                    SPData.dataOut[i] = f"{SPData.mSpeed[i]}|" + \
                                        f"{SPData.rotDirDes[i]}"
                localMu.write(f"{SPData.dataOut}\n".encode(encAlg))
                lastWrite = time.time()
            if (time.time() - lastPrint >= dtPrint):
                print(f"Speed: {SPData.mSpeed}, Direction: {SPData.rotDirDes}")
                print(f"Count: {SPData.totCount}, Homing: {SPData.homing}")
                lastPrint = time.time()
            if (time.time() - lastInput >= dtInput):
                #Check for key-press, act accordingly
                events = pygame.event.get()
                for event in events:
                    if event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_q:
                            SpeedUp(SPData, 0, 1, mSpeedSel)
                        elif event.key == pygame.K_a:
                            SpeedUp(SPData, 0, 0, mSpeedSel)
                        elif event.key == pygame.K_w:
                            SpeedUp(SPData, 1, 1, mSpeedSel)
                        elif event.key == pygame.K_s:
                            SpeedUp(SPData, 1, 0, mSpeedSel)
                        elif event.key == pygame.K_e:
                            SpeedUp(SPData, 2, 1, mSpeedSel)
                        elif event.key == pygame.K_d:
                            SpeedUp(SPData, 2, 0, mSpeedSel)
                        elif event.key == pygame.K_r:
                            SpeedUp(SPData, 3, 1, mSpeedSel)
                        elif event.key == pygame.K_f:
                            SpeedUp(SPData, 3, 0, mSpeedSel)
                        elif event.key == pygame.K_t:
                            SpeedUp(SPData, 4, 1, mSpeedSel)
                        elif event.key == pygame.K_g:
                            SpeedUp(SPData, 4, 0, mSpeedSel)
                        elif event.key == pygame.K_c:
                            mSpeedSel = ChangeSpeed(mSpeedSel, mSpeedMin, 
                                                    mSpeedMax, 5, True)
                        elif event.key == pygame.K_x:
                            mSpeedSel = ChangeSpeed(mSpeedSel, mSpeedMin, 
                                                    mSpeedMax, 5, False)

                    elif event.type == pygame.KEYUP:
                        if event.key == pygame.K_q or event.key == pygame.K_a:
                            Break(SPData, 0)
                        elif event.key == pygame.K_w or event.key == pygame.K_s:
                            Break(SPData, 1)
                        elif event.key == pygame.K_e or event.key == pygame.K_d:
                            Break(SPData, 2)
                        elif event.key == pygame.K_r or event.key == pygame.K_f:
                            Break(SPData, 3)
                        elif event.key == pygame.K_t or event.key == pygame.K_g:
                            Break(SPData, 4)
                    elif event.type == pygame.QUIT:
                        raise KeyboardInterrupt()
                lastInput = time.time()



    except KeyboardInterrupt:
        #Set motor speeds to zero & close serial.
        localMu.write(f"{['0|0'] * lenData}\n".encode(encAlg))
        time.sleep(dtComm)
        localMu.__del__()
        print("Quitting...")
    
    finally: #Always clean RPi pins!
        homeObj.CleanPins()
        return 0

if __name__ == "__main__":
    PegasusManualControl()
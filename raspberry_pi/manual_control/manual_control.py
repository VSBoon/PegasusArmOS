import os
import sys

from kinematics.kinematic_funcs import IKSpace
#Find directory path of current file
current = os.path.dirname(os.path.realpath(__file__))
#Find directory path of parent folder and add to sys path
parent = os.path.dirname(current)
sys.path.append(parent)

print("Importing local modules...")
from classes import InputError, SerialData, Joint, Link, Robot, Homing
from util import PID, Tau2Curr, Curr2MSpeed
from serial_comm.serial_comm import SReadAndParse, FindSerial, StartComms
from kinematics.kinematic_funcs import IKSpace
from dynamics.dynamics_funcs import FeedForward
print("Importing independant modules...")
from typing import Tuple, Union, List
import serial
import modern_robotics as mr
import numpy as np
import time
import pygame

def HoldPos(pos: Union[np.ndarray[float], List], pegasus: Robot, 
            SPData: SerialData, kP: np.ndarray[float], 
            kI: np.ndarray[float], kD: np.ndarray[float], 
            termI: np.ndarray[float], ILim: np.ndarray[float], dt: float, 
            errPrev: np.ndarray[float]) -> Tuple[List. np.ndarray[float]]:
    """Hold a given position using feed forward and PID control.
    :param pos: Desired configuration to hold, either as a list of 
                joint angles or the SE(3) end-effector configuration.
    :param pegasus: Robot object representing the arm.
    :param SPData: SerialData object for communication and keeping 
                   joint angles etc.
    :param kP: nxn proportional matrix, typically an identity
               matrix times a constant.
    :param kI: nxn integral matrix, typically an identity matrix
               times a constant.
    :param kD: nxn difference matrix, typically an identity matrix
               times a constant.
    NOTE: To omit P-, I-, or D action, input kX = 0
    :param termI: Accumulative integral term.
    :param ILim: Integral term limiter for anti-integral windup.
    :param dt: Time between each error calculation in seconds.
    :param errPrev: error value from the previous PID control loop.
    :return mSpeed: List of PWM motor speeds, in the range [0, 255].
    NOTE: mSpeed should be remapped to [mSpeedMin, mSpeedMax].
    :return termI: New accumulative intergral term.
    """
    #Translate pos into joint space
    if type(pos) == list:
        pass
    elif pos.shape == (4,4): #Transformation matrix
        isSE3 = mr.TestIfSE3(pos)
        if not isSE3:
            raise InputError("pos is neither a list of joint angles nor a " +\
                             "transformation matrix.")
        pos, success = IKSpace()
        if not success:
            raise ValueError("no solution to the IK problem was found.")
    else:
        raise InputError("pos is neither a list of joint angles nor a " +\
                             "transformation matrix.")
    dThetaDes = np.array([0 for i in range(len(pos))])
    ddThetaDes = dThetaDes
    FtipDes = np.array([0 for i in range(6)])
    feedForwardT, termI = FeedForward(pegasus, pos, dThetaDes, ddThetaDes, FtipDes)
    #NOTE: PID implicitely translates joint angle error to torques! 
    #(i.e. kP is in Nm/rad, etc.)
    PIDT, termI = PID(pos, SPData.currAngle, kP, kI, kD, termI, ILim, dt, errPrev)
    FFwPID = feedForwardT + PIDT
    mSpeed = [Curr2MSpeed(Tau2Curr(FFwPID[i])) for i in range(FFwPID.size)]
    return mSpeed, termI

def SpeedUpJ(SPData: SerialData, nJoint: int, rotDir: int, mSpeed: int):
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

def BreakJ(SPData: SerialData, nJoint: int):
    """Commands the motors to stop turning.
    :param SPData: SerialData instance for connection with the 
                   local microcontroller.
    :param nJoint: In iterator integer value, in the 
                   range [0, SPData.lenData)
    """
    #TODO: ADD minSpeed to counter gravity
    SPData.mSpeed[nJoint] = 0

def ChangeSpeedJ(mSpeedSel: int, mSpeedMin: int, mSpeedMax: int, dSpeed: int, 
                incr: bool):
    """Changes the set motor speed iteratively.
    All values should be an integer in the range [0, 255].
    :param mSpeedSel: The current selected motor speed.
    :param mSpeedMin: The minimal motor speed.
    :param mSpeedMax: The maximal motor speed.
    :param dSpeed: Change in speed per click.
    :param incr: Increase (True) or Decrease (False) speed.
    :return mSpeedSel: New selected motor speed in [0, 255]
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

def CheckKeysJoint(SPData: SerialData, mSpeedMin: int, mSpeedMax: int,
                   mSpeedSel: int):
    """Checks for key-presses and executes joint control commands 
    accordingly.
    All mSpeed values should be an integer in the range [0, 255].
    :param SPData: SerialData object for communication.
    :param mSpeedSel: The current selected motor speed.
    :param mSpeedMin: The minimal motor speed.
    :param mSpeedMax: The maximal motor speed.
    """
    events = pygame.event.get()
    for event in events:
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                SpeedUpJ(SPData, 0, 1, mSpeedSel)
            elif event.key == pygame.K_a:
                SpeedUpJ(SPData, 0, 0, mSpeedSel)
            elif event.key == pygame.K_w:
                SpeedUpJ(SPData, 1, 1, mSpeedSel)
            elif event.key == pygame.K_s:
                SpeedUpJ(SPData, 1, 0, mSpeedSel)
            elif event.key == pygame.K_e:
                SpeedUpJ(SPData, 2, 1, mSpeedSel)
            elif event.key == pygame.K_d:
                SpeedUpJ(SPData, 2, 0, mSpeedSel)
            elif event.key == pygame.K_r:
                SpeedUpJ(SPData, 3, 1, mSpeedSel)
            elif event.key == pygame.K_f:
                SpeedUpJ(SPData, 3, 0, mSpeedSel)
            elif event.key == pygame.K_t:
                SpeedUpJ(SPData, 4, 1, mSpeedSel)
            elif event.key == pygame.K_g:
                SpeedUpJ(SPData, 4, 0, mSpeedSel)
            elif event.key == pygame.K_c:
                mSpeedSel = ChangeSpeedJ(mSpeedSel, mSpeedMin, 
                                        mSpeedMax, 5, True)
            elif event.key == pygame.K_x:
                mSpeedSel = ChangeSpeedJ(mSpeedSel, mSpeedMin, 
                                        mSpeedMax, 5, False)

        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_q or event.key == pygame.K_a:
                BreakJ(SPData, 0)
            elif event.key == pygame.K_w or event.key == pygame.K_s:
                BreakJ(SPData, 1)
            elif event.key == pygame.K_e or event.key == pygame.K_d:
                BreakJ(SPData, 2)
            elif event.key == pygame.K_r or event.key == pygame.K_f:
                BreakJ(SPData, 3)
            elif event.key == pygame.K_t or event.key == pygame.K_g:
                BreakJ(SPData, 4)
        elif event.type == pygame.QUIT:
            raise KeyboardInterrupt()

def PegasusJointControl(SPData: SerialData, localMu: serial.Serial, dtComm:
                        float, dtPrint: float, dtInput: float, homeObj: Homing, encAlg: str = 'utf-8'):
    """Allows one to move each joint of the Pegasus arm independantly.
    :param SPData: SerialData object for communication.
    :param localMu: Serial object to connect to local microcontroller.
    :param dtComm: Interval between sending & receiving data from 
                   the local microcontroller in seconds.
    :param dtInput: Interval of checking for user inputs in seconds.
    :param dtPrint: Interval between printing robot data on the 
                    terminal in seconds.
    :param homeObj: Homing object reading & storing homing sensor data.
    :param encAlg: Encoding algorithm for serial communication."""

    #Set speeds based on user input
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
    print("Press 'C' to increment speed, 'X' to decrement.")
    print(f"speed: {mSpeedSel}")

    pygame.init()
    screen = pygame.display.set_mode([500, 500])
    print("Starting main loop.\n Type Ctrl+C to stop")
    lastCheck = time.time()
    lastWrite = time.time()
    lastPrint = time.time()
    lastInput = time.time()
    try:
        while True: #Main loop
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
                CheckKeysJoint(SPData, mSpeedMin, mSpeedMax, mSpeedSel)
                lastInput = time.time()

    except KeyboardInterrupt:
        #Set motor speeds to zero & close serial.
        localMu.write(f"{['0|0'] * SPData.lenData}\n".encode(encAlg))
        time.sleep(dtComm)
        localMu.__del__()
        print("Quitting...")
    
    finally: #Always clean RPi pins!
        homeObj.CleanPins()
        return None

def CheckKeysEF(SPData: SerialData, vMin: float, vMax: float, vSel: float, 
                wMin: float, wMax: float, wSel: float, dVel: float= 0.05) \
                -> Tuple(np.ndarray[float], float, float):
    """Checks for key-presses and alter velocity components
    and other factors accordingly.\n
    KEY-BINDINGS (all in the space frame {s}):
    w/s: Move in +/- x-direction.
    a/d: Move in +/- y-direction.
    z/x: Move in +/- z-direction.
    q/e: Rotate +/- around x-axis.
    r/f: Rotate +/- around y-axis.
    c/v: Rotate +/- around z-axis.
    t/y: Increment/decrement linear velocity.
    g/h: Increment/decrement angular velocity.\n
    :param SPData: SerialData object for communication.
    :param vSel: Previously selected linear velocity in [m/s].
    :param wSel: Previously selected angular velocity in [rad/s].
    :param vMax: Maximum linear end-effector velocity in [m/s].
    :param vMin: Minimum linear end-effector velocity in [m/s].
    :param wMax: Maximum angular end-effector velocity in [rad/s].
    :param wMin: Minimum angular end-effector velocity in [rad/s].
    :param dVel: Change in velocity per key press, in [m/s] & [rad/s].
    :return V: 6x1 velocity twist.
    :return vSel: Newly selected linear velocity in [m/s].
    :return wSel: Newly selected angular velocity in [rad/s].
    """
    V = [0 for i in range(6)]
    if SPData.limBool.any():
        limFactor = 0
    events = pygame.event.get()
    for event in events:
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                V[3] = vSel * limFactor
            elif event.key == pygame.K_s:
                V[3] = -vSel * limFactor
            elif event.key == pygame.K_a:
                V[4] = vSel * limFactor
            elif event.key == pygame.K_d:
                V[4] = -vSel * limFactor
            elif event.key == pygame.K_z:
                V[5] = vSel * limFactor
            elif event.key == pygame.K_x:
                V[5] = -vSel * limFactor
            elif event.key == pygame.K_q:
                V[0] = wSel * limFactor
            elif event.key == pygame.K_e:
                V[0] = -wSel * limFactor
            elif event.key == pygame.K_r:
                V[1] = wSel * limFactor
            elif event.key == pygame.K_f:
                V[1] = -wSel * limFactor
            elif event.key == pygame.K_c:
                V[2] = wSel * limFactor
            elif event.key == pygame.K_v:
                V[2] = -wSel * limFactor
            elif event.key == pygame.K_t:
                if (vSel + dVel) < vMax:
                    vSel += dVel
                else:
                    vSel = vMax
            elif event.key == pygame.K_y:
                if (vSel - dVel) < vMin:
                    vSel -= dVel
                else:
                    vSel = vMin
            elif event.key == pygame.K_g:
                if (wSel + dVel) < wMax:
                    wSel -= dVel
                else:
                    wSel = wMax
                print(f"linear velocity: {vSel} m/s")
            elif event.key == pygame.K_h:
                if (wSel - dVel) < wMin:
                    wSel -= dVel
                else:
                    wSel = wMin
                print(f"angular velocity: {wSel} rad/s")

        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_w or event.key == pygame.K_s:
                V[3] = 0
            elif event.key == pygame.K_a or event.key == pygame.K_d:
                V[4] = 0
            elif event.key == pygame.K_z or event.key == pygame.K_x:
                V[5] = 0
            elif event.key == pygame.K_q or event.key == pygame.K_e:
                V[0] = 0
            elif event.key == pygame.K_r or event.key == pygame.K_f:
                V[1] = 0
            elif event.key == pygame.K_c or event.key == pygame.K_v:
                V[2] = 0
        elif event.type == pygame.QUIT:
            raise KeyboardInterrupt()
        return V, vSel, wSel

def DThetaToComm(SPData: SerialData, dtheta: np.ndarray[float]) \
                -> np.ndarray[float]:
    """Translated the desired angular velocities along the screw axes
    into the desired angular velocities at the input shafts of the 
    Amatrol Pegasus robot arm.
    :param dtheta: Array of angular velocities along the screw axes of
                   the robot, as defined in its Joint objects.
    :return dthetaComm: Desired angular velocities at the input shafts 
                        of the robot arm.
    """
    dthetaComm = dtheta
    """Translate virtual joint velocities of joint 4 & 5 
    into actual motor velocities. TODO: CONFIRM MOVEMENT DIRECTION!"""
    dthetaComm[3] = dtheta[3] - dtheta[4]
    dthetaComm[4] = dtheta[3] + dtheta[4]
    gearRatioArr = np.array([SPData.joints[i].gearRatio for i in 
                             range(SPData.lenData)])
    dthetaComm = np.multiply(dtheta, gearRatioArr)
    return dthetaComm

def PegasusEFControl(SPData: SerialData, localMu: serial.Serial, dtComm:
                        float, dtPrint: float, dtInput: float, homeObj: Homing, encAlg: str = 'utf-8'):
    """Allows one to control the Pegasus robot arm in end-effector space.
    :param :"""
    #Absolute maximum and minimum linear- & rotational speeds in SI.
    vMaxLim = 0.5
    vMinLim = 0
    wMaxLim = 1
    wMinLim = 0
    #Set speeds from user input:
    try:
        vMax = int(input("vMax (m/s), make sure < 0.5 m/s: "))
    except ValueError as e:
        if "invalid literal for int()" in e.message:
            print("Invalid input, setting vMax to 0.2 m/s")
        else:
            print("Unkown error, setting vMax to 0.2 m/s")
        vMax = 0.2
    if vMax > vMaxLim:
        print(f"Invalid entry: {vMax} > 0.5. Setting vMax = {vMaxLim}")
        vMax = vMaxLim
    try:
        vMin = int(input("vMin (m/s), make sure > 0 m/s: "))
    except ValueError as e:
        if "invalid literal for int()" in e.message:
            print("Invalid input, setting vMin to 0 m/s")
        else:
            print("Unkown error, setting vMin to 0 m/s")
        vMin = 0
    if vMin < vMinLim:
        print(f"Invalid entry. {vMin} < 0. Setting vMin = 0")
        vMin = 0
    elif vMin > vMax:
        print(f"Invalid entry. {vMin} > vMax. Setting vMin = {vMax - 0.1}")
        vMin = vMax - 0.1
    try:
        wMax = int(input("wMax (rad/s), make sure < 1 rad/s: "))
    except ValueError as e:
        if "invalid literal for int()" in e.message:
            print("Invalid input, setting wMax to 0.5 rad/s")
        else:
            print("Unkown error, setting wMax to 0.5 m/s")
        wMax = 0.5
    if wMax > wMaxLim:
       print(f"Invalid entry: {wMax} > 1, Setting wMax = {wMaxLim}") 
    try:
        wMin = int(input("wMin (rad/s), make sure > 0 rad/s: "))
    except ValueError as e:
        if "invalid literal for int()" in e.message:
            print("Invalid input, setting wMin to 0 rad/s")
        else:
            print("Unkown error, setting wMin to 0 rad/s")
        wMin = 0.5
    if wMin < wMinLim:
        print(f"Invalid entry. {wMin} < 0. Setting wMin = 0")
        wMin = 0
    elif wMin > wMax:
        print(f"Invalid entry. {wMin} > wMax. Setting wMin = {wMax - 0.1}")
        wMin = wMax - 0.1
    vSel = [vMax*0.05 if vMax*0.05 > vMin else vMin][0]
    wSel = [wMax*0.05 if wMax*0.05 > wMin else wMin][0]
    print("Press 'T/Y' to increment/decrement linear velocity.")
    print("Press 'G/H' to increment/decrement angular velocity.")
    print(f"linear velocity: {vSel}")
    print(f"angular velocity: {wSel}")
    print("Starting main loop.\n Type Ctrl+C to stop")
    lastCheck = time.time()
    lastWrite = time.time()
    lastPrint = time.time()
    lastInput = time.time()
    screwAxes = [SPData.joints[i].screwAx for i in range(SPData.lenData)]
    try:
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
                    vels = [vMin, vMax, vSel, wMin, wMax, wSel]
                    V, vSel, wSel = CheckKeysEF(SPData, *vels)
                    JSpace = mr.JacobianSpace(screwAxes, SPData.currAngle)
                    dtheta = np.dot(np.linalg.pinv(JSpace), V)
                    #Translate (virtual) joint speeds to motor speeds:
                    dthetaComm = DThetaToComm(SPData, dtheta)
                    #TODO: Translate into mSpeed variables!


    except KeyboardInterrupt:
        #Set motor speeds to zero & close serial.
        localMu.write(f"{['0|0'] * SPData.lenData}\n".encode(encAlg))
        time.sleep(dtComm)
        localMu.__del__()
        print("Quitting...")
    
    finally: #Always clean RPi pins!
        homeObj.CleanPins()
        return None
    
def PegasusManualControl(method="joints"):
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
    if method == 'joint':
        PegasusJointControl(SPData, localMu, dtComm, dtPrint, dtInput, 
                            homeObj, encAlg)

    #elif method == 'end-effector':
        #PegasusEFControl
    
    return

if __name__ == "__main__":
    method = input("Please specify the desired type of control:\n" +
                   "For joint control, type 'joint'. \n" +  
                   "For end-effector control, type 'end-effector'")
    methodBool = False
    while not methodBool:
        if method == 'joint' or method == 'end-effector':
            methodBool = True
        else:
            method = input("invalid control method, please choose " + 
                           "between 'joint' and 'end-effector'.")
    PegasusManualControl(method)
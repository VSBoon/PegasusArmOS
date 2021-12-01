import os
import sys

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
from typing import Tuple, Union, List, Dict
import serial
import modern_robotics as mr
import numpy as np
import time
import pygame

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

def HoldPos(pos: Union["np.ndarray[float]", List], pegasus: Robot, 
            SPData: SerialData, kP: "np.ndarray[float]", 
            kI: "np.ndarray[float]", kD: "np.ndarray[float]", 
            termI: "np.ndarray[float]", ILim: "np.ndarray[float]", dt: float, 
            errPrev: "np.ndarray[float]") -> Tuple[List, "np.ndarray[float]"]:
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
    :return err: Angle error of the current loop.
    """
    #Translate pos into joint space
    if type(pos) == list:
        pos = np.array(pos)
    elif pos.size == errPrev.size:
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
    feedForwardT = FeedForward(pegasus, pos, dThetaDes, ddThetaDes, FtipDes)
    """NOTE: This PID implicitely translates joint angle error to 
    torques! (i.e. kP is in Nm/rad, etc.). Normally this is done 
    explicitely by multiplying kP, kI, and kD with the mass matrix, 
    but since the mass matrix is questionable in this case, we omit it.
    """
    PIDT, termI, err = PID(pos, np.array(SPData.currAngle), kP, kI, kD, 
                           termI, ILim, dt, errPrev)
    FFwPID = np.add(feedForwardT, PIDT)
    current = [Tau2Curr(FFwPID[i], pegasus.joints[i].gearRatio, 
              pegasus.joints[i].km, 2) for i in range(FFwPID.size)]
    
    mSpeed = [Curr2MSpeed(current[i]) for i in range(FFwPID.size)]
    print(f"PID+FF\n{FFwPID}\ncurr\n{current}\nmSpeed\n{mSpeed}") #Debug
    return mSpeed, termI, err

def SpeedUpJ(SPData: SerialData, nJoint: int, rotDir: int, mSpeed: int):
    """Sets motor speed for a given joint.
    :param SPData: SerialData instance for connection with the 
                   local microcontroller.
    :param nJoint: In iterator integer value, in the 
                   range [0, SPData.lenData-1]
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
                   range [0, SPData.lenData-1)
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

def CheckKeysJoint(SPData: SerialData, pressed: Dict[str, bool], 
                   mSpeedMin: int, mSpeedMax: int, mSpeedSel: int) \
                   -> Tuple[bool, Dict[str, bool], int]:
    """Checks for key-presses and executes joint control commands 
    accordingly.
    All mSpeed values should be an integer in the range [0, 255].
    :param SPData: SerialData object for communication.
    :param pressed: Dictionary of booleans to keep track of keyboard 
                    status.
    :param mSpeedSel: The current selected motor speed.
    :param mSpeedMin: The minimal motor speed.
    :param mSpeedMax: The maximal motor speed.
    :return noInput: Bool indicating no new keyboard events.
    :return pressed: Updated dictionary of booleans to keep track of
                     of keyboard status.
    :return mSpeedSel: Selected motor speed
    """
    events = pygame.event.get()
    if len(events) == 0:
        noInput = True
    else:
        noInput = False

    for event in events:
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                pressed['q'] = True
                SpeedUpJ(SPData, 0, 1, mSpeedSel)
            elif event.key == pygame.K_a:
                pressed['a'] = True
                SpeedUpJ(SPData, 0, 0, mSpeedSel)
            elif event.key == pygame.K_w:
                pressed['w'] = True
                SpeedUpJ(SPData, 1, 1, mSpeedSel)
            elif event.key == pygame.K_s:
                pressed['s'] = True
                SpeedUpJ(SPData, 1, 0, mSpeedSel)
            elif event.key == pygame.K_e:
                pressed['e'] = True
                SpeedUpJ(SPData, 2, 1, mSpeedSel)
            elif event.key == pygame.K_d:
                pressed['d'] = True
                SpeedUpJ(SPData, 2, 0, mSpeedSel)
            elif event.key == pygame.K_r:
                pressed['r'] = True
                SpeedUpJ(SPData, 3, 1, mSpeedSel)
            elif event.key == pygame.K_f:
                pressed['f'] = True
                SpeedUpJ(SPData, 3, 0, mSpeedSel)
            elif event.key == pygame.K_t:
                pressed['t'] = True
                SpeedUpJ(SPData, 4, 1, mSpeedSel)
            elif event.key == pygame.K_g:
                pressed['g'] = True
                SpeedUpJ(SPData, 4, 0, mSpeedSel)
            elif event.key == pygame.K_c:
                mSpeedSel = ChangeSpeedJ(mSpeedSel, mSpeedMin, 
                                        mSpeedMax, 5, True)
            elif event.key == pygame.K_x:
                mSpeedSel = ChangeSpeedJ(mSpeedSel, mSpeedMin, 
                                        mSpeedMax, 5, False)

        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_q:
                pressed['q'] = False
                BreakJ(SPData, 0)
            elif event.key == pygame.K_a:
                pressed['a'] = False
                BreakJ(SPData, 0)
            elif event.key == pygame.K_w:
                pressed['w'] = False
                BreakJ(SPData, 1)
            elif event.key == pygame.K_s:
                pressed['s'] = False
                BreakJ(SPData, 1)
            elif event.key == pygame.K_e:
                pressed['e'] = False
                BreakJ(SPData, 2)
            elif event.key == pygame.K_d:
                pressed['d'] = False
                BreakJ(SPData, 2)
            elif event.key == pygame.K_r:
                pressed['r'] = False
                BreakJ(SPData, 3)
            elif event.key == pygame.K_f:
                pressed['f'] = False
                BreakJ(SPData, 3)
            elif event.key == pygame.K_t:
                pressed['t'] = False
                BreakJ(SPData, 4)
            elif event.key == pygame.K_g:
                pressed['g'] = False
                BreakJ(SPData, 4)
        elif event.type == pygame.QUIT:
            raise KeyboardInterrupt()
    return noInput, pressed, mSpeedSel

def PegasusJointControl(pegasus: Robot, SPData: SerialData, localMu: 
                        serial.Serial, dtComm: float, dtPrint: float, 
                        dtInput: float, encAlg: 
                        str = 'utf-8', holdStill: bool = False, kP: 
                        "np.ndarray[float]"=0, kI: "np.ndarray[float]"=0, 
                        kD: "np.ndarray[float]"=0, ILim=[100,100,100,100,100]):
    """Allows one to move each joint of the Pegasus arm independantly.
    :param SPData: SerialData object for communication.
    :param localMu: Serial object to connect to local microcontroller.
    :param dtComm: Interval between sending & receiving data from 
                   the local microcontroller in seconds.
    :param dtInput: Interval of checking for user inputs in seconds.
    :param dtPrint: Interval between printing robot data on the 
                    terminal in seconds.
    :param encAlg: Encoding algorithm for serial communication.
    :param holdStill: Indicates if position should be actively held if 
                      the arm is not moved.
    :param kP: nxn proportional matrix, typically an identity
                   matrix times a constant.
    :param kI: nxn integral matrix, typically an identity matrix
                times a constant.
    :param kD: nxn difference matrix, typically an identity matrix
                times a constant.
    NOTE: To omit P-, I-, or D action, input kX = 0
    :param ILim: Integral term limiter for anti-integral windup.
    """

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
    noInput = True
    pressed = dict() #To be filled with booleans for key presses
    try:
        while True: #Main loop
            lastCheck = SReadAndParse(SPData, lastCheck, dtComm, 
                                      localMu, encAlg)[0]
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
                #COMMENTED OUT FOR DEBUG
                #print(f"Speed: {SPData.mSpeed}, Direction: {SPData.rotDirDes}")
                #print(f"Count: {SPData.totCount}, Homing: {SPData.homing}")
                #print(f"Angle: {['%.2f' % elem for elem in SPData.currAngle]}")
                lastPrint = time.time()
            if (time.time() - lastInput >= dtInput):
                #Check for key-press, act accordingly
                noInputPrev = noInput
                noInput, pressed, mSpeedSel = CheckKeysJoint(SPData, 
                                   pressed, mSpeedMin, mSpeedMax, 
                                   mSpeedSel)
                if holdStill:
                    if noInput and not any(list(pressed.values())):
                        if noInput != noInputPrev: #Initialize PID
                            pos = np.array(SPData.currAngle[:-1]) #excl. gripper!
                            err = np.array([0 for i in range(len(pegasus.joints))])
                            termI = np.array([0 for i in 
                                              range(len(pegasus.joints))])
                        #Note: Gripper not included in HoldPos!
                        mSpeed, termI, err = HoldPos(pos, pegasus, 
                        SPData, kP, kI, kD, termI, ILim, dtComm, err)
                        for i in range(SPData.lenData-1): #Remap mSpeed
                            if mSpeed[i] > mSpeedMax:
                                mSpeed[i] = mSpeedMax
                            elif mSpeed[i] < mSpeedMin:
                                mSpeed[i] = 0 #Avoid damaging the motor
                                #I-action will bring the value up eventually.
                        SPData.mSpeed[:-1] = mSpeed
                lastInput = time.time()

    except KeyboardInterrupt:
        #Set motor speeds to zero & close serial.
        localMu.write(f"{['0|0'] * SPData.lenData}\n".encode(encAlg))
        time.sleep(dtComm)
        localMu.__del__()
        print("Quitting...")
        return
    print("This should never happen!")

def CheckKeysEF(SPData: SerialData, pressed: Dict[str, bool], vMin: float, vMax: float, vSel: float, 
                wMin: float, wMax: float, wSel: float, dVel: float= 0.02) \
                -> Tuple["np.ndarray[float]", float, float]:
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
    :param pressed: Dictionary to keep track of keyboard activity.
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
    :return pressed: Updated dictionary for tracking keyboard activity.
    :return noInput: Boolean indicating presence of new inputs.
    """
    V = [0 for i in range(6)]
    if SPData.limBool.any():
        limFactor = 0
    else:
        limFactor = 1
    events = pygame.event.get()
    if len(events) == 0:
        noInput = True
        return V, vSel, wSel, noInput
    else:
        noInput = False

    for event in events:
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                pressed['w'] = True
                V[3] = vSel * limFactor
            elif event.key == pygame.K_s:
                pressed['s'] = True
                V[3] = -vSel * limFactor
            elif event.key == pygame.K_a:
                pressed['a'] = True
                V[4] = vSel * limFactor
            elif event.key == pygame.K_d:
                pressed['d'] = True
                V[4] = -vSel * limFactor
            elif event.key == pygame.K_z:
                pressed['z'] = True
                V[5] = vSel * limFactor
            elif event.key == pygame.K_x:
                pressed['x'] = True
                V[5] = -vSel * limFactor
            elif event.key == pygame.K_q:
                pressed['q'] = True
                V[0] = wSel * limFactor
            elif event.key == pygame.K_e:
                pressed['e'] = True
                V[0] = -wSel * limFactor
            elif event.key == pygame.K_r:
                pressed['r'] = True
                V[1] = wSel * limFactor
            elif event.key == pygame.K_f:
                pressed['f'] = True
                V[1] = -wSel * limFactor
            elif event.key == pygame.K_c:
                pressed['c'] = True
                V[2] = wSel * limFactor
            elif event.key == pygame.K_v:
                pressed['v'] = True
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
            if event.key == pygame.K_w:
                pressed['w'] = False
                V[3] = 0
            elif event.key == pygame.K_s:
                pressed['s'] = False
                V[3] = 0
            elif event.key == pygame.K_a:
                pressed['a'] = False
                V[4] = 0
            elif event.key == pygame.K_d:
                pressed['a'] = False
                V[4] = 0
            elif event.key == pygame.K_z:
                pressed['z'] = False
                V[5] = 0
            elif event.key == pygame.K_x:
                pressed['x'] = False
                V[5] = 0
            elif event.key == pygame.K_q: 
                pressed['q'] = False
                V[0] = 0
            elif event.key == pygame.K_e:
                pressed['e'] = False
                V[0] = 0
            elif event.key == pygame.K_r:
                pressed['r'] = False
                V[1] = 0
            elif event.key == pygame.K_f:
                pressed['f'] = False
                V[1] = 0
            elif event.key == pygame.K_c:
                pressed['c'] = False 
                V[2] = 0
            elif event.key == pygame.K_v:
                pressed['v'] = False
                V[2] = 0
        elif event.type == pygame.QUIT:
            raise KeyboardInterrupt()
        return V, vSel, wSel, pressed, noInput

def DThetaToComm(SPData: SerialData, dtheta: "np.ndarray[float]") \
                -> "np.ndarray[float]":
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
                             range(len(SPData.joints))])
    dthetaComm = np.multiply(dtheta, gearRatioArr)
    return dthetaComm

def PegasusEFControl(SPData: SerialData, pegasus: Robot, 
                     localMu: serial.Serial, dtComm: float, dtPrint: float, 
                     dtInput: float, encAlg: str = 'utf-8', 
                     holdStill: bool = False, kP: "np.ndarray[float]"=0, 
                     kI: "np.ndarray[float]"=0, kD: "np.ndarray[float]"=0, 
                     ILim=[100,100,100,100,100]):
    """Allows one to control the Pegasus robot arm in end-effector space.
    :param :"""
    #Absolute maximum and minimum linear- & rotational speeds in SI.
    dthetaMax = np.array([1, 1, 1, 1, 1, 1]) #TODO: CHECK!!!!
    mSpeedMin = 50 #TODO: CHECK!!!
    mSpeedMax = 200 #TODO: CHECK!!!
    vMaxLim = 0.15
    vMinLim = 0
    wMaxLim = 0.5
    wMinLim = 0
    #Set speeds from user input:
    try:
        vMax = float(input(f"vMax (m/s), make sure < {vMaxLim} m/s: "))
    except ValueError as e:
        if "invalid literal for float()" in e.message:
            print(f"Invalid input, setting vMax to {round(2*vMaxLim/3,2)} m/s")
        else:
            print(f"Unkown error, setting vMax to {round(2*vMaxLim/3,2)} m/s")
        vMax = 2*vMaxLim/3
    if vMax > vMaxLim:
        print(f"Invalid entry: {vMax} > {vMaxLim}. Setting vMax = {vMaxLim}")
        vMax = vMaxLim
    try:
        vMin = float(input("vMin (m/s), make sure > 0 m/s: "))
    except ValueError as e:
        if "invalid literal for float()" in e.message:
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
        wMax = float(input(f"wMax (rad/s), make sure < {wMaxLim} rad/s: "))
    except ValueError as e:
        if "invalid literal for float()" in e.message:
            print(f"Invalid input, setting wMax to {round(2*wMaxLim/3,2)}" +\
                   "rad/s")
        else:
            print(f"Unkown error, setting wMax to {round(2*wMaxLim/3,2)}" + \
                   "rad/s")
        wMax = 2*wMaxLim/3
    if wMax > wMaxLim:
       print(f"Invalid entry: {wMax} > {wMaxLim}, Setting wMax = " +\
             f"{round(2*wMaxLim/3,2)}") 
    try:
        wMin = float(input("wMin (rad/s), make sure > 0 rad/s: "))
    except ValueError as e:
        if "invalid literal for float()" in e.message:
            print("Invalid input, setting wMin to 0 rad/s")
        else:
            print("Unkown error, setting wMin to 0 rad/s")
        wMin = 0
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
    pygame.init()
    screen = pygame.display.set_mode([500, 500])
    lastCheck = time.time()
    lastWrite = time.time()
    lastPrint = time.time()
    lastInput = time.time()
    screwAxes = pegasus.joints[0].screwAx
    for i in range(1, len(pegasus.joints)):
        screwAxes = np.c_[screwAxes, pegasus.joints[i].screwAx]
    noInput = True
    pressed = Dict()
    try:
        while True: #Main loop
                lastCheck = SReadAndParse(SPData, lastCheck, dtComm, 
                                        localMu, encAlg)[0]
                if (time.time() - lastWrite >= dtComm):
                    #Check for each motor if the current move is allowed 
                    #within the joint limits
                    SPData.CheckJointLim() #TODO: Check if dir is correct!
                    for i in range(SPData.lenData-1):
                        SPData.dataOut[i] = f"{SPData.mSpeed[i]}|" + \
                                            f"{SPData.rotDirDes[i]}"
                    SPData.dataOut[-1] = f"{0|0}" #Gripper, TODO: Replace w/ gripper commands
                    localMu.write(f"{SPData.dataOut}\n".encode(encAlg))
                    lastWrite = time.time()
                if (time.time() - lastPrint >= dtPrint):
                    print(f"Speed: {SPData.mSpeed}, Direction: {SPData.rotDirDes}")
                    print(f"Count: {SPData.totCount}, Homing: {SPData.homing}")
                    print(f"Angle: {SPData.currAngle}")
                    lastPrint = time.time()
                if (time.time() - lastInput >= dtInput):
                    #Check for key-press, act accordingly
                    vels = [vMin, vMax, vSel, wMin, wMax, wSel]
                    noInputPrev = noInput
                    V, vSel, wSel, pressed, noInput = CheckKeysEF(SPData, 
                                                                  pressed, 
                                                                  *vels)
                    print(f"Key input:\n{V}\n{vSel}\n{wSel}\n{noInput}")
                    if holdStill:
                        if noInput:
                            #Note: Gripper not included in HoldPos!
                            if noInput != noInputPrev: #Initialize PID
                                pos = SPData.currAngle
                                err = [0 for i in range(len(pegasus.joints))]
                                termI = [0 for i in range(len(pegasus.joints))]
                            mSpeed, termI, err = HoldPos(pos, pegasus, 
                            SPData, kP, kI, kD, termI, ILim, dtComm, err)
                            for i in range(SPData.lenData): #Remap mSpeed
                                if mSpeed[i] > mSpeedMax:
                                    mSpeed[i] = mSpeedMax
                                elif mSpeed[i] < mSpeedMin:
                                    mSpeed[i] = 0 #Avoid damaging the motor
                                    #I-action will bring the value up eventually.
                            SPData.mSpeed = mSpeed
                    if not noInput or not holdStill:
                        JSpace = mr.JacobianSpace(screwAxes, SPData.currAngle[:-1])
                        dtheta = np.dot(np.linalg.pinv(JSpace), V)
                        #Translate joint speeds to motor speeds & directions
                        #Virtual joints -> real joints
                        dthetaComm = DThetaToComm(SPData, dtheta)
                        #Sign -> direction bool
                        SPData.rotDirDes = [np.sign(dthetaComm[i]) if 
                                            np.sign(dthetaComm[i]) == 1 else 0 
                                            for i in range(dthetaComm.size)]
                        #angular velocity to PWM
                        SPData.Dtheta2Mspeed(dthetaComm, dthetaMax, 
                                             mSpeedMin, mSpeedMax)


    except KeyboardInterrupt:
        #Set motor speeds to zero & close serial.
        localMu.write(f"{['0|0'] * SPData.lenData}\n".encode(encAlg))
        time.sleep(dtComm)
        localMu.__del__()
        print("Quitting...")
    
def PegasusManualControl(method="joints"):
    ### INTANTIATE ROBOT INSTANCE ###
    #Inertia matrices
    iMat0 = np.diag([0.03947, 0.03362, 0.04886])
    iMat1 = np.diag([0.00393, 0.00237, 0.00172])
    iMat2 = np.diag([0.00294, 0.00210, 0.0029])
    iMat34 = np.diag([0.00041, 0.00348, 0.00364])
    massList = [5.13, 0.507, 0.420, 0.952, 0.952]
    #Transformation matrices from CoM of links with principle axes of
    #inertia to the space frame (Tsi):
    Tsi0 = np.array([[ 0.397, 0.838,-0.375, 0.0284],
                    [-0.909, 0.416,-0.033,-0.0413],
                    [ 0.129, 0.354, 0.926, 0.0522],
                    [0    , 0     , 0    , 1     ]])
    Tsi1 = np.array([[ 0.000, 0.455, 0.890, 0.0015],
                    [ 0.001, 0.890,-0.455, 0.0026],
                    [-1.000, 0.007, 0.001, 0.0039],
                    [0    , 0     , 0    , 1     ]])
    Tsi2 = np.array([[-0.003, 0.082, 0.997, 0.0009],
                    [ 0.001, 0.997,-0.082, 0.0021],
                    [-1.000, 0.001,-0.003, 0.0029],
                    [0    , 0     , 0    , 1     ]])
    Tsi34 = np.array([[-0.999, 0.000, -0.035, 0.0076],
                    [0.000, -1.000, -0.000,-0.0159],
                    [-0.035, -0.000, 0.999, 0.5840],
                    [0    , 0     , 0    , 1     ]])
    TsbHome = np.array([[1,0,0, 0.1474],
                        [0,1,0,-0.0168],
                        [0,0,1, 0.5853],
                        [0,0,0, 1     ]])
    #Screw axes in the Space Frame {s}
    S0 = np.array([0,0,1,0,0,0])
    S1 = np.array([0,1,0,-0.125,0,0.0035])
    S2 = np.array([0,1,0,-0.355,0,0.0035])
    S3 = np.array([0,1,0,-0.585,0,0.0030])
    S4 = np.array([1,0,0,0,0.585,0.016])
    lims0 = [-0.945*np.pi, 0.945*np.pi] #+/- 170 deg
    lims1 = [-0.25*np.pi, 0.5*np.pi] #-45 deg, + 90 deg
    lims2 = [-np.pi, 0.25*np.pi] #-180 deg, + 45 deg.
    lims3 = [-np.pi, 0.25*np.pi] #-180 deg, +45 deg.
    lims4 = [-np.pi, np.pi] #+/- 180 deg.
    gearRatioList = [19.7*50, 19.7*50, (65.5*20)/9, (65.5*20)/9, (127.7*32)/9]
    cpr = 512
    L0 = Link(iMat0, massList[0], None, Tsi0)
    L1 = Link(iMat1, massList[1], L0, Tsi1)
    L2 = Link(iMat2, massList[2], L1, Tsi2)
    L34 = Link(iMat34, massList[3], L2, Tsi34)
    links = [L0, L1, L2, L34, L34]
    km = [22.7*10**(-3), 22.7*10**(-3), 22.7*10**(-3), 22.7*10**(-3),
            22.7*10**(-3), 22.7*10**(-3), 9.2*10**(-3)] 
    J0 = Joint(S0, [None, L0], gearRatioList[0], km[0], cpr, lims0)
    J1 = Joint(S1, [L0, L1], gearRatioList[1], km[1], cpr, lims1)
    J2 = Joint(S2, [L1, L2], gearRatioList[2], km[2], cpr, lims2)
    J3 = Joint(S3, [L2,L34], gearRatioList[3], km[3], cpr, lims3)
    J4 = Joint(S4, [L2,L34], gearRatioList[4], km[4], cpr, lims4)
    joints = [J0, J1, J2, J3, J4]
    Pegasus = Robot(joints, links, TsbHome)
    ### END OF ROBOT INITIATION ###

    ### SETUP SERIAL COMMUNICATION ###
    baudRate = 115200
    lenData = len(Pegasus.joints)+1 #+1 for gripper
    desAngles = [0 for i in range(lenData)]
    maxDeltaAngles = [np.pi for i in range(lenData)]
    tolAngles = [0.001*np.pi for i in range(lenData)]
    SPData = SerialData(lenData, desAngles, maxDeltaAngles, tolAngles, 
                        Pegasus.joints)
    dtComm = 0.005 #Make sure this aligns with dtComm in C++ code.
    dtPrint = 1
    dtInput = 0.03 #approximately 30 fps, also avoids window crashing
    port, warning = FindSerial(askInput=True)
    localMu = StartComms(port, baudRate)
    encAlg = "utf-8"
    ### END OF SERIAL COMMUNICATION SETUP ###
    homingPins = [7,11,13,15,29,31]
    holdStill = False
    kP = float(input("kP: "))*np.eye(len(Pegasus.joints))
    kI = float(input("kI: "))*np.eye(len(Pegasus.joints))
    kD = float(input("kD: "))*np.eye(len(Pegasus.joints))
    #10 seconds of unit action:
    ILim = [kI[0,0]*10/dtInput for i in range(len(Pegasus.joints))] 
    if method == 'joint':
        PegasusJointControl(Pegasus, SPData, localMu, dtComm, dtPrint,
                            dtInput, encAlg, holdStill, kP, kI, kD, 
                            ILim) 
    elif method == 'end-effector':
        PegasusEFControl(SPData, Pegasus, localMu, dtComm, dtPrint, dtInput, 
                         encAlg, holdStill, kP, kI, kD)
    
    return

if __name__ == "__main__":
    method = input("Please specify the desired type of control:\n" +
                   "For joint control, type 'joint'. \n" +  
                   "For end-effector control, type 'end-effector'\n")
    methodBool = False
    while not methodBool:
        if method == 'joint' or method == 'end-effector':
            methodBool = True
        else:
            method = input("invalid control method, please choose " + 
                           "between 'joint' and 'end-effector'.")
    PegasusManualControl(method)

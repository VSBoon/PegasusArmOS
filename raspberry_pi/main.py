import os
import sys

#Find directory path of current file
current = os.path.dirname(os.path.realpath(__file__))
#Find directory path of parent folder and add to sys path
parent = os.path.dirname(current)
sys.path.append(parent)
print("\n\n--- Welcome to the PegasusArm OS v1.0.0 User Interface ---\n\n")
print("Importing modules...\n")
import numpy as np
import modern_robotics as mr
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import pygame
import time
import csv
from typing import List, Tuple, Dict
from robot_init import robot as R1 
from robot_init import robotFric as R2
from settings import sett
from classes import SerialData, Robot, InputError, PID
from util import Tau2Curr, Curr2MSpeed
from kinematics.kinematic_funcs import FKSpace
from serial_comm.serial_comm import FindSerial, StartComms, GetComms, SReadAndParse
from dynamics.dynamics_funcs import FeedForward
from control.control import PosControl, VelControl, ForceControl, ImpControl

def GetEConfig(sConfig: np.ndarray, Pegasus: Robot) -> np.ndarray:
    """Obtain a desired end-effector configuration based on the input 
    of the user.
    :param sConfig: Start configuration in joint space.
    :param Pegasus: A mathematical model of the robot.
    :return sConfig: Start configuration in joint- or end-effector
                     space, based on eConfig input.
    :return eConfig: Desired end configuration in joint- or end-
                     effector space."""
    userInput = input("Please enter the desired end-configuration, "+
                            "either as a list of joint angles in pi radians " +
                            "or a 4x4 transformation matrix:\n").strip()
    #Example eConfig: [0.5,0.2,0,0,0] OR 
    #[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
    if "[[" in userInput[0:2]:
        eConfig = np.zeros((4,4))
        rows = userInput[2:-2].split('],[')
        for i in range(4):
            row = np.array([float(item) for item in rows[i].split(',')])
            eConfig[i,:] = row
        sConfig = FKSpace(Pegasus.TsbHome, Pegasus.screwAxes, sConfig)
        if eConfig.shape != sConfig.shape:
            raise InputError("Start- and end configuration are not the same "+
                             "shape.")
        return sConfig, eConfig
    elif userInput[0] == "[":
        userInput = userInput[1:-1]
        list = [float(item)*np.pi for item in userInput.split(',')]
        eConfig = np.array(list)
        if eConfig.shape != sConfig.shape:
            raise InputError("Start- and end configuration are not the same "+
                             "shape.")
        return sConfig, eConfig

def GetKeysJoint(keyDownPrev: List[bool], events: List["pygame.Event"], 
                 wSel: float, wDesPrev: np.ndarray, wMin: float, wMax: float,
                 wIncr: float) -> Tuple[List[bool], bool, float]:
    """Checks for key-presses and executes joint control commands 
    accordingly.
    All mSpeed values should be an integer in the range [0, 255].
    :param keyDownPrev: List of booleans from pygame.key.get_pressed().
    :param events: List of pygame events for key presses.
    :param wSel: The current selected motor speed.
    :param wDesPrev: Previous joint velocities from GetKeysJoint().
    :param wMin: The minimal motor speed.
    :param wMax: The maximal motor speed.
    :param wIncr: Incrementation value of motor speed.
    :return keyDown: List of booleans from pygame.key.get_pressed().
    :return noInput: Bool indicating no new keyboard events.
    :return wSel: Selected motor speed
    :return wDes: Array of desired joint velocities based on input.
    """
    keyDown = pygame.key.get_pressed()
    wDes = wDesPrev
    if keyDown != keyDownPrev:
        noInput = False
    else:
        noInput = True
        return keyDownPrev, noInput, wSel, wDes
    if keyDown[pygame.K_q]:
        wDes[0] = wSel
    elif keyDown[pygame.K_a]:
        wDes[0] = -wSel
    else:
        wDes[0] = 0
    if keyDown[pygame.K_w]:
        wDes[1] = wSel
    elif keyDown[pygame.K_s]:
        wDes[1] = -wSel
    else:
        wDes[1] = 0
    if keyDown[pygame.K_e]:
        wDes[2] = wSel
    elif keyDown[pygame.K_d]:
        wDes[2] = -wSel
    else:
        wDes[2] = 0
    if keyDown[pygame.K_r]:
        wDes[3] = wSel
    elif keyDown[pygame.K_f]:
        wDes[3] = -wSel
    else:
        wDes[3] = 0
    if keyDown[pygame.K_t]:
        wDes[4] = wSel
    elif keyDown[pygame.K_g]:
        wDes[4] = -wSel
    else:
        wDes[4] = 0

    for event in events:
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_c:
                wSel += wIncr
                if wSel >= wMax:
                    wSel = wMax
                elif wSel <= wMin:
                    wSel = wMin
                print(f"motor speed: {wSel}")
            elif event.key == pygame.K_x:
                wSel -= wIncr
                if wSel >= wMax:
                    wSel = wMax
                elif wSel <= wMin:
                    wSel = wMin
                print(f"motor speed: {wSel}")
    return keyDown, noInput, wSel, wDes

def GetKeysEF(VPrev: np.ndarray, events: List["pygame.Event"], keyDownPrev: \
              List[bool], vSel: float, wSel: float, wMin: float, wMax: float, 
              vMin: float, vMax: float, efIncrL: float, efIncrR: float) -> \
              Tuple[List[bool], bool, float, np.ndarray]:
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
    :param VPrev: Previous twist from GetKeysEF().
    :param events: List of pygame events for key presses.
    :param keyDownPrev: List of booleans obtained from the previous
                        pygame.key.get_pressed().
    :param vSel: Previously selected linear velocity in [m/s].
    :param wSel: Previously selected angular velocity in [rad/s].
    :param wMin: Minimum angular end-effector velocity in [rad/s].
    :param wMax: Maximum angular end-effector velocity in [rad/s].
    :param vMin: Minimum linear end-effector velocity in [m/s].
    :param vMax: Maximum linear end-effector velocity in [m/s].
    :param efIncrL: Linear velocity increment, in [m/s].
    :param efIncrR: Rotational velocity increment, in [rad/s].
    :param keyDown: List of booleans obtained from 
                    pygame.key.get_pressed().
    :return noInput: Boolean indicating presence of new inputs.
    :return wSel: Newly selected angular velocity in [rad/s].
    :return vSel: Newly selected linear velocity in [m/s].
    :return V: 6x1 velocity twist. 
    """
    V = VPrev
    keyDown = pygame.key.get_pressed()
    if keyDown != keyDownPrev:
        noInput = False
    else:
        noInput = True
        return keyDown, noInput, wSel, vSel, V
    if keyDown[pygame.K_w]:
        V[3] = vSel
    elif keyDown[pygame.K_s]:
        V[3] = -vSel
    else:
        V[3] = 0
    if keyDown[pygame.K_a]:
        V[4] = vSel
    elif keyDown[pygame.K_d]:
        V[4] = -vSel
    else:
        V[4] = 0
    if keyDown[pygame.K_z]:
        V[5] = vSel
    elif keyDown[pygame.K_x]:
        V[5] = -vSel
    else:
        V[5] = 0
    if keyDown[pygame.K_q]:
        V[0] = vSel
    elif keyDown[pygame.K_e]:
        V[0] = -vSel
    else:
        V[0] = 0
    if keyDown[pygame.K_r]:
        V[1] = vSel
    elif keyDown[pygame.K_f]:
        V[1] = -vSel
    else:
        V[1] = 0
    if keyDown[pygame.K_c]:
        V[2] = vSel
    elif keyDown[pygame.K_v]:
        V[2] = -vSel
    else:
        V[2] = 0
    
    for event in events:
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_t:
                if (vSel + efIncrL) < vMax:
                    vSel += efIncrL
                else:
                    vSel = vMax
                print(f"linear velocity: {vSel} m/s")
            elif event.key == pygame.K_y:
                if (vSel - efIncrL) > vMin:
                    vSel -= efIncrL
                else:
                    vSel = efIncrL
                print(f"linear velocity: {vSel} m/s")
            elif event.key == pygame.K_g:
                if (wSel + efIncrR) < wMax:
                    wSel += efIncrR
                else:
                    wSel = wMax
                print(f"angular velocity: {wSel} rad/s")
            elif event.key == pygame.K_h:
                if (wSel - efIncrR) > wMin:
                    wSel -= efIncrR
                else:
                    wSel = wMin
                print(f"angular velocity: {wSel} rad/s")
    return keyDown, noInput, wSel, vSel, V

def HoldPos(serial: SerialData, robot: Robot, PIDObj: PID, 
            thetaDes: np.ndarray, dtHold: float):
    thetaCurr = np.array(serial.currAngle[:-1]) #Minus gripper
    thetaPrev = np.array(serial.prevAngle[:-1])
    dThetaPrev = (thetaCurr - thetaPrev)/dtHold
    dThetaDes = np.array([0 for angle in thetaDes])
    ddThetaDes = np.array([0 for angle in thetaDes])
    FTip = np.array([0 for i in range(6)])
    g = np.array([0,0,-9.81])
    tauFF = FeedForward(robot, thetaDes, dThetaDes, dThetaPrev, ddThetaDes, g, FTip)
    tauPID = PIDObj.Execute(thetaDes, thetaCurr, dtHold)
    tauComm = tauFF + tauPID
    #Diff-drive properties:
    tauJ4 = tauComm[3]
    tauJ5 = tauComm[4]
    tauComm[3] = -tauJ4 - tauJ5
    tauComm[4] = tauJ4 - tauJ5 
    I = [Tau2Curr(tauComm[i], robot.joints[i].gearRatio, 
                  robot.joints[i].km, 2) for i in range(len(robot.joints))]
    PWM = [round(Curr2MSpeed(current)) for current in I]
    return PWM

robotSelected = False
while not robotSelected:
    try:
        robotType = input("Please select the robot model type.\n" +\
            "For a model with friction, enter 0.\nFor a robot " +\
            "without friction, enter 1.\n")
        if robotType == "0":
            Pegasus = R2
            robotSelected = True
        if robotType == "1":
            Pegasus = R1
            robotSelected = True
        else:
            print("Invalid entry. Enter either '0' or '1'-0")
            raise InputError() 
    except InputError:
        continue
print("\nRobot type selected. Setting up serial communication...\n")
serial = SerialData(6, Pegasus.joints)
port = FindSerial(askInput=True)[0]
Teensy = StartComms(port)

method = False
frameInterval = sett['dtFrame']
lastFrame = time.perf_counter()
dtPID = sett['dtPID']
lastPID = time.perf_counter()
dtComm = sett['dtComm']
lastComm = time.perf_counter()
lastCheck = time.perf_counter()
dtFrame = sett['dtFrame']
lastHold = time.perf_counter()
dtHold = sett['dtHold']

PIDObj = sett['PID']
errThetaMax = sett['errThetaHold']
vMax = sett['vMax']
wMax = sett['wMax']
jIncr = sett['jIncr']
efIncrL = sett['eIncrLin']
efIncrR = sett['eIncrRot']
dtPosConf = sett['dtPosConfig']
forceDamp = sett['forceDamp']
M = sett['M']
B = sett['B']
Kx = sett['Kx']
Ka = sett['Ka']

#initialize empty objects
wDesJ = np.zeros(5)
vDesE = np.zeros(6)
vPrevJ = np.zeros(5)
wSelJ = jIncr
wSelE = efIncrR
vSelE = efIncrL
noInput = True
noInputPrev = False
VPrev= np.zeros(6)
dthetaPrev = np.zeros(5)

methodSelected = False
while not methodSelected:
    try:
        method = input("Please enter a control method.\nFor position " +\
                    "control, type 'pos'.\nFor velocity control, " +\
                    "type 'vel'.\nFor force control, type 'force'.\n"+\
                    "For impedance control, type 'imp'.\n")
        if method != 'pos' and method != 'vel' and \
        method != 'force' and method != 'imp':
            print("Invalid method. Try again.")
            raise InputError()
        else:
            methodSelected = True
    except InputError:
        continue

if method == 'vel':
    spaceSelected = False
    while not spaceSelected:
        try:
            space = input("To perform joint velocity control, type "+
                            "'joint'.\nFor end-effector velocity control, "+
                            "type 'end-effector'.\n")
            if space != 'joint' and space != 'end-effector':
                raise InputError()
            else:
                spaceSelected = True
        except InputError:
            continue
elif method == 'force':
    pathSelected = False
    while not pathSelected:
        path = input("Please input the path to the CSV file with " +
                     "the desired end-effector wrenches over time\n")
        path = os.path.join(current, path)
        try:
            with open(path) as csvFile:
                csvRead = csv.reader(csvFile, delimiter=';')
                wrenchesList = []
                for wrenchCSV in csvRead:
                    wrenchRow = [float(val.replace(',','.')) for val in wrenchCSV]
                    wrenchesList.append(np.array(wrenchRow))
            pathSelected = True
        except FileNotFoundError:
            print("Incorrect path.")
            continue
    dtKnown = False
    while not dtKnown:
        try:
            dtWrench = float(input("Time between wrenches in the CSV file in [s]: "))
            dtKnown = True
        except ValueError:
            print("Invalid input. Please input a time in [s].")
elif method == 'imp':
    #Get robot into desired position.
    sConfig = np.array(serial.currAngle[:-1])
    eConfig = GetEConfig(sConfig, Pegasus)[1]
    if eConfig.shape != (4,4):
        TDes = FKSpace(Pegasus.TsbHome, Pegasus.screwAxes, eConfig)
    else:
        TDes = eConfig

print("\nSetting up UI...\n")
pygame.init()
screen = pygame.display.set_mode([700, 500])
background = pygame.image.load(os.path.join(current,'control_overview.png'))
if method == 'vel':
    keyDownPrev = pygame.key.get_pressed()
while True: #Main loop!
    try:
        if method == 'pos': #Position control
            sConfig = np.array(serial.currAngle[:-1])
            sConfig, eConfig = GetEConfig(sConfig, Pegasus)
            try:
                PosControl(sConfig, eConfig, Pegasus, serial, dtPosConf, 
                           vMax, wMax, PIDObj, dtComm, dtPID, dtFrame, Teensy, screen, background)
            except SyntaxError as e:
                print(e.msg)
                continue
            except ValueError as e:
                print(e)
                continue
            #Initiate hold-pos
            thetaDes = eConfig #exclude gripper
            #Initialize with high value
            errThetaCurr = np.array([100*np.pi for i in serial.currAngle[:-1]])
            print("Stabilizing around new position...")
            while all(np.greater(errThetaCurr, errThetaMax)):
                if time.perf_counter() - lastHold >= dtHold: 
            #While not stabilized within error bounds, do holdpos
                    serial.mSpeed[:-1] = HoldPos(serial, Pegasus, PIDObj, thetaDes, 
                                            dtHold)
                    lastHold = time.perf_counter()

                lastCheck = SReadAndParse(serial, lastCheck, dtComm, Teensy)[0]
                if (time.perf_counter() - lastComm >= dtComm):
                    errThetaCurr = thetaDes - np.array(serial.currAngle[:-1])
                    serial.rotDirDes = [1 if np.sign(speed) == 1 else 0 for 
                                        speed in serial.mSpeed]
                    for i in range(serial.lenData-1): 
                        serial.dataOut[i] = f"{serial.mSpeed[i]}|"+\
                                            f"{serial.rotDirDes[i]}"
                    serial.dataOut[-1] = f"{0|0}"
                    Teensy.write(f"{serial.dataOut}\n".encode('utf-8')) 
                    lastComm = time.perf_counter()
            print("Stabilization complete.")
            PIDObj.Reset()

        elif method == 'vel': #Velocity Control
            if space == 'joint':
                if time.perf_counter() - lastPID > dtPID:
                    if noInput and not any(keyDownPrev) and not np.any(wDesJ):
                        if noInput != noInputPrev:
                            #Initiate PID
                            PIDObj.Reset()
                            thetaDes = np.array(serial.currAngle[:-1])
                        serial.mSpeed[:-1] = HoldPos(serial, Pegasus, PIDObj, 
                                                     thetaDes, dtPID)
                    else:
                    #VelControl implicitely updates serial.mSpeed (FF+PID).
                        vPrevJ = VelControl(Pegasus, serial, wDesJ, vPrevJ, 
                                            dtPID, 'joint', dtComm, PIDObj)
                    lastPID = time.perf_counter()

                if time.perf_counter() - lastFrame >= dtFrame:
                    events = pygame.event.get()
                    for event in events:
                        if event.type == pygame.QUIT:
                            raise KeyboardInterrupt
                    noInputPrev = noInput
                    wDesPrev = wDesJ
                    keyDownPrev, noInput, wSelJ, wDesJ = GetKeysJoint(keyDownPrev, events, wSelJ, wDesPrev, 0, wMax, jIncr)
                    screen.blit(background, (0,0))
                    lastFrame = time.perf_counter()
                    
            elif space == 'end-effector':
                if time.perf_counter() - lastPID > dtPID:
                    #VelControl implicitely updates serial.mSpeed.
                    if noInput and not any(keyDownPrev):
                        if noInput != noInputPrev:
                            #Initiate PID
                            PIDObj.Reset()
                            thetaDes = np.array(serial.currAngle[:-1])
                            thetacurr = np.array(serial.currAngle[:-1])
                        serial.mSpeed[:-1] = HoldPos(serial, Pegasus, PIDObj, 
                                                     thetaDes, dtPID)
                    else:
                        #FF & PID!
                        vPrevJ = VelControl(Pegasus, serial, vDesE, vPrevJ, 
                                            dtPID, 'twist', dtComm, PIDObj)
                    lastPID = time.perf_counter()

                if time.perf_counter() - lastFrame >= dtFrame:
                    noInputPrev = noInput
                    events = pygame.event.get()
                    for event in events:
                        if event.type == pygame.QUIT:
                            raise KeyboardInterrupt
                    VPrev = vDesE
                    keyDownPrev, noInput, wSel, vSel, vDesE = \
                    GetKeysEF(VPrev, events, keyDownPrev, vSelE, wSelE, 0, 
                              wMax, 0, vMax, efIncrL, efIncrR)
                    pygame.display.update()
                    screen.blit(background, (0,0))
                    lastFrame = time.perf_counter()

            lastCheck = SReadAndParse(serial, lastCheck, dtComm, Teensy)[0]
            if (time.perf_counter() - lastComm >= dtComm):
                serial.rotDirDes = [1 if np.sign(speed) == 1 else 0 for 
                                    speed in serial.mSpeed]
                for i in range(serial.lenData-1): #TODO: Add Gripper function
                    serial.dataOut[i] = f"{abs(serial.mSpeed[i])}|"+\
                                        f"{serial.rotDirDes[i]}"
                #TODO: Replace last entry w/ gripper commands
                serial.dataOut[-1] = f"{0|0}"
                print(serial.dataOut)
                Teensy.write(f"{serial.dataOut}\n".encode('utf-8')) 
                lastComm = time.perf_counter()
        
        elif method == 'force': #Force control
            n = -1 #iterator
            startTime = time.perf_counter()
            print(f"Expected total time: {dtWrench*len(wrenchesList)} s.")
            while time.perf_counter() - startTime <= dtWrench*len(wrenchesList):
                nPrev = n
                n = round((time.perf_counter()-startTime)/dtWrench)
                if n >= len(wrenchesList):
                    print("finished!")
                    #Initiate hold-pos
                    thetaDes = np.array(serial.currAngle[:-1])
                    errThetaCurr = np.array([100*np.pi for i in serial.currAngle[:-1]])
                    print("Stabilizing around new position...")
                    while all(np.greater(errThetaCurr, errThetaMax)):
                        if time.perf_counter() - lastHold >= dtHold: 
                    #While not stabilized within error bounds, do holdpos
                            serial.mSpeed[:-1] = HoldPos(serial, Pegasus, PIDObj, thetaDes, 
                                                    dtHold)
                            lastHold = time.perf_counter()
                    
                        lastCheck = SReadAndParse(serial, lastCheck, dtComm, Teensy)[0]
                        if (time.perf_counter() - lastComm >= dtComm):
                            errThetaCurr = thetaDes - np.array(serial.currAngle[:-1])
                            serial.rotDirDes = [1 if np.sign(speed) == 1 else 0 for 
                                                speed in serial.mSpeed]
                            for i in range(serial.lenData-1): 
                                serial.dataOut[i] = f"{abs(serial.mSpeed[i])}|"+\
                                                    f"{serial.rotDirDes[i]}"
                            serial.dataOut[-1] = f"{0|0}"
                            Teensy.write(f"{serial.dataOut}\n".encode('utf-8')) 
                            lastComm = time.perf_counter()
                    print("Stabilization complete.")
                    PIDObj.Reset()
                    raise KeyboardInterrupt
                if n != nPrev:
                    ForceControl(Pegasus, serial, wrenchesList[n], forceDamp, dtWrench)
                
                if time.perf_counter() - lastFrame >= dtFrame:
                            events = pygame.event.get() #avoids freezing.
                            for event in events:
                                if event.type == pygame.QUIT:
                                    raise KeyboardInterrupt
                            screen.blit(background, (0,0))
                            pygame.display.update()
                            lastFrame = time.perf_counter()

                lastCheck = SReadAndParse(serial, lastCheck, dtComm, Teensy)[0]
                if (time.perf_counter() - lastComm >= dtComm):
                    serial.rotDirDes = [1 if np.sign(speed) == 1 else 0 for 
                                        speed in serial.mSpeed]
                    for i in range(serial.lenData-1): #TODO: Add Gripper function
                        serial.dataOut[i] = f"{abs(serial.mSpeed[i])}|"+\
                                            f"{serial.rotDirDes[i]}"
                        #TODO: Replace last entry w/ gripper commands
                        serial.dataOut[-1] = f"{0|0}"
                        Teensy.write(f"{serial.dataOut}\n".encode('utf-8')) 
                        lastComm = time.perf_counter()
        
        elif method == 'imp': #Impedance control
            if time.perf_counter() - lastPID > dtPID:
                VPrev, dthetaPrev = ImpControl(Pegasus, serial, TDes, VPrev, dthetaPrev, dtPID, M, B, Kx, Ka, PIDObj)
            if time.perf_counter() - lastFrame >= dtFrame:
                events = pygame.event.get() #avoids freezing.
                for event in events:
                    if event.type == pygame.QUIT:
                        raise KeyboardInterrupt
                screen.blit(background, (0,0))
                pygame.display.update()
                lastFrame = time.perf_counter()

            lastCheck = SReadAndParse(serial, lastCheck, dtComm, Teensy)[0]
            if (time.perf_counter() - lastComm >= dtComm):
                serial.rotDirDes = [1 if np.sign(speed) == 1 else 0 for 
                                    speed in serial.mSpeed]
                for i in range(serial.lenData-1): #TODO: Add Gripper function
                    serial.dataOut[i] = f"{abs(serial.mSpeed[i])}|"+\
                                        f"{serial.rotDirDes[i]}"
                #TODO: Replace last entry w/ gripper commands
                serial.dataOut[-1] = f"{0|0}"
                Teensy.write(f"{serial.dataOut}\n".encode('utf-8')) 
                lastComm = time.perf_counter()
    except KeyboardInterrupt:
        print("Ctrl+C pressed, Quitting...") 
        #Set motor speeds to zero & close serial.
        Teensy.write(f"{['0|0'] * serial.lenData}\n".encode("utf-8"))
        time.sleep(dtComm)
        Teensy.__del__()
        break


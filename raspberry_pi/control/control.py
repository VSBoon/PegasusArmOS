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
    """Position control by means of point-to-point trajectory 
    generation combined with feed-forward and PID torque control.
    :param sConfig: Start configuration, either in SE(3) or a list of 
                    joint angles.
    :param eConfig: End configuration, either in SE(3) or a list of 
                    joint angles.
    :param robot: A Robot object describing the robot mathematically.
    :param serial: SerialData object for data transmission and 
                   -storage.
    :param dt: Time between subconfigurations of the trajectory in [s].
    :param vMax: Maximum linear velocity of the end-effector.
    :param omgMax: Maximum rotational velocity of the joints.
    :param kP: Proportional constant for PID control.
    :param kI: Integral constant for PID control.
    :param kD: Derivative constant for PID control.
    :param dtComm: Interval of sending & receiving data w.r.t the 
                   local microcontroller in [s].
    :param dtPID: Time between PID torque updates in [s].
    
    Example input:
    Initialisation of Robot args is omitted for the sake of brevity.
    robot = Robot(joints, links, TsbHome)
    serial = SerialData(6, [0,0,0,0,0,0], [1,1,1,1,1,1], [0.1,0.1,0.1,0.1,0.1,0.1], robot.joints)
    sConfig = serial.currAngle[:-1]
    eConfig = [0.2,0.2,0.2,0.2,0.2]
    dt = 0.1
    vMax = 0.25
    omgMax = 0.25
    kP = 1
    kI = 0.01
    kD = 0.01
    dtComm = 0.5
    dtPID = 0.02
    """
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
    
def Grip(gripping: bool, serial: SerialData):
    """Function to control the gripper.
    :param gripping: Boolean indicating if the gripper should be closed
    (True) or open (False).
    :param serial: SerialData class object for data transmission and
                   -storage.
    Example input:
    gripping = True
    serial = SerialData(6, [0,0,0,0,0,0], [1,1,1,1,1,1], 
                        [0.1,0.1,0.1,0.1,0.1,0.1], robot.joints)
    
    Assumption: Closing --> positive encoder count, 
                Opening --> negative encoder count.
    TODO: Validate assumption"""

    fullCloseCount = 1000 #TODO: Find proper value!
    fullOpenCount = 0
    closingPWM = 40 #TODO: Find proper value
    if gripping:
        if serial.totCount[-1] < fullCloseCount:
            """If there is an object that prevents the gripper
            from fully closing, closingPWM will force is exerted on 
            the object."""
            serial.mSpeed[-1] = closingPWM
        else:
            serial.mSpeed[-1] = 0
    elif serial.totCount[-1] > fullOpenCount:
        serial.mSpeed[-1] = -closingPWM
    else:
        serial.mSpeed[-1] = 0


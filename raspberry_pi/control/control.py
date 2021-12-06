import os
import sys

#Find directory path of current file
current = os.path.dirname(os.path.realpath(__file__))
#Find directory path of parent folder and add to sys path
parent = os.path.dirname(current)
sys.path.append(parent)

from typing import Union, List, Tuple
import modern_robotics as mr
import numpy as np
import time
import serial
from kinematics.kinematic_funcs import IKSpace, FKSpace
from trajectory_generation.traj_gen import TrajGen, TrajDerivatives
from dynamics.dynamics_funcs import FeedForward
from serial_comm.serial_comm import SReadAndParse
from classes import Robot, SerialData, PID, IKAlgorithmError, InputError
from util import Tau2Curr, Curr2MSpeed, RToEuler, LimDamping

def PosControl(sConfig: Union[np.ndarray, List], eConfig: Union[np.ndarray, List], robot: Robot, serial: SerialData, dt: float, vMax: float, omgMax: float, PIDObj: PID, dtComm: float, dtPID: float): #Removed localMu for testing
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
    :param PIDObj: PID class for storage & execution of PID-related 
                   computations.
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
    ILim = np.array([5,5,5,5,5])
    PIDObj = PID(kP, kI, kD, ILim)
    dtComm = 0.5
    dtPID = 0.02
    """
    print("Checking position inputs...")
    #Is position defined in SE3 or theta?
    if isinstance(sConfig, np.ndarray):
        if isinstance(eConfig, np.ndarray) and sConfig.shape == eConfig.shape:
            if sConfig.shape == (4,4): #Translate both to joints space 
                sConfig, successS = IKSpace(robot.TsbHome, sConfig,
                                    robot.screwAxes, robot.limList)
                eConfig, successE = IKSpace(robot.TsbHome, eConfig, 
                                    robot.screwAxes, robot.limList)
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
            tauPID = PIDObj.Execute(traj[n], thetaCurr, dtPID)
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

def VelControl(robot: Robot, serial: SerialData, vel: 
               Union[List, np.ndarray], dthetaPrev: np.ndarray, 
               dt: float, method: str, dtComm: float, dtPID: float, 
               PIDObj: PID):
    """Single velocity control loop using feed-forward and PID to 
    compute the torque required to obtain the desired velocity, either
    in joint space or end-effector space.
    :param robot: A Robot object describing the robot mathematically.
    :param serial: SerialData object for data transmission and 
                   -storage.
    :param vel: Desired velocity, either as an end-effector twist or a
                list/array of joint velocities.
    :param dthetaPrev: Array of previous velocity.
    :param dt: Time between this control loop and the previous in [s].
    :param method: 'twist' if the velocity is a twist, 'joint' for a 
                   list/array of joint velocities.
    :param dtComm: Interval between microcontroller communications.
                   Ensure this value aligns with the value in the 
                   Teensy's code (in Teensy in [ms], here in [s])!
    :param dtPID: Interval between PID updates in [s].
    :param PIDObj: PID class for storage & execution of PID-related 
                   computations.
    """
    vel = np.array(vel)
    theta = serial.currAngle
    if method == 'twist' or method == 'Twist':
        #Calculate joint velocities with pseudo-inverse Jacobian
        Slist = np.c_[robot.screwAxes[0], robot.screwAxes[1]]
        for i in range(2, len(robot.screwAxes)):
            Slist = np.c_[Slist, robot.screwAxes[i]]
        dtheta = np.dot(np.linalg.pinv(mr.JacobianSpace(Slist, theta)),
                        vel)
    elif method == 'joint' or method == 'Joint':
        dtheta = vel
    else:
        raise InputError("Invalid method. Choose between 'joint'" +
                         "and 'twist'.")
    #Add 'directional limit damping' (k might need tweaking):
    dtheta = LimDamping(theta, dtheta, robot.limList, k=20) 
    ddtheta = (dtheta - dthetaPrev)/dt
    dthetaCurr = (np.array(serial.currAngle) - 
                  np.array(serial.prevAngle))/dtComm
    g = np.array([0,0,-9.81])
    FTip = np.zeros(6) #Velocity control, no FTip
    tauFF = FeedForward(robot, theta, dtheta, ddtheta, g, FTip)
    tauPID = PIDObj.Execute(dtheta, dthetaCurr, dtPID)
    tau = tauFF + tauPID
    I = [Tau2Curr(tau[i], robot.joints[i].gearRatio, robot.joints[i].km, 
                  currLim=2) for i in range(len(robot.joints))]
    #TODO: Confirm conversion factor I --> PWM
    PWM = [round(Curr2MSpeed(current)) for current in I]
    serial.mSpeed[:-1] = PWM
    #TODO: Add current / PWM for gripper

    
def ForceControl(robot: Robot, serial: SerialData,
    FTip: np.ndarray, damping: float, dt: float):
    """ Feed-forward force control with velocity damping to ensure
    safety.
    :param robot: A Robot object describing the robot mathematically.
    :param serial: SerialData object for data transmission and 
                   -storage.
    :param FTip: Desired end-effector wrench
    :param damping: Gain term to limit velocity of the end-effector.
    :param dt: Interval between ForceControl updates.

    NOTE: Due to the lack of force sensors, this code is highly 
    reliant on an accurate model of the robot, which as of writing this
    code is not exactly the case. Mainly frictional terms and masses 
    of links should be researched closely before accurate force control
    can be realised. Alternatively, if one can integrate a force-torque
    sensor at the 'wrist' of the end-effector, this can be used for PI 
    feedback control (do not use derivative-control, as explained in 
    the Modern Robotics book!!!). Lastly, another possibility for more 
    accurate tau control would be adding integrating strain sensors at
    the output shafts of the joints for torque feedback, which can 
    easily be mapped to an end-effector wrench for feedback control.
    """
    theta = serial.currAngle
    g = np.array([0,0,-9.81])
    dtheta = np.zeros(len(robot.joints))
    ddtheta = np.zeros(len(robot.joints))
    tauFF = FeedForward(robot, theta, dtheta, ddtheta, g, FTip)
    kDamping = np.eye(6)*damping
    dthetaCurr = (theta - serial.prevAngle)/dt
    Slist = np.c_[robot.screwAxes[0], robot.screwAxes[1]]
    for i in range(2, len(robot.screwAxes)):
        Slist = np.c_[Slist, robot.screwAxes[i]]
    twist = np.dot(mr.JacobianSpace(Slist, theta), dthetaCurr)
    """The final term in tau prevents the end-effector from quickly
    moving / accelerating when it is not pushing against anything."""
    tau = tauFF - np.dot(kDamping, twist) 
    I = [Tau2Curr(tau[i], robot.joints[i].gearRatio, robot.joints[i].km, 
                  currLim=2) for i in range(len(robot.joints))]
    #TODO: Confirm conversion factor I --> PWM
    PWM = [round(Curr2MSpeed(current)) for current in I]
    serial.mSpeed[:-1] = PWM
    #TODO: Add current / PWM for gripper

def ImpControl(robot: Robot, serial: SerialData, TDes: np.ndarray,
               VPrev: np.ndarray, dthetaPrev: np.ndarray, dt: float, 
               M: np.ndarray, B: np.ndarray, Kx: np.ndarray, 
               Ka: np.ndarray, PIDObj: PID):
    """Impedance control loop in the end-effector space.
    :param robot: A Robot object describing the robot mathematically.
    :param serial: SerialData object for data transmission and 
                   -storage.
    :TDes: Desired end-effector configuration as an SE(3) 
           transformation matrix.
    :param VPrev: Previous end-effector twist.
    :pram dthetaPrev: Array of previous joint velocities.
    :param dt: time between two calls of ImpControl in [s].
    :param M: Positive-definite 6x6 impedance mass matrix
    :param B: Positive-definite 6x6 impedance damping matrix.
    :param Kx: Postive-definite 3x3 impedance linear spring matrix.
    :param Ka: 3x3 impedance rotational spring matrix."""
    """Determine position & end-effector angles relative to the desired 
    configuration."""
    R,posDes = mr.TransToRp(TDes)
    thetaDes = IKSpace(robot.TsbHome, TDes, robot.screwAxes, robot.limList)
    anglesDes = RToEuler(R)
    TCurr = FKSpace(robot.TsbHome, robot.screwAxes, serial.currAngle[:-1])
    RCurr, posCurr = mr.TransToRp(TCurr)
    anglesCurr = RToEuler(RCurr)
    pos = posCurr - posDes
    angles = anglesCurr - anglesDes

    """Obtain the error twist and derivative of the error twist"""
    theta = serial.currAngle[:-1]
    dtheta = (serial.currAngle[:-1] - serial.prevAngle[:-1])/dt
    Slist = np.c_[robot.screwAxes[0], robot.screwAxes[1]]
    for i in range(2, len(robot.screwAxes)):
        Slist = np.c_[Slist, robot.screwAxes[i]]
    V = np.dot(mr.JacobianSpace(Slist, theta), dtheta)
    dV = (V - VPrev)/dt
    ddtheta = (dtheta - dthetaPrev)/dt
    g = np.array([0,0,-9.81])
    FlinK = np.zeros(6)
    FlinK[0:3] = np.dot(Kx, pos)
    FrotK = np.zeros(6)
    FrotK[0:3] = np.dot(Ka, angles)
    FK = (FlinK + FrotK)
    #Simulate a mass-spring-damper system at the end-effector
    FTip = -(np.dot(M,dV) + np.dot(B,V) + FK)
    ###EXPERIMENTAL PID, uncomment at own risk###
    # if np.allclose(FTip, np.zeros(6), atol=0.01): #FTip has stabilized
    #     #NOTE: atol might need tweaking!
    #     FTip = np.zeros(6)
    #     #PID in theta-space
    #     tauPID = PIDObj.Execute(thetaDes, serial.currAngle, dt)
    # else:
    #     PIDObj.errPrev = 0
    #     PIDObj.termI = np.zeros(len(robot.joints))
    #     tauPID = 0
    tauFF = FeedForward(robot, theta, dtheta, ddtheta, g, FTip)
    tauPID = np.zeros(len(tauFF)) #Remove if experimental PID is uncommented
    tau = tauFF + tauPID
    I = [Tau2Curr(tau[i], robot.joints[i].gearRatio, robot.joints[i].km, 
                  currLim=2) for i in range(len(robot.joints))]
    #TODO: Confirm conversion factor I --> PWM
    PWM = [round(Curr2MSpeed(current)) for current in I]
    serial.mSpeed[:-1] = PWM
    #TODO: Add current / PWM for gripper
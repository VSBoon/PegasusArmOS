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
import pygame
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
from kinematics.kinematic_funcs import IKSpace, FKSpace
from trajectory_generation.traj_gen import TrajGen, TrajDerivatives
from dynamics.dynamics_funcs import FeedForward
from serial_comm.serial_comm import SReadAndParse
from classes import Robot, SerialData, PID, IKAlgorithmError, InputError
from util import Tau2Curr, Curr2MSpeed, RToEuler, LimDamping

def PosControl(sConfig: Union[np.ndarray, List], eConfig: Union[np.ndarray, List], robot: Robot, serial: SerialData, dt: float, vMax: float, omgMax: float, PIDObj: PID, dtComm: float, dtPID: float, dtFrame: float, localMu: serial.Serial, screen: pygame.Surface, background: pygame.Surface): 
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
    :param localMu: Serial-object for local microcontroller comms.
    :param screen: Pygame screen object.
    :param background: Pygame background image object.
    
    Example input:
    Initialisation of Robot args is omitted for the sake of brevity.
    robot = Robot(joints, links, TsbHome)
    serial = SerialData(6, [0,0,0,0,0,0], [1,1,1,1,1,1], [0.1,0.1,0.1,0.1,0.1,0.1], robot.joints)
    port = FindSerial()[0]
    localMu = StartComms(port)
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
    screen = pygame.display.set_mode([700, 500])
    background = pygame.image.load(os.path.join(current,'control_overview.png'))
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
    g = np.array([0,0,-9.81*2]) #EXPERIMENTAL!
    FTip = np.zeros(6) #Position control, --> assume no end-effector force.
    tauPID = np.zeros(traj[0,:].size)
    n = -1 #iterator
    PWM = [0 for i in range(5)]
    startTime = time.perf_counter()
    lastFrame = time.perf_counter()
    lastWrite = time.perf_counter()
    lastPID = time.perf_counter()
    print("Starting trajectory...")
    while time.perf_counter() - startTime < dt*traj[:,0].size: #Trajectory loop
        nPrev = n
        n = round((time.perf_counter()-startTime)/dt)
        if n >= traj[:,0].size:
            break
        if n != nPrev:
            tauFF = FeedForward(robot, traj[n], velTraj[n], accTraj[n], g, FTip)
            #hacky fix, but works for now
            tauFF[0] = [0 if velTraj[n,0] == 0 else tauFF[0]][0]
        if time.perf_counter() - lastPID >= dtPID:
            thetaCurr = np.array(serial.currAngle[:-1]) #Exclude gripper data
            thetaPrev = np.array(serial.prevAngle[:-1])
            dthetaCurr = (thetaCurr - thetaPrev)/dtPID
            tauPID = PIDObj.Execute(traj[n], thetaCurr, dtPID)
            lastPID = time.perf_counter()
            tau = tauFF + tauPID
            tauFric = np.zeros(5)
            for i in range(tau.size):
                """Compute joint friction based on friction model of joint."""
                if not np.isclose(tauPID[i], 0, atol=1e-05) and np.isclose(dthetaCurr[i], 0, atol=1e-01):
                    tauStat = robot.joints[i].fricPar['stat']
                    tauFric[i] = tauStat*np.sign(tau[i])
                elif np.isclose(tauPID[i], 0, atol=1e-05) and not np.isclose(dthetaCurr[i],0,atol=1e-01):
                    tauKin = robot.joints[i].fricPar['kin']
                    tauFric[i] = tauKin*np.sign(tau[i]) 
            tau += tauFric
            #Diff-drive properties:
            tauJ4 = tau[3]
            tauJ5 = tau[4]
            tau[3] = (-tauJ4*1.1 - tauJ5) #This motor struggle more than the other
            tau[4] = tauJ4 - tauJ5
            # I = [Tau2Curr(tau[i], robot.joints[i].gearRatio, robot.joints[i].km, 
            #               currLim=2) for i in range(len(robot.joints))]
            PWM = tau*33.78 #EXPERIMENTAL
            PWM = np.round(LimDamping(thetaCurr, PWM, robot.limList, k=20))
            #Take care of communication on an interval basis:
        if (time.perf_counter() - lastWrite >= dtComm):
            SReadAndParse(serial, localMu)
            serial.mSpeed[:-1] = [abs(val) for val in PWM]
            serial.rotDirDes = [np.sign(PWM[i]) if 
                                np.sign(PWM[i]) == 1 else 0 for i in 
                                range(serial.lenData-1)]
            for i in range(serial.lenData-1): #TODO: Add Gripper function
                serial.dataOut[i] = f"{abs(serial.mSpeed[i])}|"+\
                                    f"{serial.rotDirDes[i]}"
            #TODO: Replace last entry w/ gripper commands
            serial.dataOut[-1] = f"{0|0}"
            localMu.write(f"{serial.dataOut}\n".encode('utf-8')) 
            lastWrite = time.perf_counter()
        #Take care of PyGame screen on an interval basis:
        now = time.perf_counter()
        if now - lastFrame >= dtFrame:
            events = pygame.event.get() #To interact with pygame, avoids freezing.
            for event in events:
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt
            screen.blit(background, (0,0))
            pygame.display.update()
            lastFrame = time.perf_counter()
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
               dt: float, method: str, dtComm: float,PIDObj: PID) -> \
               np.ndarray:
    """Single velocity control loop using feed-forward and PID to 
    compute the torque required to obtain the desired velocity, either
    in joint space or end-effector space.
    :param robot: A Robot object describing the robot mathematically.
    :param serial: SerialData object for data transmission and 
                   -storage.
    :param vel: Desired velocity, either as an end-effector twist or a
                list/array of joint velocities.
    :param dthetaPrev: Array of previous actual velocity.
    :param dt: Time between this control loop and the previous in [s].
    :param method: 'twist' if the velocity is a twist, 'joint' for a 
                   list/array of joint velocities.
    :param dtComm: Interval between microcontroller communications.
                   Ensure this value aligns with the value in the 
                   Teensy's code (in Teensy in [ms], here in [s])!
    :param PIDObj: PID class for storage & execution of PID-related 
                   computations.
    :return dtheta: Current desired joint velocities (w/ limit damping) 
    """
    vel = np.array(vel)
    theta = serial.currAngle[:-1]
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
    #Pegasus characteristics: Move with the previous joint
    dtheta[2] += dtheta[1]
    dtheta[3] += dtheta[2]
    ddtheta = (dtheta - dthetaPrev)/dt
    dthetaCurr = (np.array(serial.currAngle[:-1]) - 
                  np.array(serial.prevAngle[:-1]))/dtComm
    g = np.array([0,0,-9.81])
    FTip = np.zeros(6) #Velocity control, no FTip
    tauFF = FeedForward(robot, theta, dtheta, ddtheta, g, FTip)
    tauPID = PIDObj.Execute(dtheta, dthetaCurr, dt)
    tau = tauFF + tauPID
    #Diff-drive properties:
    tauJ4 = tau[3]
    tauJ5 = tau[4]
    tau[3] = -tauJ4 - tauJ5
    tau[4] = tauJ4 - tauJ5
    I = [Tau2Curr(tau[i], robot.joints[i].gearRatio, robot.joints[i].km, 
                  currLim=2) for i in range(len(robot.joints))]
    #TODO: Confirm conversion factor I --> PWM
    PWM = [Curr2MSpeed(current) for current in I]
    #Add 'directional limit damping' (k might need tweaking):
    PWM = LimDamping(theta, PWM, robot.limList, k=20)
    PWM = [round(val) for val in PWM]
    serial.mSpeed[:-1] = PWM
    #TODO: Add current / PWM for gripper
    return dthetaCurr

    
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
    theta = np.array(serial.currAngle[:-1]) #Exclude gripper
    g = np.array([0,0,-9.81])
    dtheta = np.zeros(len(robot.joints))
    ddtheta = np.zeros(len(robot.joints))
    kDamping = np.eye(6)*damping
    dthetaCurr = (theta - np.array(serial.prevAngle[:-1]))/dt
    Slist = np.c_[robot.screwAxes[0], robot.screwAxes[1]]
    for i in range(2, len(robot.screwAxes)):
        Slist = np.c_[Slist, robot.screwAxes[i]]
    twist = np.dot(mr.JacobianSpace(Slist, theta), dthetaCurr)
    """The final term in FTip prevents the end-effector from quickly
    moving / accelerating when it is not pushing against anything."""
    FTip -= np.dot(kDamping,twist)
    tau = FeedForward(robot, theta, dtheta, ddtheta, g, FTip)
    #Diff-drive properties:
    tauJ4 = tau[3]
    tauJ5 = tau[4]
    tau[3] = -tauJ4 - tauJ5
    tau[4] = tauJ4 - tauJ5
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
    dtheta = (np.array(serial.currAngle[:-1]) - np.array(serial.prevAngle[:-1]))/dt
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
    #Diff-drive properties:
    tauJ4 = tau[3]
    tauJ5 = tau[4]
    tau[3] = -tauJ4 - tauJ5
    tau[4] = tauJ4 - tauJ5
    I = [Tau2Curr(tau[i], robot.joints[i].gearRatio, robot.joints[i].km, 
                  currLim=2) for i in range(len(robot.joints))]
    #TODO: Confirm conversion factor I --> PWM
    PWM = [round(Curr2MSpeed(current)) for current in I]
    serial.mSpeed[:-1] = PWM
    #TODO: Add current / PWM for gripper
    return V, dtheta

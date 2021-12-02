import os
import sys
#Find directory path of current file
current = os.path.dirname(os.path.realpath(__file__))
#Find directory path of parent folder and add to sys path
parent = os.path.dirname(current)
sys.path.append(parent)

from classes import Robot, Joint, Link
from robot_init import robot
from typing import List, Tuple
import modern_robotics as mr
import numpy as np

""" NOTE: IF NOT TESTED ON RPi, ALL MENTIONS OF RPi LIBRARY SHOULD BE
    REMOVED FROM ALL IMPORTS BEFORE RUNNING THESE TESTS!
"""
np.set_printoptions(precision=5, suppress=True)

def FricTau(tauComm: float, dtheta: float,  tauStat: float, 
            bVisc: float, tauKin: float, eff: float) -> float:
    """Calculates friction torque according to a model utilizing 
    static, viscous, and kinetic friction coefficients. 
    :param tauComm: Desired torque based on inverse dynamics at the 
                    output shaft.
    :param dtheta: Desired joint velocity.
    :param tauStat: Torque required to overcome static friction 
                    (occurs if dthetaPrev == 0).
    :param bVisc: Viscous friction coefficient in (N*m^2*s)/rad
    :param tauKin: Torque required to overcome kinetic friction.
                   NOTE: tauKin should be smaller than tauStat!
    :param eff: Efficiency (heat dissipation, gearbox losses, etc.)
    :return tauComm: Desired output torque, including friction- and
                     efficiency compensation.
    NOTE: All variables are taken w.r.t the output shaft, so after 
    the internal & external gearbox!
    """
    tauFric = 0
    if np.isclose(dtheta, 0, atol=1e-04) and \
        not tauComm == 0: #Static friction
        if dtheta != 0:
            dir = np.sign(dtheta)
        else:
            dir = np.sign(tauComm) #Reasonable approximation
        tauFric = tauStat*dir
    elif not tauComm == 0: #Kinetic & viscous friction
        tauFric = tauKin*np.sign(dtheta) + \
                   bVisc*dtheta
    tauFric += tauComm/eff - tauComm
    return tauFric

def FeedForward(robot: Robot, theta: List, dtheta: List, ddtheta: List, 
                g: np.ndarray, FTip: np.ndarray) -> np.ndarray:
    """Optimized feed-forward for the Pegasus arm, only going through 
    the Newton-Euler inverse dynamics algorithm once. Based on the
    Modern Robotics Python Library.
    :param robot: A Robot class describing the robot mathematically.
    :param thetaList: List of desired joint angles.
    :param dthetaList: List of desired joint velocities.
    :param ddthetaList: List of desired joint accelerations.
    :param g: 3-dimensional gravity vector in [m/s^2].
    :param FTip: 6-dimensional End-effector wrench.
    :return tau: A list of required joint torques at the output shaft.

    Example input:
    (Init of Robot parameters not included for brevity)
    robot = Robot(joints, links, TsbHome)
    thetaList = [0,0.5*np.pi, 0.25*np.pi, 0.25*np.pi, 0]
    dthetaList = [0,1,0,0,0]
    ddtheta = [0,1,1,1,1]
    g = np.array([0,0,-9.81])
    FTip = np.array([1,1,1,1,1,1])
    Output:
    [ 0.81  -1.343  0.767  1.754  0.958]
    """
    #Initialization
    #Note: 6,shape is used to obtain column vectors for calculations.
    n = len(theta) #Number of joints
    Tsi = np.eye(4)
    AdHis = np.zeros((n+1,6,6))
    screwS = np.c_[robot.screwAxes[0], robot.screwAxes[1]]
    for i in range(2,n):
        screwS = np.c_[screwS, robot.screwAxes[i]]
    screwA = np.zeros((6, n))
    expT = np.zeros((n,4,4))
    AdTiiN = np.zeros((n+1,6,6))
    """The end-effector frame (TllList[n]) is rigidly connected
    to the frame of the previous link --> constant T_(n-1,n)"""
    AdTiiN[n] = mr.Adjoint(mr.TransInv(robot.TllList[n]))
    V = np.zeros((6, n+1))
    dV = np.zeros((6, n+1))
    dV[3:,0] = -g #Gravity is equivalent to upward acceleration
    #This way, the gravity gets 'carried on' in forward iterations.
    F = np.zeros((6, n+1))
    F[:, n] = FTip
    tau = np.zeros(n)
    tauFric = np.zeros(n)
    #Forward iterations
    for i in range(n):
        if i != n-1:
            #Adjoint of the transformation matrix {i} to {s} in the home config
            #For some reason, Tsi = robot.links[i].Tsi gives the wrong answer...
            Tsi = np.dot(Tsi, robot.links[i].Tii)
            AdHis[i] = mr.Adjoint(mr.TransInv(Tsi))
            #Screw axis in frame {i}. np.array([]) to go from 1D to 2D
            screwA[:,i] = np.dot(AdHis[i],screwS[:,i])
            #Exponential of A with angle theta
            expT[i] = mr.MatrixExp6(mr.VecTose3(screwA[:,i]*-theta[i]))
            #Adjoint of T_(i,i-1), given current config:
            AdTiiN[i] = mr.Adjoint(np.dot(expT[i],mr.TransInv(robot.TllList[i])))
            #Twist due to joint velocity + twist due to prev. link
            V[:,i+1] = screwA[:,i]*dtheta[i] + np.dot(AdTiiN[i], V[:,i])
            #Acceleration due to joint acceleration + vel-prod. term + prev link
            dV[:,i+1] = screwA[:,i]*ddtheta[i] + np.dot(AdTiiN[i], dV[:,i]) + \
                    np.dot(mr.ad(V[:,i+1]), screwA[:,i]*dtheta[i])
        elif i == n-1:
            AdHis[i] = mr.Adjoint(mr.TransInv(Tsi))
            screwA[:,i] = np.dot(AdHis[i],screwS[:,i])
            expT[i] = mr.MatrixExp6(mr.VecTose3(screwA[:,i]*-theta[i]))
            AdTiiN[i] = mr.Adjoint(np.dot(expT[i],mr.TransInv(robot.TllList[i])))
            #NOTE: Prev twist == V[:,i-1] & Prev acc == dV[:,i-1]
            V[:,i+1] = screwA[:,i]*dtheta[i] + np.dot(AdTiiN[i], V[:,i-1])
            dV[:,i+1] = screwA[:,i]*ddtheta[i] + np.dot(AdTiiN[i], dV[:,i-1]) + \
                    np.dot(mr.ad(V[:,i+1]), screwA[:,i]*dtheta[i])
        """Compute joint friction based on friction model of joint."""
        tauStat = robot.joints[i].fricPar['stat']
        tauKin = robot.joints[i].fricPar['kin']
        bVisc = robot.joints[i].fricPar['visc']
        eff = robot.joints[i].fricPar['eff']
        tauFric[i] = FricTau(tau[i], dtheta[i], tauStat, bVisc, tauKin, eff)

    #Backward iterations
    for i in range(n-1, -1, -1):
        if i != n-2:
            """Wrench due to wrench next link + link acceleration - 
            Coriolis/centripetal terms"""
            F[:,i] = np.dot(AdTiiN[i+1].T, F[:,i+1]) + \
                np.dot(robot.GiList[i], dV[:,i+1]) - \
                np.dot(mr.ad(V[:,i+1]).T, np.dot(robot.GiList[i], V[:,i+1]))
        elif i == n-2: #Pegasus arm specific mechanics
            F[:,i] = np.dot(AdTiiN[i+2].T, F[:,i+2]) + \
                np.dot(robot.GiList[i], dV[:,i+1]) - \
                np.dot(mr.ad(V[:,i+1]).T, np.dot(robot.GiList[i], V[:,i+1]))
        """Obtain torques by projection through screwA, effectively a
        column of the Jacobian between wrenches/twists in {i} and 
        joint space"""
        tau[i] = np.dot(F[:,i].T, screwA[:,i]) + tauFric[i] 
    return tau

def MassMatrix(robot: Robot, theta: List) -> np.ndarray:
    """Computes the mass matrix at the given configuration by calling
    the inverse dynamics n-times, once for each column with everything
    set to zero except one joint accelleration, set at 1.
    :param robot: A Robot object describing the robot mathematically. 
    :param theta: List of current joint angles
    :return M: nxn Mass matrix, mapping joint accellerations to 
               joint torques in the current configuration.
    
    Example input:
    (Init of Robot parameters not included for brevity)
    robot = Robot(joints, links, TsbHome)
    theta = [0,0,0,0,0]
    Output:
    [[ 0.07049  0.00835  0.00436  0.       0.     ]
     [ 0.00835  0.22213  0.12376  0.00282  0.     ]
     [ 0.00436  0.12376  0.10763  0.0032   0.     ]
     [ 0.       0.00282  0.0032   0.00364  0.     ]
     [-0.00008  0.00008  0.00001  0.       0.00048]]
    """
    n = len(theta)
    M = np.zeros((n,n))
    dtheta = [0 for i in range(n)]
    g = np.array([0,0,0])
    FTip = np.array([0,0,0,0,0,0])
    for i in range(n): #Obtain the ith column of the mass matrix
        ddtheta = [0 for i in range(n)]
        ddtheta[i] = 1 
        M[:, i] = FeedForward(robot, theta, dtheta, ddtheta, g, FTip)
    return M

def CorrCentTorques(robot: Robot, theta: List, dtheta: List) -> np.ndarray:
    """Computes the torques needed to overcome the quadratic velocity 
    forces caused by the Coriolis effect and centripetal forces by 
    calling the inverse dynamics with everything set to zero, 
    except for the joint configurations and -velocities.
    :param robot: A Robot object describing the robot mathematically.
    :param theta: List of desired joint angles.
    :param dtheta: List of desired joint velocities.
    :return tauC: Torques needed to adhere to the Coriolis- and 
                  centripetal forces
    
    Example input:
    (Init of Robot parameters not included for brevity)
    robot = Robot(joints, links, TsbHome)
    theta = [1,1,1,1,1]
    dtheta = [1,1,1,1,1]
    Output:
    [-0.0887  -0.03611  0.04584  0.00264 -0.0074]
    """
    n = len(theta)
    ddtheta = [0]*n
    g = np.array([0,0,0])
    FTip = np.array([0,0,0,0,0,0])
    tauC = FeedForward(robot, theta, dtheta, ddtheta, g, FTip)
    return tauC

def GravTorques(robot: Robot, theta: List, g: np.ndarray) -> np.ndarray:
    """Computes the torques required to overcome gravity in the current
    configuration by calling the inverse dynamics algorithm with only 
    the gravity vector and current joint configuration.
    :param robot: A Robot object describing the robot mathematically.
    :param theta: List of desired joint angles.
    :param g: 3-vector describing gravity accelleration in the space 
              frame.
    Example input:
    (Init of Robot parameters not included for brevity)
    robot = Robot(joints, links, TsbHome)
    theta = [1,1,1,1,1]
    g = np.array([0,0,-9.81])
    Output:
    [ 0.00225 -2.62294 -0.54641  0.12139 -0.02637]
    """
    n = len(theta)
    dtheta = [0] * n
    ddtheta = [0] * n
    FTip = np.array([0,0,0,0,0,0])
    tauG = FeedForward(robot, theta, dtheta, ddtheta, g, FTip)
    return tauG

def FTipTorques(robot: Robot, theta: List, FTip: np.ndarray) -> np.ndarray:
    """Computes the torques required to generate the desired end-
    effector wrench FTip in the desired configuration by calling the
    inverse dynamics algorithm with all dynamic parameters zero except 
    for the end-effector wrench and joint configuration.
    :param robot: A Robot object describing the robot mathematically.
    :param theta: List of desired joint angles.
    :param FTip: 6-vector representing the desired wrench at the end-
                 effector
    
    Example input:
    (Init of Robot parameters not included for brevity)
    robot = Robot(joints, links, TsbHome)
    theta = [1,1,1,1,1]
    FTip = np.array([1,1,1,1,1,1])
    Output:
    [-0.87197  1.2791   1.16805  0.85243  1.01331]
    """
    n = len(theta)
    dtheta = [0]*n
    ddtheta = [0]*n
    g = np.array([0,0,0])
    tauF = FeedForward(robot, theta, dtheta, ddtheta, g, FTip)
    return tauF

def ForwardDynamics(robot: Robot, theta: List, dtheta: List, 
                    tau: np.ndarray, g: np.ndarray, FTip: np.ndarray) \
                    -> List:
    """Calculates the joint accellerations given the supplied torque,
    current joint configuration and -velocities, as well as a gravity 
    vector and end-effector wrench, by calling the inverse dynamics 
    algorithm n+3 times, for n being the number of robot joints.
    :param robot: A Robot object describing the robot mathematically.
    :param theta: List of current joint angles.
    :param dtheta: List of current joint velocities.
    :param tau: Array of current supplied joint torques.
    :param g: 3-vector describing gravitational accelleration in the 
              space frame.
    :param FTip: 6-vector representing the desired wrench at the end-
                 effector
    
    Example input:
    (Init of Robot parameters not included for brevity)
    robot = Robot(joints, links, TsbHome)
    theta = [1,1,1,1,1]
    dtheta = [1,1,1,1,1]
    tau = [1,1,1,1,1]
    g = np.array([0,0,-9.81])
    FTip = np.array([1,1,1,1,1,1])
    Output:
    [8.3721, 25.2644, -24.1423, 23.0293, 22.2858]
    """
    M = MassMatrix(robot, theta)
    tauC = CorrCentTorques(robot, theta, dtheta)
    tauG = GravTorques(robot, theta, g)
    tauF = FTipTorques(robot, theta, FTip)
    tauFric = np.zeros(len(theta))
    for i in range(len(theta)):
        tauStat = robot.joints[i].fricPar['stat']
        tauKin = robot.joints[i].fricPar['kin']
        bVisc = robot.joints[i].fricPar['visc']
        eff = robot.joints[i].fricPar['eff']
        tauFric[i] = FricTau(tau[i], dtheta[i], tauStat, bVisc, tauKin, eff)
    #TODO: Consult Sander on wormgear integration
    tauAcc = tau - tauC - tauG - tauF - tauFric
    ddtheta = np.dot(np.linalg.inv(M), tauAcc)
    return ddtheta.tolist()

def SimulateStep(robot: Robot, thetaPrev: List, dthetaPrev: List, 
                 ddthetaPrev: List, tau: np.ndarray, g: np.ndarray, 
                 FTip: np.ndarray, dt: float) -> Tuple[List]:
    """Calculates the joint accelerations, -velocities, and -positions 
    in the next time step based on the previous joint accelerations, 
    -velocities, and -positions, as well as the current joint torques,
    the gravity vector, and end-effector wrench.
    :param robot: A Robot object describing the robot mathematically.
    :param thetaPrev: List of previous joint angles.
    :param dthetaPrev: List of previous joint velocities.
    :param tau: Array of current supplied joint torques.
    :param g: 3-vector describing gravitational acceleration in the 
              space frame.
    :param FTip: 6-vector representing the desired wrench at the end-
                 effector.
    :param dt: Time between current and previous iteration in seconds.
    :return theta: List of current joint angles.
    :return dtheta: List of current joint velocities.
    :return ddtheta: List of current joint accelerations. 

    Example input:
    (Init of Robot parameters not included for brevity)
    robot = Robot(joints, links, TsbHome)
    thetaPrev = [1,1,1,1,1]
    dthetaPrev = [1,1,1,1,1]
    ddthetaPrev = [1,1,1,1,1]
    tau = [1,1,1,1,1]
    g = np.array([0,0,-9.81])
    FTip = np.array([1,1,1,1,1,1])
    dt = 0.001
    Output:
    [1.0010, 1.0010, 1.0010, 1.0010, 1.0010]
    [1.0052, 1.0136, 0.9889, 1.0125, 1.0121]
    [8.3721, 25.2644, -22.1423, 23.0293, 22.2858]
    NOTE: Excluded the effect of model inaccuracies & PID for more 
    advanced simulation.
    """
    n = len(thetaPrev)
    theta = [None for i in range(n)]
    dtheta = [None for i in range(n)]
    ddtheta = ForwardDynamics(robot, thetaPrev, dthetaPrev, tau, g, FTip)
    for i in range(len(ddtheta)):
         #Trapezoidal integration
        dtheta[i] = dthetaPrev[i] + ((ddthetaPrev[i] + ddtheta[i])/2)*dt
        theta[i] = thetaPrev[i] + ((dthetaPrev[i] + dtheta[i])/2)*dt 
    return (theta, dtheta, ddtheta)

if __name__ == "__main__":
    thetaPrev = [1,1,1,1,1]
    dthetaPrev = [1,1,1,1,1]
    ddthetaPrev = [1,1,1,1,1]
    tau = [1,1,1,1,1]
    g = np.array([0,0,-9.81])
    FTip = np.array([1,1,1,1,1,1])
    dt = 0.001
    theta, dtheta, ddtheta = SimulateStep(robot, thetaPrev, dthetaPrev, ddthetaPrev, tau, g, FTip, dt)
    print(['%.4f' % t for t in theta])
    print(['%.4f' % v for v in dtheta])
    print(['%.4f' % a for a in ddtheta])
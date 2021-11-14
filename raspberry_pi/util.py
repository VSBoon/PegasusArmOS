import numpy as np
from typing import List, Tuple
    
def screwsToMat(screws: List[np.ndarray]) -> np.ndarray:
    """Translates list of screw axes into a matrix desired by the 
    Modern Robotics library.
    :param screws: List of 6x1 screw vectors, as per Definition
                   3.24 of the Modern Robotics book (but transposed 
                   relatively).
    :return screwMat: nx6 matrix with the screw axes as the columns.
    Example input:
    screws = (np.array([[0, 1, 0, 0, 0, 0.2]]), 
              np.array([[0, 0, 1, 0.1, 0, 0]]))
    Output:
    np.array([[0, 0]
              [1, 0]
              [0, 0]
              [0, 0.1]
              [0, 0]
              [0, 0.2]]"""
    screwMat = screws[0]
    for i in range(1,len(screws)):
        screwMat = np.hstack((screwMat, screws[i]))
    return screwMat

def screwsToMat1D(screws1D: List[np.ndarray]) -> np.ndarray:
    """Translates list of one dimensional screw axes into a matrix 
    desired by the Modern Robotics library.
    :param screws1D: List of 1D screw vectors with 6 entries.
    :return screwMat: nx6 matrix with the screw axes as the columns.
    Example input:
    screws = (np.array([0, 1, 0, 0, 0, 0.2]), 
              np.array([0, 0, 1, 0.1, 0, 0]))
    Output:
    np.array([[0, 0]
              [1, 0]
              [0, 0]
              [0, 0.1]
              [0, 0]
              [0, 0.2]]"""
    screwMat = screws1D[0].reshape(screws1D[0].shape[0], 1)
    for i in range(1, len(screws1D)): #Make transposed 2D arrays
        screwMat = np.hstack((screwMat, screws1D[i].
        reshape(screws1D[i].shape[0], 1))) 
    return screwMat

def screwsToMatT(screwsT: List[np.ndarray]) -> np.ndarray:
    """Translates list of transposed screw axes into a matrix 
    desired by the Modern Robotics library.
    :param screws1D: List of 1x6 screw vectors, as per Definition
                     3.24 of the Modern Robotics book.
    :return screwMat: nx6 matrix with the screw axes as the columns.
    Example input:
    screws = (np.array([[0, 1, 0, 0, 0, 0.2]].T), 
              np.array([[0, 0, 1, 0.1, 0, 0]].T))
    Output:
    np.array([[0, 0]
              [1, 0]
              [0, 0]
              [0, 0.1]
              [0, 0]
              [0, 0.2]]"""
    screwMat = screwsT[0].T
    for i in range(1,len(screwsT)):
        screwMat = np.hstack((screwMat, screwsT[i].T))
    return screwMat

def ThetaInitGuess(psbHome: np.ndarray, psbTarget: np.ndarray, majorScrewJoints: 
                   List[np.ndarray], jointLimits: List[List[float]]) -> float:
    """Computes the initial angle gues of an inverse-kinematics 
    problem for the given joint.
    :param psbHome: the 3-vector describing the coordinates of the end-
                    effector in its home configuration.
    :param psbTarget: the 3-vector describing the coordinates of the end-
                      effector in its goal configuration.
    :param spaceScrew: List containing the screw axes in the space 
                       frame (6x1) of the joints nearest to the base 
                       of the robot. 
    :param jointLimits: A list of the lower & upper joint limits of
                        each joint.
    :return thetaInitGuess: A rudimentary initial guess of the required
                            joint angles to go from the home 
                            configuration to the target configuration.
    Example input:
    psbCurrent = np.array([1,0,0])
    psbTarget = np.array([2,0,0])
    majorScrewJoints = [np.array([0,1,0,-1,0,0])]
    jointLimits = [[-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi], 
                   [-np.pi, np.pi], [-np.pi, np.pi]]
    Output:
    [-0.32, 0, 0, 0, 0]
    Known limits: 
    * Only works with rotational axes in line with major
    axes of the space frame.
    * Only works with revolute joints"""
    nJoints = len(jointLimits)
    thetaGuessList = [0]*nJoints
    for i in range(len(majorScrewJoints)):
        #Project onto plane of rotation
        rotAxis = np.where((majorScrewJoints[i] == 1) | 
                           (majorScrewJoints[i] == -1))[0][0]
        if rotAxis == 0: #x-axis
            psbHome2D = np.take(psbHome, [1,2])
            psbTarget2D = np.take(psbTarget, [1,2])
            #The linear velocity in the screw axis is caused by the distance
            #orthogonal to the said velocity direction. 
            pJoints2D = np.take(majorScrewJoints[i], [5,4])
        elif rotAxis == 1: #y-axis
            psbHome2D = np.take(psbHome, [0,2])
            psbTarget2D = np.take(psbTarget, [0,2])
            pJoints2D = np.take(majorScrewJoints[i], [5,3])
        elif rotAxis == 2: #z-axis
            psbHome2D = np.take(psbHome, [0,1])
            psbTarget2D = np.take(psbTarget, [0,1])
            pJoints2D = np.take(majorScrewJoints[i], [4,3])
        pJointTarget2D = psbTarget2D + pJoints2D
        pJointb2D =  psbHome2D + pJoints2D
        thetaGuessList[i] = np.round(np.arctan2(np.cross(pJointTarget2D, 
                                     pJointb2D), np.dot(pJointb2D, 
                                     pJointTarget2D)), decimals=4)
         #Normalize & check limits                               
        if thetaGuessList[i] > np.pi:
            thetaGuessList[i] = thetaGuessList[i] % np.pi
        elif thetaGuessList[i] < -np.pi:
            thetaGuessList[i] = thetaGuessList[i] % -np.pi
        if thetaGuessList[i] > jointLimits[i][1]:
            thetaGuessList[i] = jointLimits[i][1]
        elif thetaGuessList[i] < jointLimits[i][0]:
            thetaGuessList[i] = jointLimits[i][0]
    return thetaGuessList

def Tau2Curr(tauComm: float, gRatio: float, km: float, 
             currLim: float) -> float:
    """Computes the to-be commanded current based on the desired 
    output torque.
    :param tauComm: Desired output torque based on inverse dynamics and
                    additional estimated losses.
    :param gRatio: Gearbox ratio (>1).
    :param km: Motor constant in [Nm/A].
    :param currLim: Maximum allowed current to each motor in [A].
    :return currMotor: Desired current in [A]."""
    tauMotor = tauComm/gRatio
    currMotor = tauMotor/km
    if currMotor > currLim:
        currMotor == currLim
    return currMotor

def Curr2MSpeed(currMotor: float) -> float:
    """Converts current to PWM motor speed command.
    :param currMotor: Desired current in [A]
    :return mSpeed: PWM value in the range [0,255].
    NOTE: mSpeed should still be normalized to [mSpeedMin, mSpeedMax]!
    """
    linFactor = 2/255 #TODO: FIND ACCURATE LINFACTOR!
    mSpeed = currMotor *linFactor
    return mSpeed

def PID(ref: "np.ndarray[float]", Fdb: "np.ndarray[float]",
        kP: "np.ndarray[float]", kI: "np.ndarray[float]", 
        kD: "np.ndarray[float]", termI: "np.ndarray[float]", 
        ILim: "np.ndarray[float]", dt: float, errPrev: 
        "np.ndarray[float]") -> Tuple["np.ndarray[float]"]:
        """Adds discrete PID error control to a reference signal, 
        given a feedback signal and PID constants.
        :param ref: Reference / Feed-forward signal.
        :param Fdb: Feedback signal.
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
        :return refWPID: Reference signal added with the PID signal.
        :return termI: The new accumulative integral term.
        :return err: The new error calculation.
        
        Example input:
        ref = [0, 1, 2, 3, 4]
        Fdb = [0, 1.1, 1.9, 3.5, 4]
        kP = 1*np.eye(5)
        kI = 0.1*np.eye(5)
        kD = 0.01*np.eye(5)
        termI = [0,0,0,0,0]
        ILim = [5,5,5,5,5]
        dt = 0.01
        errPrev = [0,0,0,0,0]

        Output:
        ([0.  0.7 2.3 1.5 4. ],
        [0. 0. 0. 0. 0. ],
        [ 0.  -0.1  0.1 -0.5  0. ])
        """
        err = ref - Fdb
        termP = np.dot(kP, err)
        if any(abs(termI) >= ILim):
            termI = [np.sign(termI[i])*ILim[i] if termI[i] >= ILim[i] else termI[i] for i in range(err.size)]
        else: #Trapezoidal integration
            trpz = dt*(errPrev + err)/2
            np.add(termI, np.dot(kI, trpz), out=termI, casting="unsafe")
        termD = np.dot(kD, (err - errPrev)/dt)
        refWPID = ref + termP + termI + termD
        return refWPID, termI, err
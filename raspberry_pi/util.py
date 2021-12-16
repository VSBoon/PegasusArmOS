import numpy as np
import modern_robotics as mr
from typing import List
    
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

def RToEuler(R: np.ndarray):
    """Calculates one solution of Euler angles related to a SO(3)
    rotation matrix.
    :param R: An SO(3) rotation matrix.
    :return eulerAngles: Array of Euler angles satisfying R.
    NOTE: A second set of Euler angles satisfying R can often be found 
    by setting thetaX2 = np.pi - thetaX and solving the rest of the 
    algorithm accordingly with thetaX2. 
    
    Example input: 
    R = np.array([[0,0,1],[0,-1,0],[1,0,0]])
    Output:
    [-1.57079633  3.14159265  0.        ]

    NOTE: Credits to Gregory G. Slabaugh of Queen Mary University of 
    London for writing the pseudo-code for this function."""
    if abs(R[2,0]) != 1:
        thetaY = -np.arcsin(R[2,0])
        thetaX = np.arctan2(R[2,0]/np.cos(thetaY), R[2,2]/np.cos(thetaY))
        thetaZ = np.arctan2(R[1,0]/np.cos(thetaY), R[0,0]/np.cos(thetaY))
    else:
        thetaZ = 0
        if R[2,0] == -1:
            thetaY = np.pi/2
            thetaX = thetaZ + np.arctan2(R[0,1], R[0,2])
        else:
            thetaX = -np.pi/2
            thetaY = -thetaZ + np.arctan2(-R[0,1], -R[0,2])
        #Normalize angles
        thetaX = np.mod(thetaX,2*np.pi)
        thetaY = np.mod(thetaY,2*np.pi)
        thetaZ = np.mod(thetaZ,2*np.pi)
    return np.array([thetaX, thetaY, thetaZ])

def EulerToR(angles: List[float]):
    """Calculates the SO(3) rotation matrix given a set of Euler 
    angles.
    :param angles: List of three Euler angles around the x-, y-, and z-
    axis respectively.
    
    Example input:
    angles = [-1.57079633  3.14159265  0.        ]
    Output:
    
    NOTE: Credits to Gregory G. Slabaugh of Queen Mary University of 
    London for writing the equations online for this function """
    x = angles[0]
    y = angles[1]
    z = angles[2]
    Rx = np.array([[1, 0        , 0         ],
                   [0, np.cos(x), -np.sin(x)],
                   [0, np.sin(x), np.cos(x) ]])
    Ry = np.array([[np.cos(y),  0,  np.sin(y)],
                   [0,          1,  0        ],
                   [-np.sin(y), 0, np.cos(y)]])
    Rz = np.array([[np.cos(z), -np.sin(z), 0],
                   [np.sin(z),  np.cos(z), 0],
                   [0,          0,         1]])
    R = np.dot(np.dot(Rz, Ry), Rx)
    if not mr.TestIfSO3(R):
        raise SyntaxError("Computed R is not SO(3).")
    return np.round(R, 8)

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
        currMotor = currLim
    elif currMotor < -currLim:
        currMotor = -currLim
    return currMotor

def Curr2MSpeed(currMotor: float) -> float:
    """Converts current to PWM motor speed command.
    :param currMotor: Desired current in [A]
    :return mSpeed: PWM value in the range [0,255].
    NOTE: mSpeed should still be normalized to [mSpeedMin, mSpeedMax]!
    """
    linFactor = 255/2 #TODO: FIND ACCURATE LINFACTOR!
    mSpeed = currMotor *linFactor
    return mSpeed

def LimDamping(theta: np.ndarray, val: np.ndarray, 
               limList: List[List[float]], k: float=10) -> np.ndarray:
    """Ensures joint velocities are damped when coming close to / crossing"""
    n = len(limList)
    valNew = val.copy()
    for i in range(n):
        overshot = False
        damping = 1
        if abs(theta[i] - limList[i][0]) < \
        abs(theta[i] - limList[i][1]):
            limClose = limList[i][0]
            toLim = val[i] < 0
            if theta[i] < limClose:
                overshot = True
        else:
            limClose = limList[i][1]
            toLim = val[i] > 0
            if theta[i] > limClose:
                overshot = True
        if toLim:
            if overshot:
                damping = 0
            else:
                damping = abs(np.arctan(k*(theta[i]-limClose)))/(0.5*np.pi)
        valNew[i] *= damping
    return valNew




def saveToCSV(dataMat: np.ndarray, csvTitle: str, headers: str=None, 
              reset=False):
    """Saves an array of data to a CSV file, row by row.
    :param dataMat: 2-dimensional numpy array with data.
    :param csvTitle: Address of the CSV file relative to the current
                     file.
    :param headers: Optional, a list of headers on top of the CSV file.
    :param reset: Optional, truncate the CSV file (delete all data 
                  inside) before writing."""
    if reset:
        with open(csvTitle, "w+") as csvFile:
            csvFile.close() #Truncate the file
    with open(csvTitle, 'a') as csvFile:
        if headers:
            np.savetxt(csvFile, dataMat, fmt='%.5f', delimiter=',', 
                       header=headers)
        else:
            np.savetxt(csvFile, dataMat, fmt='%.5f', delimiter=',')

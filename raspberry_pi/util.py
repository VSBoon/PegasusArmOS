import numpy as np
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

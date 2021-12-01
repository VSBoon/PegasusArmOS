import os
import sys
#Find directory path of current file
current = os.path.dirname(os.path.realpath(__file__))
#Find directory path of parent folder and add to sys path
parent = os.path.dirname(current)
sys.path.append(parent)

import modern_robotics as mr
import numpy as np
from typing import List, Tuple
from util import ThetaInitGuess, screwsToMat, screwsToMat1D, screwsToMatT
from classes import IKAlgorithmError

#TODO: Implement IK, translate back to actual joint angles m4 & m5

def FKSpace(TsbHome: np.ndarray, spaceScrews: List[np.ndarray], 
           thetaList: List[float]) -> np.ndarray:
    """Computes the 4x4 SO(3) matrix of the end-effector configuration
    given the home configuration of the end-effector, the screw axes of
    the joints in the end-effector frame, and the list of joint angles.
    :param TsbHome: The SO(3) transformation matrix from the space 
                    frame {s} to the end-effector frame {b} when the 
                    robot is in its home configuration.
    :param spaceScrews: List of 6x1 screw vectors in the space frame, 
                        as per Definition 3.24 of the Modern Robotics 
                        book.
    :return TsbNew: The new end-effector configuration based on the 
                    list of joint angles.
    
    
    Example input:
    TsbHome = np.array([[1, 0, 0, 0.2],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0.10],
                        [0, 0, 0, 1]])
    spaceScrews = [np.array([0,0,1,0,0.1,0]), np.array([0,1,0,0.2,0,0])]
    thetaList = np.array([0.5*np.pi, 0.25*np.pi])
    Output:
    [[ 0.     -1.      0.      0.1   ]
    [ 0.7071  0.      0.7071  0.2414]
    [-0.7071  0.      0.7071  0.0414]
    [ 0.      0.      0.      1.    ]]
    """
    spaceMat = screwsToMat(spaceScrews)
    try:
        #Uses Product of Exponentials, see Modern Robotics, Chapter 4.1
        TsbNew = mr.FKinSpace(TsbHome, spaceMat, thetaList) 
    except IndexError as e:
        if "array is 1-dimensional" in str(e):
            spaceMat = screwsToMat1D(spaceScrews)
        elif "out of bounds" in str(e): #Non-transposed screw axes.
            spaceMat = screwsToMatT(spaceScrews)
        TsbNew = mr.FKinBody(TsbHome, spaceMat, thetaList)

    #TODO: Add checks to see if screw axes are normalized 
    #(i.e. either |rotational part| == 1, or |linear part| == 1)
    return TsbNew

def IKSpace(TsbHome: np.ndarray, TsbTarget: np.ndarray, spaceScrews: 
            List[np.ndarray], jointLimits: List[List[float]], nGuessJoints: 
            int=2, eRad: float=2e-2, eLin: float=2e-2) -> Tuple[np.ndarray, bool]:
    """Iteratively computes the list of joint angles of a robot that 
    result in the desired end-effector configuration. If multiple 
    solutions exist, it will return the solution with the smallest
    change in joint angles. Alternatively, if no exact solution exists,
    it returns the solution closest to it in terms of joint angles. 
    :param TsbHome: The SO(3) representation of the end- 
                    effector home configuration.
    :param TsbTarget: The SO(3) representation of the desired end-
                      effector configuration
    :param spaceScrews: A list of 6x1 vectors describing the screw axes
                        of each joint in the space frame, as per 
                        Definition 3.24 of the Modern Robotics book.
    :param jointLimits: A list of joint limits per joint, with the n-th
                        entry being a list of the lower- and upper 
                        joint limit of the n-th joint.
    :param nGuessJoints: The number of joints on which the joint angle 
                         guessing algorithm is used to generate an 
                         initial guess.
    :param eRad: The maximum allowed end-effector orientation error
                 with respect to the target configuration.
    :param eLin: The maximum allowed end-effector position error with
                 respect to the target configuration.
    :return thetaList: List of joint angles which result in an end-
                       effector configuration that is within the error 
                       limits eRad and eLin to the target configuration 
                       TsbTarget.
    :return success: Boolean that returns whether a valid solution was
                     found (True) or not (False).
    
    Example input:
    TsbCurrent = np.array([[1,0,0,0],
                           [0,1,0,5],
                           [0,0,1,3],
                           [0,0,0,1]])
    TsbTarget = np.array([[0,1,0,5],
                          [-1,0,0,0],
                          [0,0,1,3],
                          [0,0,0,1]])
    spaceScrews = [np.array([0,0,1,0,0,0]), np.array([-1,0,0,0,-2,0]), np.array([0,1,0,-3,0,0])]
    nGuessJoints = 1
    jointLimits = [[-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi]]
    eRad=1e-2
    eLin=1e-2
    Output: ([1.5708, 0.0, 0.0], True)
    """
    nJoints = len(spaceScrews)
    #Extract location from home transformation matrices
    psbHome = mr.TransToRp(TsbHome)[1]
    psbTarget = mr.TransToRp(TsbTarget)[1]
    majorScrewJoints = spaceScrews[0:nGuessJoints]
    thetaGuessList = ThetaInitGuess(psbHome, psbTarget, majorScrewJoints, 
                                    jointLimits)
    spaceMat = screwsToMat(spaceScrews)
    try:
        #Uses Product of Exponentials, see Modern Robotics, Chapter 4.1
        thetaList, success = mr.IKinSpace(spaceMat, TsbHome, TsbTarget, 
                                          thetaGuessList, eomg=eRad, ev=eLin)
    except IndexError as e:
        if "array is 1-dimensional" in str(e):
            spaceMat = screwsToMat1D(spaceScrews)
        elif "out of bounds" in str(e): #Non-transposed screw axes.
            spaceMat = screwsToMatT(spaceScrews)
        thetaList, success = mr.IKinSpace(spaceMat, TsbHome, TsbTarget, 
                                          thetaGuessList, eomg=eRad, ev=eLin)
    if not success:
        print("Best guess: ", thetaList)
        print("Leads to:\n", FKSpace(TsbHome, spaceScrews, thetaList))
        print("Versus original:\n", TsbTarget)
        raise IKAlgorithmError()
    #Normalize & check limits
    for i in range(nJoints):
        if thetaList[i] > np.pi:
            thetaList[i] = thetaList[i] % np.pi
        elif thetaList[i] < -np.pi:
            thetaList[i] = thetaList[i] % -np.pi
        if thetaList[i] > jointLimits[i][1]:
            thetaList[i] = jointLimits[i][1]
        elif thetaList[i] < jointLimits[i][0]:
            thetaList[i] = jointLimits[i][0]
    return (thetaList, success)
import numpy as np
import modern_robotics as mr
from typing import List, Union

class Joint():
    """A storage class combining all the information relevant to joint
    calculations."""
    def __init__(self, screwAx: np.ndarray, Tjoints: np.ndarray, inertiaMat: 
                 np.ndarray, linkMass: float, jointChild: 'Joint' = None, 
                 lims: List[float] = [-2*np.pi, 2*np.pi], 
                 inSpace: bool = True):
        """Constructor for Joint class.
        :param screwAx: A 6x1 screw axis, as per Definition 3.24 of the 
        Modern Robotics book
        :param Tjj: SO(3) representation of the transformation to go from 
        the current joint n to its successor in the chain, joint n+1
        :param inertiaMat: 3x3 inertia matrix of the link connection this
        joint, n, and joint n+1.
        :param inertiaMat: 3x3 inertia matrix of the link following the 
        joint n.
        :param linkMass: Mass of the link in kg.
        :param jointChild: Joint n+1, which is one link closer to the 
        end-effector than the current join, n.
        :param lims: The joint limits, in the order '[lower, upper]'.
        :param inSpace: Notes if the screw axis is in the space frame
        (True) or in the body frame (False)"""        
        self.screwAx = screwAx
        self.Tjoints = Tjoints
        self.Gb = np.zeros((6,6)) #6x6 spatial inertia matrix
        self.Gb[0:3,0:3] = inertiaMat
        self.Gb[3:6, 3:6] = np.diag(k=linkMass)
        self.jointChild = jointChild
        self.lims = lims
        self.inSpace = inSpace

class Robot():
    """Overarching robot class"""
    def __init__(self, jointList: List[Joint], TsbHome: np.ndarray):
        self.jointList = jointList
        self.screwAxes = [joint.screwAx for joint in jointList]
        self.lims = [joint.lims for joint in jointList]
        self.TsbHome = TsbHome

class SpaceTraj():
    """A storage class for all trajectory information of a trajectory 
    in the space frame"""
    def __init__(self, traj: List[np.ndarray], trajVel: List[float], 
                 trajAcc: List[float], timeList: List[float], dT: int):
        self.trajSpace = traj
        self.trajVel = trajVel
        self.trajAcc = trajAcc
        self.timeList = timeList
        self.dT = dT

class JointTraj():
    """A storage class for all trajectory information of a trajectory 
    in joint space"""
    def __init__(self, traj: List[float], trajVel: List[float], 
                trajAcc: List[float], timeList: List[float], dT: int):
        self.traj = traj
        self.trajVel = trajVel
        self.trajAcc = trajAcc
        self.timeList = timeList
        self.dT = dT


### ERROR CLASSES
class IKAlgorithmError(BaseException):
    """Custom error class for when the inverse kinematics algorithm is 
    unsuccesful.
    """
    def __init__(self, message: str = "The Inverse Kinematics algorithm " + 
                 "could not find a solution within its given iterations." + 
                 "This implies that either the configuration is outside" + 
                 "of the workspace of the robot, or the initial angle guess" +  
                 "was too far off."):
        self.message = message
    def __str__(self):
        return self.message

class DimensionError(BaseException):
    """Custom error class if dimensions of two inputs do not add up 
    while they should.
    """
    def __init__(self, message: str = ""):
        self.message = message
    def __str__(self):
        return self.message

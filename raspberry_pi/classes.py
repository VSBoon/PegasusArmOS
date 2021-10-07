import numpy as np
import modern_robotics as mr
from typing import List

class Link():
    def __init__(self, inertiaMat: np.ndarray, mass: float, joints: 
                 List['Joint'], Tsi: np.ndarray):
        """Constructer for Link class.
        :param inertiaMat: The 3x3 inertia matrix of the link with 
                           respect to the link frame. If the axes of 
                           the link frame coincide with the principle 
                           axes of inertia, this matrix simplifies to 
                           a diagonal matrix.
        :param mass: The mass of the link in kg.
        :param joints: The joints that the link connects, in the order 
                       [closer to base, closer to end-effector]. For 
                       the end-effector, the second joint is None.
        :param Tsi: The 4x4 SE(3) transformation matrix of the link
                    frame {i} in the space frame {s}.
        """
        self.Gi: np.ndarray = np.zeros((6,6)) #6x6 spatial inertia matrix
        self.Gi[0:3,0:3] = inertiaMat
        self.Gi[3:6, 3:6] = np.diag([mass]*3, k=0)
        self.joints: List['Joint'] = joints
        self.Tsi: np.ndarray = Tsi
    
    def __repr__(self):
        return f"Link(Gi: {self.Gi}\nJoints: {self.joints}\nTsi: {self.Tsi})"

class Joint():
    """A storage class combining all the information relevant to joint
    calculations."""
    def __init__(self, screwAx: np.ndarray, links: List[Link], 
                 jointChild: 'Joint' = None, lims: List[float] = 
                 [-2*np.pi, 2*np.pi], inSpace: bool = True):
        """Constructor for Joint class.
        :param screwAx: A 6x1 screw axis in the home configuration, per 
        Definition 3.24 of the Modern Robotics book.
        :param Tjoints: SO(3) representation of the transformation to 
                        go from the current joint n to its successor 
                        in the chain, joint n+1.
        :param links: Connecting links, in the order [prev, next].
        :param jointChild: Joint n+1, which is one link closer to the 
                            end-effector than the current join, n.
        :param lims: The joint limits, in the order [lower, upper].
        :param inSpace: Notes if the screw axis is in the space frame
                        (True) or in the body frame (False)."""        
        self.screwAx: np.ndarray = screwAx
        self.jointChild: Joint = jointChild
        self.lims: List[float] = lims
        self.links: List[Link] = links
        self.inSpace: bool = inSpace
    
    def __repr__(self):
        return f"Joint(screwAx: {self.screwAx}\njointChild:" +\
               f"{self.jointChild}\nlims: {self.lims}\nlinks: {self.links}" +\
               f"\ninSpace: {self.inSpace}"  

class Robot():
    """Overarching robot class"""
    def __init__(self, jointList: List[Joint], linkList: List[Link]):
        self.jointList: List[Joint] = jointList
        self.screwAxes: List[np.ndarray] = [joint.screwAx for joint in 
                                            jointList]
        self.limList: List[List[float]] = [joint.lims for joint in jointList]
        self.linkList: List[Link] = linkList
        self.GiList: List[np.ndarray] = [link.Gi for link in linkList]
        
        self.TllList: List[np.ndarray] = []
        for i in range(len(1,self.linkList)):
            #Transformation matrix of link {i} in link {i-1}
            Tll = np.dot(mr.TransInv(self.linkList[i-1].Tsi), 
                         self.linkList[i].Tsi) 
            self.TllList[i-1] = Tll
        self.TsbHome: np.ndarray = self.TllList[-1]
    
    def __repr__(self):
        return f"Robot:(jointList: {self.jointList}\nscrewAxes: " +\
               f"{self.screwAxes}\nlimList: {self.limList}\nlinkList: " +\
               f"{self.linkList}\n GiList: {self.GiList}\n TllList: " +\
               f"{self.TllList}\n TsbHome: {self.TsbHome}"

# class SpaceTraj():
#     """A storage class for all trajectory information of a trajectory 
#     in the space frame"""
#     def __init__(self, traj: List[np.ndarray], trajVel: List[float], 
#                  trajAcc: List[float], timeList: List[float], dT: int):
#         self.trajSpace = traj
#         self.trajVel = trajVel
#         self.trajAcc = trajAcc
#         self.timeList = timeList
#         self.dT = dT

# class JointTraj():
#     """A storage class for all trajectory information of a trajectory 
#     in joint space"""
#     def __init__(self, traj: List[float], trajVel: List[float], 
#                 trajAcc: List[float], timeList: List[float], dT: int):
#         self.traj = traj
#         self.trajVel = trajVel
#         self.trajAcc = trajAcc
#         self.timeList = timeList
#         self.dT = dT


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

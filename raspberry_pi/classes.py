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
    def __init__(self, screwAx: np.ndarray, links: List[Link], gearRatio: 
                 float, jointChild: 'Joint' = None, lims: List[float] = 
                 [-2*np.pi, 2*np.pi], inSpace: bool = True):
        """Constructor for Joint class.
        :param screwAx: A 6x1 screw axis in the home configuration, per 
        Definition 3.24 of the Modern Robotics book.
        :param links: Connecting links, in the order [prev, next].
        :param jointChild: Joint n+1, which is one link closer to the 
                            end-effector than the current join, n.
        :param gearRatio: The reduction ratio 1:gearRatio between the
                          motor shaft and joint axes.
        :param lims: The joint limits, in the order [lower, upper].
        :param inSpace: Notes if the screw axis is in the space frame
                        (True) or in the body frame (False)."""        
        self.screwAx: np.ndarray = screwAx
        self.jointChild: Joint = jointChild
        self.lims: List[float] = lims
        self.gearRatio = gearRatio
        self.links: List[Link] = links
        self.inSpace: bool = inSpace
    
    def __repr__(self):
        return f"Joint(screwAx: {self.screwAx}\njointChild:" +\
               f"{self.jointChild}\nlims: {self.lims}\nlinks: {self.links}" +\
               f"\ninSpace: {self.inSpace}"  

class Robot():
    """Overarching robot class"""
    def __init__(self, joints: List[Joint], links: List[Link]):
        self.joints: List[Joint] = joints
        self.screwAxes: List[np.ndarray] = [joint.screwAx for joint in 
                                            joints]
        self.limList: List[List[float]] = [joint.lims for joint in joints]
        self.links: List[Link] = links
        self.GiList: List[np.ndarray] = [link.Gi for link in links]
        
        self.TllList: List[np.ndarray] = []
        for i in range(len(1,self.links)):
            #Transformation matrix of link {i} in link {i-1}
            Tll = np.dot(mr.TransInv(self.links[i-1].Tsi), 
                         self.links[i].Tsi) 
            self.TllList[i-1] = Tll
        self.TsbHome: np.ndarray = self.TllList[-1]
    
    def __repr__(self):
        return f"Robot:(joints: {self.joints}\nscrewAxes: " +\
               f"{self.screwAxes}\nlimList: {self.limList}\nlinks: " +\
               f"{self.links}\n GiList: {self.GiList}\n TllList: " +\
               f"{self.TllList}\n TsbHome: {self.TsbHome}"

class SerialData():
    def __init__(self, lenData: int, cprList: List[int], desAngles: 
                 List[float], maxDeltaAngle: List[float], 
                 angleTol: List[float]) -> "SerialData":
        self.lenData = lenData
        self.cpr = cprList
        self.desAngle = desAngles
        self.totCount = [None for i in range(lenData)]
        self.rotDirCurr = [None for i in range(lenData)]
        self.currAngle = [0 for i in range(lenData)]
        self.prevAngle = [0 for i in range(lenData)]
        self.mSpeed = [None for i in range(lenData)]
        self.rotDirDes = [None for i in range(lenData)]
        self.dataOut = [None for i in range(lenData)]
        self.maxDeltaAngle = maxDeltaAngle
        self.angleTol = angleTol

    def ExtractVars(self, dataPacket: str, i: int):
        self.totCount[i], self.rotDirCurr[i] = dataPacket.split('|')
        self.totCount[i] = int(self.totCount[i])
        self.rotDirCurr[i] = int(self.rotDirCurr[i])
        self.prevAngle[i] = self.currAngle[i]
        self.currAngle[i] = (self.totCount[i]/self.cpr[i]) * 2*np.pi
    
    def CheckCommFault(self, i: int) -> bool:
        """Known issue: If commFault occurs because maxDeltaAngle is 
        too low, i.e. the encoder actually moved more than maxDelta-
        Angle in one step, then the motor ceases to run because 
        the condition will now always be True."""
        commFault = False
        if abs(self.prevAngle[i] - self.currAngle[i]) >= self.maxDeltaAngle[i]:
            commFault = True
            self.currAngle[i] = self.prevAngle[i]
            self.mSpeed[i] = 0
            self.dataOut[i] = f"{self.mSpeed[i]}|{self.rotDirDes[i]}"
        return commFault
    
    def CheckTolAng(self, i: int) -> bool:
        success = False
        if abs(self.currAngle[i] - self.desAngle[i]) <= self.angleTol[i]:
            success = True
            self.mSpeed[i] = 0
            self.dataOut[i] = f"{self.mSpeed[i]}|{self.rotDirDes[i]}"
        return success

    def GetDir(self, i: int):
        if self.currAngle[i] <= self.desAngle[i] and \
           self.rotDirCurr[i] != 0:
            self.rotDirDes[i] = 0
        elif self.currAngle[i] > self.desAngle[i] and self.rotDirCurr[i] != 1:
            self.rotDirDes[i] = 1
        else:
            self.rotDirDes[i] = self.rotDirCurr[i]
    
    def PControl1(self, i: int, mSpeedMax: int, mSpeedMin: int):
        angleErr = self.desAngle[i] - abs(self.currAngle[i])
        self.mSpeed[i] = int(mSpeedMin + (angleErr/np.pi)* \
                             (mSpeedMax - mSpeedMin))
        if self.mSpeed[i] > mSpeedMax:
            self.mSpeed[i] = mSpeedMax
        elif self.mSpeed[i] < mSpeedMin:
            self.mSpeed[i] = mSpeedMin

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

class InputError(BaseException):
    """Custom error class if the data received over serial 
    communication is invalid
    """
    def __init__(self, message: str = ""):
        self.message = message
    def __str__(self):
        return self.message

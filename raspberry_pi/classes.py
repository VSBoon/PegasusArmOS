import numpy as np
import modern_robotics as mr
from typing import List

class Link():
    def __init__(self, inertiaMat: np.ndarray, mass: float, prevLink: 'Link',
                 Tsi: np.ndarray):
        """Constructer for Link class.
        :param inertiaMat: The 3x3 inertia matrix of the link with 
                           respect to the link frame. If the axes of 
                           the link frame coincide with the principle 
                           axes of inertia, this matrix simplifies to 
                           a diagonal matrix.
        :param mass: The mass of the link in kg.
        :param prevLink: Previous link in the chain.
        :param Tsi: The 4x4 SE(3) transformation matrix of the link
                    frame {i} in the space frame {s}.
        """
        self.Gi: np.ndarray = np.zeros((6,6)) #6x6 spatial inertia matrix
        self.Gi[0:3,0:3] = inertiaMat
        self.Gi[3:6, 3:6] = np.diag([mass]*3, k=0)
        self.Tsi: np.ndarray = Tsi
        """Tii is the transformation matrix from current 
        link frame {i} to previous link frame {i-1}:"""
        if prevLink == None:
            self.Tii = self.Tsi
        else:
            self.Tii: np.ndarray = np.dot(mr.TransInv(prevLink.Tsi), Tsi)
    
    def __repr__(self):
        return f"Link(Gi: {self.Gi}\nJoints: {self.joints}\nTsi: {self.Tsi})"

class Joint():
    """A storage class combining all the information relevant to joint
    calculations."""
    def __init__(self, screwAx: np.ndarray, links: List[Link], gearRatio: 
                 float, cpr: int, lims: List[float] = [-2*np.pi, 2*np.pi], 
                 inSpace: bool = True):
        """Constructor for Joint class.
        :param screwAx: A 6x1 screw axis in the home configuration, per 
        Definition 3.24 of the Modern Robotics book.
        :param links: Connecting links, in the order [prev, next].
        :param gearRatio: The reduction ratio 1:gearRatio between the
                          motor shaft and joint axes.
        :param cpr: Counts per revolution of the encoder shaft.
        :param lims: The joint limits, in the order [lower, upper].
        :param inSpace: Notes if the screw axis is in the space frame
                        (True) or in the body frame (False).
        """        
        self.screwAx: np.ndarray = screwAx
        self.lims: List[float] = lims
        self.gearRatio = gearRatio
        self.cpr = cpr
        self.enc2Theta = 1/(cpr*gearRatio) * 2*np.pi
        self.links: List[Link] = links
        self.inSpace: bool = inSpace
    
    def __repr__(self):
        return f"Joint(screwAx: {self.screwAx}\njointChild:" +\
               f"{self.jointChild}\nlims: {self.lims}\nlinks: {self.links}" +\
               f"\ninSpace: {self.inSpace}"  

class Robot():
    """Overarching robot class, containing all joints & links 
    described using their respective class instances.
    """
    def __init__(self, joints: List[Joint], links: List[Link]):
        """ Constructor for Robot class.
        :param joints: A list of Joint objects, with the joints 
                       being in ascending order from the joint 
                       closest to the robot's connection with the 
                       fixed world.
        :param links: A list of Link objects, with the links being 
                      in ascending order from the link closest to 
                      the robot's connection to the fixed world.
        """
        self.joints: List[Joint] = joints
        self.screwAxes: List[np.ndarray] = [joint.screwAx for joint in 
                                            joints]
        self.limList: List[List[float]] = [joint.lims for joint in joints]
        self.links: List[Link] = links
        self.GiList: List[np.ndarray] = [link.Gi for link in links]
        
        self.TllList: List[np.ndarray] = [link.Tii for link in links]
        self.TsbHome: np.ndarray = self.TllList[-1]
    
    def __repr__(self):
        return f"Robot:(joints: {self.joints}\nscrewAxes: " +\
               f"{self.screwAxes}\nlimList: {self.limList}\nlinks: " +\
               f"{self.links}\n GiList: {self.GiList}\n TllList: " +\
               f"{self.TllList}\n TsbHome: {self.TsbHome}"

class SerialData():
    """Container class containing all relevant information and functions
    for parsing and acting on data received over serial communication."""
    def __init__(self, lenData: int, desAngles: 
                 List[float], maxDeltaAngle: List[float], 
                 angleTol: List[float], joints: List[Joint]) -> "SerialData":
        """Constructor for SerialData class.
        :param lenData: The expected number of data packets (equal to
                        number of motor units).
        :param desAngles: List of desired angles for each motor, in 
                          radians (for position control).
        :param maxDeltaAngle: Maximum change in each joint angle 
                              between two received data packets (anti-
                              corruption check).
        :param joints: List of all Joint instances of the robot.
        :param angleTol: Tolerance of each desired joint angle in 
                         radians.
        """
        self.lenData = lenData
        self.desAngle = desAngles
        self.joints = joints
        if len(joints) != lenData:
            msg = "Number of joints does not match length of data: "+\
                  f"({len(joints)}, {lenData})"
            raise DimensionError(msg)
        self.totCount = [0 for i in range(lenData)]
        self.rotDirCurr = [None for i in range(lenData)]
        self.current = [None for i in range(lenData)]
        self.homing = [None for i in range(lenData)]
        self.currAngle = [0 for i in range(lenData)]
        self.prevAngle = [0 for i in range(lenData)]
        self.mSpeed = [0 for i in range(lenData)]
        self.rotDirDes = [None for i in range(lenData)]
        self.dataOut = [None for i in range(lenData)]
        self.maxDeltaAngle = maxDeltaAngle
        self.angleTol = angleTol

    def ExtractVars(self, dataPacket: List[str]):
        """Extracts & translates information in each datapacket.
        :param dataPacket: A string of the form 'totCount|rotDirCurr',
                           where totCount is an integer and rotDirCurr
                           a boolean (0 or 1).
        
        Example input:
        dataPacket = '1234|0'
        """
        for i in range(self.lenData):
            args = dataPacket[i].split('|')
            if len(args) == 4:
                self.current[i], self.homing[i] = args[2:4]
                self.current[i] = int(self.current[i])
                #TODO: Add current[i] volt -> amp conversion
                self.homing[i] = int(self.homing[i])
            self.totCount[i], self.rotDirCurr[i] = args[0:2]
            self.totCount[i] = int(self.totCount[i])
            self.rotDirCurr[i] = int(self.rotDirCurr[i])
            self.prevAngle[i] = self.currAngle[i]
            self.currAngle[i] = self.totCount[i] * self.joints[i].enc2Theta
            if i == 3 or i == 4:
                """Due to the unique mechanics of the robot, the 
                encoder only measures the absolute angle of 3 or 4, 
                not relative to 2 or 3, respectively."""
                self.currAngle[i] -= self.currAngle[i-1] 


    def CheckCommFault(self) -> bool:
        """Checks if data got corrupted using a maximum achievable 
        change in angle between two timesteps.
        :return commFault: Boolean indicating if new angle is 
                           reasonable (False) or not (True).
        
        Known issue: If commFault occurs because maxDeltaAngle is 
        too low, i.e. the encoder actually moved more than maxDelta-
        Angle in one step, then the motor ceases to run because 
        the condition will now always be True.
        """
        commFault = [False for i in range(self.lenData)]
        for i in range(self.lenData):
            if abs(self.prevAngle[i] - self.currAngle[i]) >= self.maxDeltaAngle[i]:
                print(f"prev: {self.prevAngle[i]}, curr: {self.currAngle[i]}, {self.maxDeltaAngle[i]}") #DEBUG
                commFault[i] = True
                self.currAngle[i] = self.prevAngle[i]
                self.mSpeed[i] = 0
                self.dataOut[i] = f"{self.mSpeed[i]}|{self.rotDirDes[i]}"
        return commFault
    
    def CheckTolAng(self) -> bool:
        """Determines if the desired angle has been reached within the 
        given tolerance, and if the desired angle is reachable.
        If the desired angle is unreachable, the closest angle is chosen.
        :return success: Indicates if the angle is within the 
        tolerance of the goal angle (True) or not (False).
        """
        success = [False for i in range(self.lenData)]
        for i in range(self.lenData):
            if self.desAngle[i] < self.joints[i].lims[0]:
                self.desAngle[i] = self.joints[i].lims[0]
            elif self.desAngle[i] > self.joints[i].lims[1]:
                self.desAngle[i] = self.joints[i].lims[1] 
            if abs(self.currAngle[i] - self.desAngle[i]) <= self.angleTol[i]:
                success[i] = True
                self.mSpeed[i] = 0
                self.dataOut[i] = f"{self.mSpeed[i]}|{self.rotDirDes[i]}"
        return success

    def GetDir(self):
        """Gives the desired direction of rotation
        """
        for i in range(self.lenData):
            if self.currAngle[i] <= self.desAngle[i] and \
            self.rotDirCurr[i] != 0:
                self.rotDirDes[i] = 0
            elif self.currAngle[i] > self.desAngle[i] and self.rotDirCurr[i] != 1:
                self.rotDirDes[i] = 1
            else:
                self.rotDirDes[i] = self.rotDirCurr[i]
    
    def PControl1(self, i: int, mSpeedMax: int, mSpeedMin: int):
        """Gives motor speed commands proportional to the angle error.
        :param i: Current iteration number.
        :param mSpeedMax: Maximum rotational speed, 
                          portrayed in PWM [0, 255].
        :param mSpeedMin: Minimum rotational speed, 
                          portrayed in PWM [0, 255]."""
        angleErr = abs(self.desAngle[i] - self.currAngle[i])
        self.mSpeed[i] = int(mSpeedMin + (angleErr/(10*np.pi))* \
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
    communication is invalid.
    """
    def __init__(self, message: str = ""):
        self.message = message
    def __str__(self):
        return self.message

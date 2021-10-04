import numpy as np
from typing import List

class Joint():
    """A storage class combining all the information relevant to joint
    calculations."""
    def __init__(self, screwAx: np.ndarray, Tjoints: np.ndarray, inertiaMat: 
        np.ndarray, linkMass: float, lims: List[float] = [-2*np.pi, 2*np.pi], 
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
        :param lims: The joint limits, in the order '[lower, upper]'.
        :param inSpace: Notes if the screw axis is in the space frame
        (True) or in the body frame (False)"""        
        self.screwAx = screwAx
        self.Tjoints = Tjoints
        self.Gb = np.zeros((6,6)) #6x6 spatial inertia matrix
        self.Gb[0:3,0:3] = inertiaMat
        self.Gb[3:6, 3:6] = np.diag(k=linkMass)
        self.lims = lims
        self.inSpace = inSpace
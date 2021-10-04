from kinematic_funcs import FKSpace, IKSpace
from util import IKAlgorithmError, ThetaInitGuess, Joint
import modern_robotics as mr
import numpy as np
import math

S1 = np.array([[0, 0, 1, 0, -0.41325, 0]]).T
S2 = np.array([[0, 1, 0, 0, 0, -0.4469]]).T
S3 = np.array([[0, 1, 0, 0, 0, -0.382]]).T
S4 = np.array([[0, 1, 0, 0, 0, 0.153]]).T
S5 = np.array([[0, 0, 1, 0, -0.153, 0]]).T
SList = np.hstack((S1,S2,S3,S4,S5))
TsbCurrent = np.array([[1, 0, 0, 0.41325],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.10], #p_sb z component to be changed!!!
                    [0, 0, 0, 1]])
thetaList = np.array([0.5*np.pi,0,0,0,0])
TsbExpected = mr.FKinSpace(TsbCurrent, SList, thetaList)

def test_FKSpaceNormal():
    """Test if the function works properly under expected 
    circumstances"""
    TsbNew = FKSpace(TsbCurrent, (S1,S2,S3,S4,S5), thetaList)
    assert TsbNew.all() == TsbExpected.all()

def test_FKSpaceNoTranspose():
    """Test to ensure that if a user does not transpose the screw axes, 
    the program catches this."""
    bodyScrewsTest = (S1.T, S2.T, S3.T, S4.T, S5.T)
    TsbNew = FKSpace(TsbCurrent, bodyScrewsTest, thetaList)
    assert TsbNew.all() == TsbExpected.all()

def test_FKSpace1DArray():
    """Test to ensure that a 1D array (no second set of square 
    brackets)gets correctly interpreted by the FKSpace function."""
    S1Test = np.array([0, 0, 1, 0, -0.41325, 0]).T
    S2Test = np.array([0, 1, 0, 0, 0, -0.4469]).T
    S3Test = np.array([0, 1, 0, 0, 0, -0.382]).T
    S4Test = np.array([0, 1, 0, 0, 0, 0.153]).T
    S5Test = np.array([0, 0, 1, 0, -0.153, 0]).T
    bodyScrewsTest = (S1Test, S2Test, S3Test, S4Test, S5Test)
    TsbNew = FKSpace(TsbCurrent, bodyScrewsTest, thetaList)
    assert TsbNew.all() == TsbExpected.all()

def test_ThetaGuessNormal():
    """Test to ensure that the appropriate joint angles are
    generated based on the intended algorithm"""
    psbCurrent = np.array([1,0,0])
    psbTarget = np.array([2,0,0])
    majorScrewJoints = [np.array([0,1,0,-1,0,0])]
    thetaExpectedList = [-0.32, 0, 0, 0, 0]
    jointLimits = [[-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi]]
    thetaGuessList = ThetaInitGuess(psbCurrent, psbTarget, majorScrewJoints, 
                                    jointLimits)
    assert np.isclose(thetaExpectedList[0], thetaGuessList[0], rtol = 0.01) 

def test_ThetaGuessNoJoints():
    """Test to check if inserting no major screw joints returns a 
    list of all zeros"""
    psbCurrent = np.array([1,0,0])
    psbTarget = np.array([2,0,0])
    majorScrewJoints = []
    thetaExpectedList = [0, 0, 0, 0, 0]
    jointLimits = [[-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi]]
    thetaGuessList = ThetaInitGuess(psbCurrent, psbTarget, majorScrewJoints, 
                                    jointLimits)
    assert thetaExpectedList == thetaGuessList

def test_ThetaGuessNormalizing():
    """Checks if the normalization of the joint angles to the interval
    (-pi, pi] works."""
    psbCurrent = np.array([1,0,0])
    psbTarget = np.array([2,0,0])
    majorScrewJoints1 = [np.array([0,1,0,-0.01,0,-1.5])]
    jointLimits = [[-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi]]
    thetaGuessList1 = ThetaInitGuess(psbCurrent, psbTarget, majorScrewJoints1, 
                                    jointLimits)
    assert -np.pi < thetaGuessList1[0] <= np.pi 
    majorScrewJoints2 = [np.array([0,1,0,-0.01,0,-1.5])]
    thetaGuessList2 = ThetaInitGuess(psbCurrent, psbTarget, majorScrewJoints2, 
                                    jointLimits)
    assert -np.pi < thetaGuessList2[0] <= np.pi 

def test_IKSpaceNormal():
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
    thetaList, success = IKSpace(TsbCurrent, TsbTarget, spaceScrews, jointLimits, nGuessJoints, eRad, eLin)
    if not success:
        print(f"thetaList: {thetaList} \nspaceScrews: {spaceScrews}\n TsbCurrent: {TsbCurrent}")
        print(FKSpace(TsbCurrent, spaceScrews, thetaList))
    assert success

def test_IKSpaceJointClass():
    """Checks if IKSpace() works with the Joint class"""
    TsbCurrent = np.array([[1,0,0,0],
                           [0,1,0,5],
                           [0,0,1,3],
                           [0,0,0,1]])
    TsbTarget = np.array([[0,1,0,5],
                          [-1,0,0,0],
                          [0,0,1,3],
                          [0,0,0,1]])
    J1 = Joint(screwAx = np.array([0,0,1,0,0,0]), lims = [-np.pi, np.pi])
    J2 = Joint(screwAx= np.array([-1,0,0,0,-2,0]), lims= [-np.pi, np.pi])
    J3 = Joint(screwAx= np.array([0,1,0,-3,0,0]), lims= [-np.pi, np.pi])
    spaceScrews = [J1.screwAx, J2.screwAx, J3.screwAx]
    jointLimits = [J1.lims, J2.lims, J3.lims]
    nGuessJoints = 1
    eRad = 1e-2
    eLin = 1e-2
    thetaList, success = IKSpace(TsbCurrent, TsbTarget, spaceScrews, jointLimits, nGuessJoints, eRad, eLin)
    if not success:
        print(f"thetaList: {thetaList} \nspaceScrews: {spaceScrews}\n TsbCurrent: {TsbCurrent}")
        print(FKSpace(TsbCurrent, spaceScrews, thetaList))
    assert success

def test_IKSpaceOutOfWorkSpace():
    """Checks if IKSpace throws an error when the target configuration
    is outside of the workspace"""
    TsbCurrent = np.array([[1,0,0,0],
                           [0,1,0,5],
                           [0,0,1,3],
                           [0,0,0,1]])
    TsbTarget = np.array([[0,1,0,6],
                          [-1,0,0,0],
                          [0,0,1,3],
                          [0,0,0,1]])
    J1 = Joint(screwAx = np.array([0,0,1,0,0,0]), lims = [-np.pi, np.pi])
    J2 = Joint(screwAx= np.array([-1,0,0,0,-2,0]), lims= [-np.pi, np.pi])
    J3 = Joint(screwAx= np.array([0,1,0,-3,0,0]), lims= [-np.pi, np.pi])
    spaceScrews = [J1.screwAx, J2.screwAx, J3.screwAx]
    jointLimits = [J1.lims, J2.lims, J3.lims]
    nGuessJoints = 1
    eRad = 1e-2
    eLin = 1e-2
    try:
        thetaList, success = IKSpace(TsbCurrent, TsbTarget, spaceScrews, jointLimits, nGuessJoints, eRad, eLin)
    except IKAlgorithmError:
        assert True
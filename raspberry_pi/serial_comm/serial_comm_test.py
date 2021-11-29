import os
import sys
#Find directory path of current file
current = os.path.dirname(os.path.realpath(__file__))
#Find directory path of parent folder and add to sys path
parent = os.path.dirname(current)
sys.path.append(parent)
import numpy as np
import serial
import time
from serial_comm import FindSerial, StartComms, GetComms, SReadAndParse, SetPointControl1
from classes import SerialData, InputError, Joint, Link, Robot

"""Trying to do multiple test cases for FindSerial() and GetComms() is 
difficult, since all are run in one go, and the microcontroller cannot 
be connected and disconnected simultaneously. Therefore, uncomment the 
test that can be done and iterate through them.
TODO: Test with Teensy!
"""
# All tested successfully on Windows PC with Arduino 19-10-2021
# def test_FindSerialNoPort():
#     """Test to see if an IOError gets raised when no 
#     microController is connected.
#     """
#     try:
#         FindSerial()
#     except IOError:
#         assert True


# def test_FindSerial1Windows():
#     """Check if the serial port is found on a Windows PC: 
#     edit knownPort before testing!
#     """
#     knownPort = 'COM11'
#     port = FindSerial()[0]
#     assert port == knownPort

# def test_FindSerial1Linux():
#     """Check if the serial port is found on a Linux PC: 
#     edit knownPort before testing!
#     """
#     knownPort = '/dev/ttyACM0'
#     port = FindSerial()[0]
#     assert port == knownPort

# def test_FindSerial2():
#     """Check if the user is warned about multiple controllers being 
#     connected.
#     """
#     port, warning = FindSerial()
#     assert warning

def test_StartCommsNormal():
    """Check if StartComms opens the serial port"""
    comPort = FindSerial()[0]
    localMu = StartComms(comPort)
    assert localMu.isOpen()

def test_StartCommsWrongPort():
    """Checks if StartComms throws an error when a non-existing
    serial port is used as input argument."""
    comPort = 'RandomText'
    try:
        localMu = StartComms(comPort)
    except serial.serialutil.SerialException:
        assert True
        return None
    assert False

# All tested successfully on Windows PC with Arduino 19-10-2021
# def test_GetCommsNormal():
#     """See if function can obtain input data sent over serial.
#     """
#     port = FindSerial()[0]
#     localMu = StartComms(port)
#     dataIn = GetComms(localMu)

# def test_GetCommsWrongStartMarker():
#     """Check if appropriate error is thrown when an invalid 
#     start marker is used.
#     NOTE: Ensure that serial is sending invalid start marker data"""
#     port = FindSerial()[0]
#     localMu = StartComms(port)
#     try:
#         while not localMu.inWaiting():
#             continue
#         dataIn = GetComms(localMu)
#     except InputError as e:
#         print(e.message)
#         if "Invalid start marker" == e.message:
#             assert True
#             return None
#     assert False

# def test_GetCommsNoInput():
#     """Check if InputError is thrown if no string is sent.
#     NOTE: Make sure serial is not sending any infromation.
#     """
#     port = FindSerial()[0]
#     localMu = StartComms(port)
#     try:
#         dataIn = GetComms(localMu)
#     except InputError as e:
#         if "No string read." == e.message:
#             assert True
#             return None
#     assert False

# def test_SReadAndParseWrongLen():
#     """Check if control is disabled when data of the wrong length is 
#     sent. NOTE: Make sure local microcontroller sends different data 
#     length."""
#     port, warning = FindSerial()
#     baudRate = 115200
#     lenData = 6
#     cprList = [4320 for i in range(lenData)]
#     desAngles = [3*np.pi for i in range(lenData)]
#     maxDeltaAngles = [np.pi for i in range(lenData)]
#     tolAngle = [0.04*np.pi for i in range(lenData)]
#     SPData = SerialData(lenData, cprList, desAngles, maxDeltaAngles, tolAngle)
#     dtComm = 0.005
#     localMu = StartComms(port, baudRate)
#     encAlg = "utf-8"
#     lastCheckOld = time.time() - dtComm #Ensure data is parsed
#     lastCheck, controlBool = SReadAndParse(SPData, lastCheckOld, dtComm, localMu, encAlg)
#     if lastCheck != lastCheckOld:
#         assert not controlBool
#         assert SPData.totCount[0] == 0 #Data inputs 1000, should not be taken
#         return None
#     assert False

def test_SReadAndParseNormal():
    """Check if control is disabled when data of the wrong length is 
    sent. NOTE: Make sure local microcontroller sends different data 
    length."""
    port, warning = FindSerial()
    baudRate = 115200
    lenData = 5 #Make sure local microcontroller sends data of same length.
    cprList = [4320 for i in range(lenData)]
    desAngles = [3*np.pi for i in range(lenData)]
    maxDeltaAngles = [np.pi for i in range(lenData)]
    tolAngle = [0.04*np.pi for i in range(lenData)]

    ###ROBOT INITIALISATION: NECESSARY FOR SPDATA() INSTANCE###
    #Inertia matrices
    iMat0 = np.diag([0.03947, 0.03362, 0.04886])
    iMat1 = np.diag([0.00393, 0.00237, 0.00172])
    iMat2 = np.diag([0.00294, 0.00210, 0.0029])
    iMat34 = np.diag([0.00041, 0.00348, 0.00364])
    massList = [5.13, 0.507, 0.420, 0.952, 0.952]
    #Transformation matrices from CoM of links with principle axes of
    #inertia to the space frame (Tsi):
    Tsi0 = np.array([[ 0.397, 0.838,-0.375, 0.0284],
                    [-0.909, 0.416,-0.033,-0.0413],
                    [ 0.129, 0.354, 0.926, 0.0522],
                    [0    , 0     , 0    , 1     ]])
    Tsi1 = np.array([[ 0.000, 0.455, 0.890, 0.0015],
                    [ 0.001, 0.890,-0.455, 0.0026],
                    [-1.000, 0.007, 0.001, 0.0039],
                    [0    , 0     , 0    , 1     ]])
    Tsi2 = np.array([[-0.003, 0.082, 0.997, 0.0009],
                    [ 0.001, 0.997,-0.082, 0.0021],
                    [-1.000, 0.001,-0.003, 0.0029],
                    [0    , 0     , 0    , 1     ]])
    Tsi34 = np.array([[-0.999, 0.000, -0.035, 0.0076],
                    [0.000, -1.000, -0.000,-0.0159],
                    [-0.035, -0.000, 0.999, 0.5840],
                    [0    , 0     , 0    , 1     ]])
    TsbHome = np.array([[1,0,0, 0.1474],
                        [0,1,0,-0.0168],
                        [0,0,1, 0.5853],
                        [0,0,0, 1     ]])
    #Screw axes in the Space Frame {s}
    S0 = np.array([0,0,1,0,0,0])
    S1 = np.array([0,1,0,-0.125,0,0.0035])
    S2 = np.array([0,1,0,-0.355,0,0.0035])
    S3 = np.array([0,1,0,-0.585,0,0.0030])
    S4 = np.array([1,0,0,0,0.585,0.016])
    lims0 = [-0.945*np.pi, 0.945*np.pi] #+/- 170 deg
    lims1 = [-0.25*np.pi, 0.5*np.pi] #-45 deg, + 90 deg
    lims2 = [-np.pi, 0.25*np.pi] #-180 deg, + 45 deg.
    lims3 = [-np.pi, 0.25*np.pi] #-180 deg, +45 deg.
    lims4 = [-np.pi, np.pi] #+/- 180 deg.
    gearRatioList = [19.7*50, 19.7*50, (65.5*20)/9, (65.5*20)/9, (127.7*32)/9]
    cpr = 512
    L0 = Link(iMat0, massList[0], None, Tsi0)
    L1 = Link(iMat1, massList[1], L0, Tsi1)
    L2 = Link(iMat2, massList[2], L1, Tsi2)
    L34 = Link(iMat34, massList[3], L2, Tsi34)
    links = [L0, L1, L2, L34, L34]
    km = [22.7*10**(-3), 22.7*10**(-3), 22.7*10**(-3), 22.7*10**(-3),
            22.7*10**(-3), 22.7*10**(-3), 9.2*10**(-3)] 
    J0 = Joint(S0, [None, L0], gearRatioList[0], km[0], cpr, lims0)
    J1 = Joint(S1, [L0, L1], gearRatioList[1], km[1], cpr, lims1)
    J2 = Joint(S2, [L1, L2], gearRatioList[2], km[2], cpr, lims2)
    J3 = Joint(S3, [L2,L34], gearRatioList[3], km[3], cpr, lims3)
    J4 = Joint(S4, [L2,L34], gearRatioList[4], km[4], cpr, lims4)
    joints = [J0, J1, J2, J3, J4]
    robot = Robot(joints, links, TsbHome)
    ###END OF ROBOT INITIALISATION###

    SPData = SerialData(lenData, desAngles, maxDeltaAngles, tolAngle, joints)
    dtComm = 0.005
    localMu = StartComms(port, baudRate)
    encAlg = "utf-8"
    lastCheckOld = time.time() - dtComm #Ensure data is parsed
    while localMu.inWaiting() == 0:
        continue
    lastCheck, controlBool = SReadAndParse(SPData, lastCheckOld, dtComm, localMu, encAlg)
    if lastCheck != lastCheckOld:
        assert controlBool
        assert SPData.totCount[0] == 1000 #serial input is 1000, should be taken
        return None
    assert False
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
from classes import SerialData, InputError, Joint, Link

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
    iMat0 = np.diag([0.03584238, 0.02950513, 0.04859042])
    iMat1 = np.diag([0.00393345, 0.00236823, 0.00171701])
    iMat2 = np.diag([0.0092912, 0.00210452, 0.0029424])
    iMat34 = np.diag([0.00363966, 0.00347835, 0.00041124])
    massList = [4.99, 0.507, 0.420, 0.952, 0.952]
    #Transformation matrices from CoM of links with principle axes of
    #inertia to the space frame (Tsi):
    Tsi0 = np.array([[0.3804 ,-0.9215,0.0786 ,-0.0103],
                     [0.8774 ,0.3864 ,0.2843 ,-0.0292],
                     [-0.2924,-0.0392,0.9555 ,0.0642 ],
                     [0      ,0      ,0      ,1      ]])
    Tsi1 = np.array([[-0.0002,0.0008 ,-1.0000,0.0350 ],
                     [0.4553 ,0.8903 ,0.0007 ,-0.0083],
                     [0.8903 ,-0.4553,-0.0006,0.1666 ],
                     [0      ,0      ,0      ,1      ]])
    Tsi2 = np.array([[-0.9966,0.0819 ,-0.0003,0.0814 ],
                     [-0.0819,-0.9966,0.0008 ,-0.0083],
                     [-0.0002,0.0008 ,1.0000 ,0.3550 ],
                     [0      ,0      ,0      ,1      ]])
    Tsi34 = np.array([[-0.0326,0.0002 ,-0.9995,0.2640 ],
                      [-0.0004,1.0000 ,0.0002 ,-0.0059],
                      [-0.9995,0.0004 ,-0.0326,0.3504 ],
                      [0      ,0      ,0      ,1      ]])
    #Screw axes in the Space Frame {s}
    S0 = np.array([0,0,1,0,0,0])
    S1 = np.array([0,1,0,-0.0035,0,0.126])
    S2 = np.array([0,1,0,-0.0035,0,0.335])
    S3 = np.array([0,1,0,-0.234,0,0.355])
    S4 = np.array([0,0,1,-0.234,-0.016,0])
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
    J0 = Joint(S0, [None, L0], gearRatioList[0], cpr, lims0)
    J1 = Joint(S1, [L0, L1], gearRatioList[1], cpr, lims1)
    J2 = Joint(S2, [L1, L2], gearRatioList[2], cpr, lims2)
    J3 = Joint(S3, [L2,34], gearRatioList[3], cpr, lims3)
    J4 = Joint(S4, [L2,L34], gearRatioList[4], cpr, lims4)
    joints = [J0, J1, J2, J3, J4]
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
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
from classes import SerialData
from robot_init import robot

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
    SPData = SerialData(lenData, desAngles, maxDeltaAngles, tolAngle, robot.joints)
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
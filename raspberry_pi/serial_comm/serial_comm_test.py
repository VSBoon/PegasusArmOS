import os
import sys
#Find directory path of current file
current = os.path.dirname(os.path.realpath(__file__))
#Find directory path of parent folder and add to sys path
parent = os.path.dirname(current)
sys.path.append(parent)

from serial_comm import FindSerial, StartComms, GetComms, SReadAndParse, SetPointControl1
from classes import SerialData, InputError

"""Trying to do multiple test cases for FindSerial() is difficult, 
since all are run in one go, and the microcontroller cannot be 
connected and disconnected simultaneously. Therefore, uncomment the 
test that can be done and iterate through them.
TODO: Test with Teensy!
"""

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

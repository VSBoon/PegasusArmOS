import os
import sys
#Find directory path of current file
current = os.path.dirname(os.path.realpath(__file__))
#Find directory path of parent folder and add to sys path
parent = os.path.dirname(current)
sys.path.append(parent)

from classes import SerialData, InputError, Joint, Link, Robot
from typing import List, Tuple
from trajectory_generation.traj_gen import TrajGen, TrajDerivatives
import serial
import serial.tools.list_ports
import time
import numpy as np

#TODO: Docstrings, examples, & tests.
def FindSerial(askInput=False) -> str:
    """Finds the Serial port to which the Arduino / Teensy is connected
    :param askInput: If multiple connections are found, 
                     ask the user to choose.
    :return port: The string representation of the port.
    :return warning: Boolean indicating a warning has been printed."""
    warning = False
    comports = serial.tools.list_ports.comports()
    port = []
    for p in comports:
        if p.manufacturer:
            if 'Arduino' in p.manufacturer or 'Teensy' in p.manufacturer:
                port.append(p.device)
    if not port:
        raise IOError("No microcontroller found!")
    elif len(port) > 1:
        print("Multiple micrcontrollers connected: ")
        if askInput:
            portIndex = input("Please choose the index (starting from 0)" + 
                              f"from the following list: {port}")
            try:
                port = port[portIndex]
            except:
                print("Invalid index, taking first port.")
                port = port[0]
        else:
            print(f"Taking first port ({port[0]})")
            port = port[0]
        warning = True
    else:
        port = port[0]
    return port, warning

def StartComms(comPort: str, baudRate: int = 115200) -> serial.Serial:
    """Intantiates a serial connection with a microcontroller over USB.
    :param comPort: The address of the communication port to which the 
                    local microcontroller is connected.
    :param baudRate: Maximum number of bytes being sent over serial
                     per second. Has to equate the set baudrate of the
                     local microcontroller.
    :return localMu: A Serial-class instance.

    Example input:
    comPort = "/dev/ttyAMA0"
    baudRate = 115200
    Output:
    localMu: serial.Serial(comPort, baudRate)
    """
    localMu = serial.Serial(comPort, baudRate, timeout=1)
    time.sleep(0.05) #wait for serial to open
    if localMu.isOpen():
        print(f"{localMu.port} connected!")
    return localMu

def GetComms(localMu: serial.Serial, encAlg: str = "utf-8") -> str:
    """Reads and decodes incoming byte data over a serial connection.
    :param localMu: A serial.Serial() instance representing the serial
                    communication to the local microcontroller.
    :param encAlg: String containing the algorithm used to encode and 
                   decode the bytes sent over serial.
    :return dataIn: String of decoded bytes available in the serial buffer 
                    until the representation of the newline character.
    Example input:
    localMu = StartComms("COM9", baudRate=115200)
    encalg = "utf-8"
    Output:
    "This is an example of information sent over the serial port."
    """
    dataIn = localMu.readline().decode(encAlg).rstrip() #remove decoding
    if(not dataIn):
        raise InputError("No string read.")
    elif(dataIn[0] != "["):
        raise InputError("Invalid start marker")
    return dataIn

def SReadAndParse(SPData: SerialData, lastCheckOld: float, dtComm: float,
                  localMu: serial.Serial, encAlg: str = "utf-8") \
                  -> Tuple[float, bool]:
    """Serial read function which parses data into SerialData object.
    :param SPData: SerialData instance, stores & parses serial data.
    :param lastCheckOld: time.time() value representing last time 
                         data was parsed.
    :param dtComm: Desired minimal time between communication loops.
    :param localMu: serial.Serial() instance representing the serial
                    communication with the local microcontroller.
    :param encAlg: Algorithm used to encode data into bytes for serial.
    :return lastCheck: time.time() value representing last time
                       data was parsed.
    :return controlBool: Boolean indicating if control can be done on 
                         new data.
    
    Example input:
    baudRate = 115200
    lenData = 6 #Number of motors
    cprList = [4320 for i in range(lenData)]
    desAngles = [3*np.pi for i in range(lenData)]
    maxDeltaAngles = [np.pi for i in range(lenData)]
    tolAngle = [0.04*np.pi for i in range(lenData)]
    SPData = SerialData(lenData, cprList, desAngles, maxDeltaAngles, tolAngle)
    dtComm = 0.005
    localMu = StartComms("COM9", baudRate)
    encAlg = "utf-8"
    lastCheckOld = time.time()

    Example output:
    1634299822.1247501, True
    """
    controlBool = True
    lastCheck = time.time()
    elapsedTime = time.time() - lastCheckOld
    if elapsedTime >= dtComm:
        if localMu.inWaiting() == 0:
            return lastCheck, controlBool
        elif localMu.inWaiting() > 0:
            try:
                dataIn = GetComms(localMu, encAlg)
            except InputError as e:
                controlBool = False
                print(str(e))
                localMu.reset_input_buffer()
                return lastCheck, controlBool
            except UnicodeDecodeError as e:
                controlBool = False
                localMu.reset_input_buffer()
                return lastCheck, controlBool
            dataPacket = dataIn[1:-1].split('][')
            if len(dataPacket) != SPData.lenData: #flush & retry
                controlBool = False
                localMu.reset_input_buffer()
                return lastCheck, controlBool
            #Expected form dataPacket[i]: "totCount|rotDir"
            #Or "totCount|rotDir|currentVal|homingBool"
            #Extract both variables, put into SPData object.
            SPData.ExtractVars(dataPacket)
    else:
        return lastCheckOld, controlBool
    return lastCheck, controlBool

#Proof-of-concept function: No tests available
def SetPointControl1(SPData: SerialData, localMu: serial.Serial, 
                    mSpeedMax: int = 255, mSpeedMin: int = 150, 
                    encAlg: str = "utf-8"):
    """Outputs the desired motor speeds and rotational direction based 
    on the inputs of a local microcontroller over serial.
    :param: SPData: A SerialData class instance.
    :param localMu: A serial.Serial() instance representing the serial
                    communication to the local microcontroller.
    :param mSpeedMax: Maximum integer value of the motor speed PWM.
    :param mSpeedMin: Minimum integer value of the motor speed PWM.
    :param encAlg: String containing the algorithm used to encode and 
                   decode the bytes sent over serial. 
    
    Example input:
    lenData = 5 #Number of motors
    cprList = [4320 for i in range(lenData)]
    desAngles = [3*np.pi for i in range(lenData)]
    maxDeltaAngles = [0.1*np.pi for i in range(lenData)]
    tolAngle = [0.02*np.pi for i in range(lenData)]
    SPData = SerialData(lenData, cprList, desAngles, maxDeltaAngles, 
                        tolAngle)
    localMu = StartComms("COM9", 115200)
    mSpeedMax = 200
    mSpeedMin = 150
    encAlg = "utf-8"
    SetPointControl1(SPData, localMu, mSpeedMax, mSpeedMin, encAlg)
    """
    commFault = SPData.checkCommFault()
    SPData.GetDir()
    success = SPData.CheckTolAng()
    for i in range(SPData.lenData):
        if commFault[i] or success[i]:
            continue
        else:
            SPData.PControl1(i, mSpeedMax, mSpeedMin)
            #Proportional control
        SPData.dataOut[i] = f"{SPData.mSpeed[i]}|" + \
                            f"{SPData.rotDirDes[i]}"
    localMu.write(f"{SPData.dataOut}\n".encode(encAlg))

if __name__ == "__main__":
    ### SETUP OF ROBOT INSTANCE ###
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
    #Joint limits in home configuration, of the form [lower, upper]:
    lims0 = [-0.945*np.pi, 0.945*np.pi] #+/- 170 deg
    lims1 = [-0.25*np.pi, 0.5*np.pi] #-45 deg, + 90 deg
    lims2 = [-np.pi, 0.25*np.pi] #-180 deg, + 45 deg.
    lims3 = [-np.pi, 0.25*np.pi] #-180 deg, +45 deg.
    lims4 = [-np.pi, np.pi] #+/- 180 deg.
    limDummy = [-20*np.pi, 20*np.pi] #FOR TESTING!!!
    cpr = 1440
    #gearRatioList = [19.7*50, 19.7*50, (65.5*20)/9, (65.5*20)/9, (127.7*32)/9]
    gearRatioList = [3, 1, 1, 1, 1] #Temporary
    L0 = Link(iMat0, massList[0], None, Tsi0)
    L1 = Link(iMat1, massList[1], L0, Tsi1)
    L2 = Link(iMat2, massList[2], L1, Tsi2)
    L34 = Link(iMat34, massList[3], L2, Tsi34)
    J0 = Joint(S0, [None, L0], gearRatioList[0], cpr, limDummy) #CHANGE LIMDUMMY!
    J1 = Joint(S1, [L0, L1], gearRatioList[1], cpr, lims1)
    J2 = Joint(S2, [L1, L2], gearRatioList[2], cpr, lims2)
    J3 = Joint(S3, [L2,34], gearRatioList[3], cpr, lims3)
    J4 = Joint(S4, [L2,L34], gearRatioList[4], cpr, lims4)
    Pegasus = Robot([J0, J1, J2, J3, J4], [L0, L1, L2, L34])
    ### END OF ROBOT INSTANCE SETUP ###
    
    ### SETUP SERIAL COMMUNICATION ###
    baudRate = 115200
    lenData = 5 #Number of motors
    cprList = [Pegasus.joints[i].cpr for i in range(lenData)]
    desAngles = [4*np.pi, 4*np.pi, 4*np.pi, 4*np.pi, 4*np.pi]
    maxDeltaAngles = [5*np.pi for i in range(lenData)]
    tolAngle = [0.02*np.pi for i in range(lenData)]
    SPData = SerialData(lenData, desAngles, maxDeltaAngles, tolAngle, Pegasus.joints)
    dtComm = 0.005
    port, warning = FindSerial()
    localMu = StartComms(port, baudRate)
    mSpeedMax = 200
    mSpeedMin = 120
    encAlg = "utf-8"
    print("Starting serial communication. \nType Ctrl+C to stop")
    try:
        lastCheck = time.time()
        while True:
            lastCheck, controlBool = SReadAndParse(SPData, lastCheck, dtComm, localMu, encAlg)
            if controlBool and (time.time() - lastCheck >= dtComm):
                commFault = SPData.CheckCommFault()
                SPData.GetDir()
                success = SPData.CheckTolAng()
                for i in range(SPData.lenData):
                    if commFault[i] or success[i]:
                        continue
                    else:
                        angleErr = SPData.desAngle[i] - SPData.currAngle[i]
                        SPData.PControl1(i, mSpeedMax, mSpeedMin)
                        if SPData.mSpeed[i] < mSpeedMin:
                            SPData.mSpeed[i] = mSpeedMin
                        elif SPData.mSpeed[i] > mSpeedMax:
                            SPData.mSpeed[i] = mSpeedMax
                    SPData.dataOut[i] = f"{SPData.mSpeed[i]}|" + \
                                        f"{SPData.rotDirDes[i]}"
                print(SPData.dataOut)
                localMu.write(f"{SPData.dataOut}\n".encode(encAlg))
    except KeyboardInterrupt:
        #Set motor speeds to zero & close serial.
        localMu.write(f"{['0|0'] * lenData}\n".encode(encAlg))
        time.sleep(dtComm)
        localMu.__del__()
        print("Ctrl+C pressed, quitting...")
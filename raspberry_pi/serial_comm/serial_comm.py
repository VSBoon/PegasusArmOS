import sys
sys.path.append('C:\\DeKUT_Internship\\Robot_Arm\\PegasusArmOS\\raspberry_pi\\')
from classes import SerialData, InputError
from typing import List, Tuple
import serial
import time
import numpy as np

#TODO: Docstrings, examples, & tests.
def startComms(comPort: str, baudRate: int = 115200) -> serial.Serial:
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

def getComms(localMu: serial.Serial, encAlg: str = "utf-8") -> str:
    """Reads and decodes incoming byte data over a serial connection.
    :param localMu: A serial.Serial() instance representing the serial
                    communication to the local microcontroller.
    :param encAlg: String containing the algorithm used to encode and 
                   decode the bytes sent over serial.
    :return dataIn: String of decoded bytes available in the serial buffer 
                    until the representation of the newline character.
    Example input:
    localMu = startComms("COM9", baudRate=115200)
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

def SetPointControl(SPData: SerialData, lastCheckOld: float, dtComm: float, 
                    localMu: serial.Serial, mSpeedMax: int = 255, 
                    mSpeedMin: int = 150, encAlg: str = "utf-8") -> float:
    """Outputs the desired motor speeds and rotational direction based 
    on the inputs of a local microcontroller over serial.
    :param: SPData: A SerialData class instance.
    :param lastCheckOld: time.time() since last data was sent.
    :param dtComm: The desired time between sending data to the local
                   microcontroller.
    :param localMu: A serial.Serial() instance representing the serial
                    communication to the local microcontroller.
    :param mSpeedMax: Maximum integer value of the motor speed PWM.
    :param mSpeedMin: Minimum integer value of the motor speed PWM.
    :param encAlg: String containing the algorithm used to encode and 
                   decode the bytes sent over serial. 
    
    Example input:
    lenData = 6 #Number of motors
    cprList = [4320 for i in range(lenData)]
    desAngles = [3*np.pi for i in range(lenData)]
    maxDeltaAngles = [0.1*np.pi for i in range(lenData)]
    tolAngle = [0.02*np.pi for i in range(lenData)]
    SPData = SerialData(lenData, cprList, desAngles, maxDeltaAngles, 
                        tolAngle)
    dtComm = 0.005
    localMu = startComms("COM9", 115200)
    mSpeedMax = 200
    mSpeedMin = 150
    encAlg = "utf-8"
    lastCheck = time.time()
    SetPointControl(SPData, lastCheck, dtComm, localMu, mSpeedMax, 
    mSpeedMin, encAlg)
    Output:
    1634299822.1247501
    """
    elapsedTime = time.time() - lastCheckOld
    if elapsedTime >= dtComm:
        lastCheck = time.time()
        if localMu.inWaiting() == 0:
            return lastCheckOld
        elif localMu.inWaiting() > 0:
            try:
                try:
                    dataIn = getComms(localMu, encAlg)
                except InputError: 
                    localMu.reset_input_buffer()
                    return lastCheckOld
                dataPacket = dataIn[1:-1].split('][')
                if len(dataPacket) != SPData.lenData: #flush & retry
                    localMu.reset_input_buffer()
                    return lastCheck
                for i in range(SPData.lenData):
                    #Expected form dataPacket[i]: "totCount|rotDir"
                    #Extract both variables, put into SPData object.
                    SPData.ExtractVars(dataPacket[i], i)
                    commFault = SPData.CheckCommFault(i) 
                    SPData.GetDir(i)
                    success = SPData.CheckTolAng(i)
                    if commFault or success:
                        continue
                    else:
                        SPData.PControl1(i, mSpeedMax, mSpeedMin)
                        #Proportional control
                    SPData.dataOut[i] = f"{SPData.mSpeed[i]}|" + \
                                        f"{SPData.rotDirDes[i]}"
                localMu.write(f"{SPData.dataOut}\n".encode('utf-8'))
            except ValueError as e:
                if "invalid literal for int() with base 10" in str(e) or "not enough values to unpack" in str(e):
                    localMu.reset_input_buffer()
                    pass
        return lastCheck
    else:
        return lastCheckOld

if __name__ == "__main__":
    baudRate = 115200
    lenData = 6 #Number of motors
    cprList = [4320 for i in range(lenData)]
    desAngles = [3*np.pi for i in range(lenData)]
    maxDeltaAngles = [np.pi for i in range(lenData)]
    tolAngle = [0.04*np.pi for i in range(lenData)]
    SPData = SerialData(lenData, cprList, desAngles, maxDeltaAngles, tolAngle)
    dtComm = 0.005
    localMu = startComms("COM9", baudRate)
    mSpeedMax = 200
    mSpeedMin = 150
    encAlg = "utf-8"
    print("Starting serial communication. \nType Ctrl+C to stop")
    try:
        lastCheck = time.time()
        while True:
            lastCheck = SetPointControl(SPData, lastCheck, dtComm, localMu, mSpeedMax, mSpeedMin, encAlg)
    except KeyboardInterrupt:
        #Set motor speeds to zero & close serial.
        localMu.write(f"{['0|0'] * lenData}\n".encode('utf-8'))
        time.sleep(dtComm)
        localMu.__del__()
        print("Ctrl+C pressed, quitting...")
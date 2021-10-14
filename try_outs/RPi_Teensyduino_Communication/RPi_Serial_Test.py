import serial
import time
import numpy as np
#Code from: www.aranacorp.com/en/serial-communication-between-raspberry-pi-and-arduino/

def startComms(baudRate: int) -> "Serial":
    arduino = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
    time.sleep(0.05) #wait for serial to open
    if arduino.isOpen():
        print(f"{arduino.port} connected!")
    return arduino


def getComms(arduino: "Serial") -> str:
    dataIn = arduino.readline().decode('utf-8').rstrip() #remove decoding
    return dataIn

if __name__ == "__main__":
    nCommands = 6 #Number of data packets expected per signal from Ard.
    cpr = 4320 #counts per revolution on output shaft
    desAngle = [4*np.pi] * nCommands
    angleTol = (1/180)*np.pi
    mSpeedMin = 130 #Minimal PWM speed (out of 255)
    mSpeedMax = 220 #Due to encoder concerns
    mSpeedRange = mSpeedMax - mSpeedMin
    maxDeltaAngle = 0.5*np.pi #To catch comm errors, MIGHT CAUSE ISSUES!
    
    print("Starting serial communication. \nType Ctrl+C to stop")
    arduino = startComms(115200)
    totCount = [None] * nCommands
    rotDir = [None] * nCommands
    prevAngle = [None] * nCommands
    currentAngle = [0] * nCommands
    rotCCW = [None] * nCommands
    mSpeed = [None] * nCommands
    dataOut = [None] * nCommands
    lastCheck = time.time()
    deltaTComm = 0.005
    
    try:
        while True:
            elapsedTime = time.time() - lastCheck
            if elapsedTime >= deltaTComm:
                lastCheck = time.time()
                if arduino.inWaiting() == 0:
                    pass
                elif arduino.inWaiting() > 0:
                    try:
                        dataIn = getComms(arduino)
                        dataPacket = dataIn[1:-1].split('][')
                        if len(dataPacket) != nCommands: #flush & retry
                            arduino.reset_input_buffer()
                            continue
                        for i in range(nCommands):
                            #Expected form: "[totCount|rotDir]"
                            #Extract both variables
                            totCount[i], rotDir[i] = dataPacket[i].split('|')
                            totCount[i] = int(totCount[i])
                            rotDir[i] = int(rotDir[i])
                            prevAngle[i] = currentAngle[i]
                            currentAngle[i] = (totCount[i]/cpr) * 2*np.pi 
                            if abs(prevAngle[i] - currentAngle[i]) >= maxDeltaAngle:
                                currentAngle[i] == prevAngle[i]
                                mSpeed[i] = 0
                                dataOut[i] = f"{mSpeed[i]}|{rotCCW[i]}"
                                print("likely comm error")
                                continue
                            if currentAngle[i] <= desAngle[i] and rotDir[i] != 0:
                                rotCCW[i] = 0
                            elif currentAngle[i] > desAngle[i] and rotDir[i] != 1:
                                rotCCW[i] = 1
                            else:
                                rotCCW[i] = rotDir[i]
                            if abs(desAngle[i] - currentAngle[i]) <= angleTol:
                                mSpeed[i] = 0
                                dataOut[i] = f"{mSpeed[i]}|{rotCCW[i]}"
                                continue
                            else:
                                #Proportional control
                                angleErr = desAngle[i] - abs(currentAngle[i])
                                mSpeed[i] = int(mSpeedMin + (angleErr/np.pi)* mSpeedRange)
                                if mSpeed[i] > mSpeedMax:
                                    mSpeed[i] = mSpeedMax
                                elif mSpeed[i] < mSpeedMin:
                                    mSpeed[i] = mSpeedMin
                            dataOut[i] = f"{mSpeed[i]}|{rotCCW[i]}"
                        arduino.write(f"{dataOut}\n".encode('utf-8'))
                    except ValueError as e:
                        if "invalid literal for int() with base 10" in str(e) or "not enough values to unpack" in str(e):
                            print("int / unpack error!")
                            arduino.reset_input_buffer()
                            pass
    except KeyboardInterrupt:
        arduino.write(f"{['0|0', '0|0']}\n".encode('utf-8'))
        arduino.__del__()
        print("Ctrl+C pressed, quitting...")


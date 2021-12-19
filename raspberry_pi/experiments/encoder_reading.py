import os
import sys
#Find directory path of current file
current = os.path.dirname(os.path.realpath(__file__))
#Find directory path of parent folder and add to sys path
parent = os.path.dirname(current)
sys.path.append(parent)

import matplotlib.pyplot as plt
import numpy as np
import time
import csv
from serial_comm.serial_comm import StartComms, GetComms, FindSerial, SReadAndParse
from classes import SerialData, Robot
from robot_init import robotFric as Pegasus
from settings import sett


serial = SerialData(6, Pegasus.joints)
#port = FindSerial(askInput=True)[0]
Teensy = StartComms('COM13') #TEMPORARY, REPLACE WITH port
name = input("Desired name of csv file (including '.csv'): ")
dt = float(input("Total time of recording in [s]: "))
dtComm = sett['dtComm']
lastComm = time.perf_counter()
dataMat = np.zeros((1,2))
PWM = int(input("PWM: "))
serial.mSpeed[2] = PWM
serial.rotDirDes[2] = 0
print(f"Start recording for {dt} seconds.")
start = time.perf_counter()
while time.perf_counter() - start <= dt:
    if (time.perf_counter() - lastComm >= dtComm):
        SReadAndParse(serial, Teensy)
        data = np.hstack((np.array(serial.currAngle[2]), np.array(serial.mSpeed[2])))
        dataMat = np.vstack([dataMat, data])
        for i in range(serial.lenData-1): 
            serial.dataOut[i] = f"{abs(serial.mSpeed[i])}|"+\
                                f"{serial.rotDirDes[i]}"
        serial.dataOut[-1] = f"{0|0}"
        Teensy.write(f"{serial.dataOut}\n".encode('utf-8')) 
        lastComm = time.perf_counter()
np.savetxt(name, dataMat, delimiter=",", fmt='%.4f')
serial.dataOut = [f"{0|0}" for i in range(6)]
Teensy.write(f"{serial.dataOut}\n".encode('utf-8'))
print("Done recording")
dtArr = np.linspace(0, dt, num=int(np.floor(dt/dtComm)))
print(dataMat[3:,0].size)
print(dtArr.size)
plt.scatter(dtArr[2:], dataMat[3:,0], color='r', marker='P', s=10, label='angle M3')
plt.scatter(dtArr[2:], dataMat[3:,1], color='g', marker='.', s=40, label='PWM M3')
#plt.plot(dtArr, dataMat[:,1], color='g', label='Wrist Up/Down (M4/M5)')
#plt.scatter(dtArr, dataMat[:,2], color='b', marker='.', s=20, label='Wrist twisting (M4/M5)')
plt.xlabel("Time (s)")
plt.ylabel("Angle (rad)")
plt.title("Gravity compensation with steady PWM")
plt.xlim((0,dt))
plt.ylim((-np.pi,np.pi))
plt.legend(loc='best')
plt.grid()
plt.show()
plt.close()
Teensy.__del__()
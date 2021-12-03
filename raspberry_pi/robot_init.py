import os
import sys
#Find directory path of current file
current = os.path.dirname(os.path.realpath(__file__))
#Find directory path of parent folder and add to sys path
parent = os.path.dirname(current)
sys.path.append(parent)

import numpy as np
from classes import Link, Joint, Robot

"""Initialisation script for the PegasusArm OS robot instance.
If the model is to be updated, it is highly recommended to do it
centerally in this file. All data for this initialization is also
stored in the folder "Parts & Hardware info" in various .txt files.
When updating these values, kindly update those .txt files accordingly.
"""

#Inertia matrices of each link
iMat0 = np.diag([0.03947, 0.03362, 0.04886])
iMat1 = np.diag([0.00393, 0.00237, 0.00172])
iMat2 = np.diag([0.00294, 0.00210, 0.0029])
iMat34 = np.diag([0.00041, 0.00348, 0.00364])
massList = [5.13, 0.507, 0.420, 0.952, 0.952]

"""Transformation matrices from CoM of links with principle axes of
inertia as the cardinal directions to the space frame (Tsi):"""
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

"""Transformation matrix of the end-effector in the space frame in
the home position."""
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

"""Lower- and upper limits of each (virtual) joint from the home
position."""
lims0 = [-0.945*np.pi, 0.945*np.pi] #+/- 170 deg
lims1 = [-0.25*np.pi, 0.5*np.pi] #-45 deg, + 90 deg
lims2 = [-np.pi, 0.25*np.pi] #-180 deg, + 45 deg.
lims3 = [-np.pi, 0.25*np.pi] #-180 deg, +45 deg.
lims4 = [-np.pi, np.pi] #+/- 180 deg.

gearRatioList = [19.7*50, 19.7*50, (65.5*20)/9, (65.5*20)/9, (127.7*32)/9]
cpr = 512 #Encoder counts per revolution of the initial motor shaft.

"""Link-class objects for each link"""
L0 = Link(iMat0, massList[0], None, Tsi0)
L1 = Link(iMat1, massList[1], L0, Tsi1)
L2 = Link(iMat2, massList[2], L1, Tsi2)
L34 = Link(iMat34, massList[3], L2, Tsi34)
links = [L0, L1, L2, L34, L34]

"""Torque constant at the input shaft of the motor, so before the 
internal gearbox"""
km = [36.5e-3,36.5e-3,36.5e-3,36.5e-3,36.5e-3,26.4e-3] 
J0 = Joint(S0, [None, L0], gearRatioList[0], km[0], cpr, lims0)
J1 = Joint(S1, [L0, L1], gearRatioList[1], km[1], cpr, lims1)
J2 = Joint(S2, [L1, L2], gearRatioList[2], km[2], cpr, lims2)
J3 = Joint(S3, [L2,L34], gearRatioList[3], km[3], cpr, lims3)
J4 = Joint(S4, [L2,L34], gearRatioList[4], km[4], cpr, lims4)
joints = [J0, J1, J2, J3, J4]
robot = Robot(joints, links, TsbHome)
###END OF ROBOT INITIALISATION###

"""Here, we add a friction model to our joints for a second (hopefully 
more accurate) instance of the robot. Values should be experimentally confirmed!"""
PWMMinWormGear = 60 #Can be confirmed & adjusted!
PWMMinSprocket = 30 #Should be confirmed & adjusted!
eff = [0.65, 0.65, 0.75, 0.75, 0.75]
tauStat0 = (2/255)*(J0.gearRatio*J0.km)*PWMMinWormGear
tauStat1 = (2/255)*(J1.gearRatio*J1.km)*PWMMinWormGear
tauStat2 = (2/255)*(J2.gearRatio*J2.km)*PWMMinSprocket
tauStat3 = (2/255)*(J3.gearRatio*J3.km)*PWMMinSprocket
tauStat4 = (2/255)*(J4.gearRatio*J4.km)*PWMMinSprocket
tauStat = [tauStat0, tauStat1, tauStat2, tauStat3, tauStat4]
tauKin = [0.9*stat for stat in tauStat]
bVisc = [0,0,0,0,0]
J0N = Joint(S0, [None, L0], gearRatioList[0], km[0], cpr, lims0, tauStat[0], 
            tauKin[0], bVisc[0], eff[0])
J1N = Joint(S1, [L0, L1], gearRatioList[1], km[1], cpr, lims1, tauStat[1], 
            tauKin[1], bVisc[1], eff[1])
J2N = Joint(S2, [L1, L2], gearRatioList[2], km[2], cpr, lims2, tauStat[2], 
            tauKin[2], bVisc[2], eff[2])
J3N = Joint(S3, [L2,L34], gearRatioList[3], km[3], cpr, lims3, tauStat[3], 
            tauKin[3], bVisc[3], eff[3])
J4N = Joint(S4, [L2,L34], gearRatioList[4], km[4], cpr, lims4, tauStat[4], 
            tauKin[4], bVisc[4], eff[4])
joints = [J0N, J1N, J2N, J3N, J4N]
robotFric = Robot(joints, links, TsbHome)
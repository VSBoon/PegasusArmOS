import os
import sys
#Find directory path of current file
current = os.path.dirname(os.path.realpath(__file__))
#Find directory path of parent folder and add to sys path
parent = os.path.dirname(current)
sys.path.append(parent)

from dynamics.dynamics_funcs import FeedForward, LossComp

tauComm = 5
dtheta = 1
tauStat = 0.02
bVisc = 0.04
tauKin = 0.015
eff = 0.8

def test_LossCompNoLoss():
    """Checks functionality if loss model is nullified"""
    tauStat = 0
    bVisc = 0
    tauKin = 0
    eff = 1
    tau = LossComp(tauComm, dtheta, tauStat, bVisc, tauKin, eff)
    assert tau == tauComm

def test_LossCompEff():
    """Checks efficiency calculation"""
    tauStat = 0
    bVisc = 0
    tauKin = 0
    eff = 0.8
    tau = LossComp(tauComm, dtheta, tauStat, bVisc, tauKin, eff)
    assert tau == tauComm/eff

def test_LossCompStat():
    """Observe if static & kinetic friction (w/o viscous) works."""
    tauStat = 1
    dtheta = 0
    eff = 1
    bVisc = 0.04
    tau = LossComp(tauComm, dtheta, tauStat, bVisc, tauKin, eff)
    assert tau == tauComm + tauStat
    dtheta = 1
    bVisc = 0
    tau = LossComp(tauComm, dtheta, tauStat, bVisc, tauKin, eff)
    assert tau == tauComm + tauKin

def test_LossCompDyn():
    """Checks if full functionality works"""
    tau = LossComp(tauComm, dtheta, tauStat, bVisc, tauKin, eff)
    assert tau == (tauComm + tauKin + bVisc*dtheta)/eff

def test_LossCompSign():
    """See if LossComp correctly handles velocity signes"""
    dtheta = -1
    eff = 1
    tau = LossComp(tauComm, dtheta, tauStat, bVisc, tauKin, eff)
    assert tau == tauComm - tauKin + bVisc*dtheta     
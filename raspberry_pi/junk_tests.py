import numpy as np
thetaGuessList = [4.9*np.pi]
jointLimits = [[-0.5*np.pi, 0.5*np.pi]]
i = 0
if thetaGuessList[i] > np.pi:
    thetaGuessList[i] = thetaGuessList[i] % np.pi
elif thetaGuessList[i] <= -np.pi:
    thetaGuessList[i] = thetaGuessList[i] % -np.pi
if thetaGuessList[i] > jointLimits[i][1]:
    thetaGuessList[i] = jointLimits[i][1]
elif thetaGuessList[i] < jointLimits[i][0]:
    thetaGuessList[i] = jointLimits[i][0]
print(thetaGuessList[i])
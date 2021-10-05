import modern_robotics as mr
import numpy as np
from typing import Union, List, Tuple

def TrajGen(startConfig: Union[np.ndarray, List[float]], endConfig: 
    Union[np.ndarray, List[float]], tTot: float, nSubConfigs: int, 
    method: str="screw", timeScaling: int = 3) -> Tuple[np.ndarray, List[float]]:
    """Docstring"""
    if timeScaling != 3 | timeScaling != 5:
                print("Invalid timeScaling; defaulting to quintic.")
                timeScaling = 5
    
    if method == "joint" or method == "Joint":
        if not isinstance(startConfig, list) or \
           not isinstance(endConfig, list):
            raise SyntaxError("To make a trajectory in the joint space," +
                              " please input a list of joint angles.")
        else:
            trajList = mr.JointTrajectory(np.array(startConfig), 
            np.array(endConfig), tTot, nSubConfigs, timeScaling)
    else:
        if not isinstance(startConfig, np.ndarray) or \
           not isinstance(endConfig, np.ndarray):
            raise SyntaxError("To make a trajectory in SE(3), input SE(3) " + 
                              "numpy arrays as start- and end configurations.")
        elif not mr.TestIfSE3(startConfig) or not mr.TestIfSE3(endConfig):
            raise SyntaxError("Ensure that both the start- and end " +
                              "configuration are part of the SE(3) " + 
                              "manifold.")
        
        else:
            if method == "screw" or method == "Screw":
                trajList = mr.ScrewTrajectory(startConfig, endConfig, tTot, 
                nSubConfigs, timeScaling)
            elif method == "cartesian" or method == "Cartesian":
                trajList = mr.CartesianTrajectory(startConfig, endConfig, 
                tTot, nSubConfigs, timeScaling)
            else:
                raise SyntaxError("Invalid method input. Please choose " + 
                                  "between 'joint', 'screw', or 'cartesian'")
    timeList = [dt * (tTot/(nSubConfigs-1)) for dt in range(0, nSubConfigs)]
    return (trajList, timeList)
import numpy as np

from nav_msgs.msg import Odometry

class Controller:
    def __init__(self, time: float):
        pass

    # TODO: Change ref pose to ref odom
    def control(time, thrust_cmd: float=None, reference_odom: Odometry=None, current_state: Odometry=None, v_ref: np.array=None, v_cur: np.array=None):
        pass

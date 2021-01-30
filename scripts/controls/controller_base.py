import numpy as np

from nav_msgs.msg import Odometry

class Controller:
    def __init__(self, time: float):
        pass

    # TODO: Change ref pose to ref odom
    def control(time, reference_odom: Odometry, current_state: Odometry, v_ref: np.array=None, v_cur: np.array=None, thrust_cmd: float = None):
        pass

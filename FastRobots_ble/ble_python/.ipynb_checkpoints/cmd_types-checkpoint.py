from enum import Enum

class CMD(Enum):
    LOG = 0
    ICM = 1
    DISTANCE1 = 2
    DISTANCE2 = 3
    PID = 4
    STOP = 5
    GAINS = 6
    DRIVE = 7
    RLIMIT = 8
    LLIMIT = 9
import enum


class ReliabilityPolicy(enum.Enum):
    RELIABLE = 1
    BEST_EFFORT = 2


class DurabilityPolicy(enum.Enum):
    VOLATILE = 1
    TRANSIENT_LOCAL = 2


class HistoryPolicy(enum.Enum):
    KEEP_LAST = 1


class QoSProfile:
    def __init__(self, depth=10, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST):
        self.depth, self.reliability, self.durability, self.history = depth, reliability, durability, history


qos_profile_sensor_data = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

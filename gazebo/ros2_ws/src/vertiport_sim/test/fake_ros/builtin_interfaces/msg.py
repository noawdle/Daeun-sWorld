class Time:
    def __init__(self, sec=0, nanosec=0):
        self.sec, self.nanosec = int(sec), int(nanosec)

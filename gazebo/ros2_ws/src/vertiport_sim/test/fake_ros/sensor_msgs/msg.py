from builtin_interfaces.msg import Time


class Header:
    def __init__(self):
        self.stamp = Time()
        self.frame_id = ''


class Image:
    def __init__(self):
        self.header = Header()
        self.height = self.width = self.step = 0
        self.encoding = ''
        self.is_bigendian = 0
        self.data = b''


class CameraInfo:
    def __init__(self):
        self.header = Header()
        self.height = self.width = 0
        self.distortion_model = ''
        self.d = []
        self.k = [0.0] * 9

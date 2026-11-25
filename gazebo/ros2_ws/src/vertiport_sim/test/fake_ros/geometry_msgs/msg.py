from builtin_interfaces.msg import Time


class _V:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class TransformStamped:
    def __init__(self):
        self.header = _V(stamp=Time(), frame_id='')
        self.child_frame_id = ''
        self.transform = _V(translation=_V(x=0.0, y=0.0, z=0.0), rotation=_V(x=0.0, y=0.0, z=0.0, w=1.0))

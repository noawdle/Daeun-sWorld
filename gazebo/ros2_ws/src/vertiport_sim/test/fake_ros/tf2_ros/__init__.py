"""tf2_ros 대체물: 보낸 변환을 기록."""
SENT = []
STATIC = []


class TransformBroadcaster:
    def __init__(self, node):
        self.node = node

    def sendTransform(self, t):
        ts = t if isinstance(t, list) else [t]
        SENT.extend(ts)
        del SENT[:-50]


class StaticTransformBroadcaster(TransformBroadcaster):
    def sendTransform(self, t):
        STATIC.extend(t if isinstance(t, list) else [t])

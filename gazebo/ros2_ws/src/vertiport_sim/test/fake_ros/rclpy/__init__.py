"""테스트용 초소형 rclpy 대체물 (프로세스 내 pub/sub + 가상 시간). 실제 ROS 가 없는 환경 검증용."""
import queue as _queue
import threading as _threading

_ok = False
BUS = {}            # topic -> [Subscription]
PUBS = {}           # topic -> [Publisher]
LATCH = {}          # topic -> last msg (transient local)
TIMERS = []
DELIVERY = _queue.Queue()
PARAMS = {}         # node_name -> {param: value}
VT = [0.0]          # 가상 시간 [s]
LOG = []
_lock = _threading.Lock()


def init(args=None):
    global _ok
    _ok = True


def ok():
    return _ok


def shutdown():
    global _ok
    _ok = False


def reset():
    BUS.clear(); PUBS.clear(); LATCH.clear(); TIMERS.clear(); PARAMS.clear(); LOG.clear()
    while not DELIVERY.empty():
        DELIVERY.get_nowait()
    VT[0] = 0.0


def drain():
    n = 0
    while True:
        try:
            sub, msg = DELIVERY.get_nowait()
        except _queue.Empty:
            return n
        if not sub.dead:
            sub.cb(msg)
        n += 1


def advance(dt):
    """가상 시간을 dt 만큼 진행하며 타이머/메시지 처리."""
    end = VT[0] + dt
    while True:
        drain()
        due = [t for t in TIMERS if not t.cancelled]
        if not due:
            VT[0] = end
            return
        nxt = min(t.next for t in due)
        if nxt > end:
            VT[0] = end
            drain()
            return
        VT[0] = max(VT[0], nxt)
        for t in due:
            if t.next <= VT[0] and not t.cancelled:
                t.next += t.period
                t.cb()
                drain()


def spin(node):
    raise RuntimeError('use advance() in tests')


def spin_once(node, timeout_sec=None):
    advance(timeout_sec or 0.01)


def create_node(name):
    from .node import Node
    return Node(name)

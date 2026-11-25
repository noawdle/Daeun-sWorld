import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile


class _Param:
    def __init__(self, value):
        self.value = value


class _Logger:
    def __init__(self, name):
        self.name = name

    def _log(self, lvl, msg):
        rclpy.LOG.append((self.name, lvl, msg))
        if lvl in ('WARN', 'ERROR') or rclpy.__dict__.get('VERBOSE'):
            print(f'  [{lvl}] [{self.name}] {msg}')

    def info(self, m): self._log('INFO', m)
    def warn(self, m): self._log('WARN', m)
    warning = warn
    def error(self, m): self._log('ERROR', m)
    def debug(self, m): pass


class _Time:
    def __init__(self, ns):
        self.nanoseconds = ns


class _Clock:
    def now(self):
        return _Time(int(rclpy.VT[0] * 1e9))


class Timer:
    def __init__(self, period, cb):
        self.period, self.cb, self.cancelled = period, cb, False
        self.next = rclpy.VT[0] + period

    def cancel(self):
        self.cancelled = True


class Publisher:
    def __init__(self, node, mtype, topic, qos):
        self.node, self.mtype, self.topic, self.qos = node, mtype, topic, qos
        rclpy.PUBS.setdefault(topic, []).append(self)

    def publish(self, msg):
        if not isinstance(msg, self.mtype):
            raise TypeError(f'{self.topic}: expected {self.mtype.__name__}, got {type(msg).__name__}')
        if getattr(self.qos, 'durability', None) == DurabilityPolicy.TRANSIENT_LOCAL:
            rclpy.LATCH[self.topic] = msg
        for s in list(rclpy.BUS.get(self.topic, [])):
            if s.mtype is not self.mtype:
                raise TypeError(f'type mismatch on {self.topic}')
            rclpy.DELIVERY.put((s, msg))

    def get_subscription_count(self):
        return len([s for s in rclpy.BUS.get(self.topic, []) if not s.dead])


class Subscription:
    def __init__(self, mtype, topic, cb, qos):
        self.mtype, self.topic, self.cb, self.qos, self.dead = mtype, topic, cb, qos, False


def _check_type(name, default, value):
    def kind(v):
        if isinstance(v, bool): return 'bool'
        if isinstance(v, int): return 'int'
        if isinstance(v, float): return 'float'
        if isinstance(v, str): return 'str'
        if isinstance(v, (list, tuple)):
            ks = {kind(x) for x in v}
            return 'list[' + ','.join(sorted(ks)) + ']'
        return type(v).__name__
    if kind(default) != kind(value):
        raise TypeError(f'InvalidParameterTypeException: {name}: default {kind(default)} vs override {kind(value)}')


class Node:
    def __init__(self, name):
        self._name = name
        self._subs = []
        self._overrides = rclpy.PARAMS.get(name, {})
        self._logger = _Logger(name)

    def get_name(self):
        return self._name

    def declare_parameter(self, name, default):
        if name in self._overrides:
            v = self._overrides[name]
            _check_type(f'{self._name}.{name}', default, v)
            return _Param(v)
        return _Param(default)

    def create_publisher(self, mtype, topic, qos):
        if isinstance(qos, int):
            qos = QoSProfile(depth=qos)
        return Publisher(self, mtype, topic, qos)

    def create_subscription(self, mtype, topic, cb, qos):
        if isinstance(qos, int):
            qos = QoSProfile(depth=qos)
        s = Subscription(mtype, topic, cb, qos)
        rclpy.BUS.setdefault(topic, []).append(s)
        self._subs.append(s)
        if qos.durability == DurabilityPolicy.TRANSIENT_LOCAL and topic in rclpy.LATCH:
            rclpy.DELIVERY.put((s, rclpy.LATCH[topic]))
        return s

    def destroy_subscription(self, s):
        s.dead = True
        if s in rclpy.BUS.get(s.topic, []):
            rclpy.BUS[s.topic].remove(s)

    def create_timer(self, period, cb):
        t = Timer(period, cb)
        rclpy.TIMERS.append(t)
        return t

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return _Clock()

    def get_topic_names_and_types(self):
        names = set(k for k, v in rclpy.PUBS.items() if v) | set(k for k, v in rclpy.BUS.items() if v)
        return [(n, ['?']) for n in sorted(names)]

    def count_publishers(self, topic):
        return len(rclpy.PUBS.get(topic, []))

    def count_subscribers(self, topic):
        return len([x for x in rclpy.BUS.get(topic, []) if not x.dead])

    def destroy_node(self):
        for s in self._subs:
            self.destroy_subscription(s)

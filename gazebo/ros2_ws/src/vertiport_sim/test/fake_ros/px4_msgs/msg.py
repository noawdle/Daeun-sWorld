"""px4_msgs 대체물: 필드 이름을 엄격히 검사 (오타 → AttributeError)."""
import copy


class _Msg:
    FIELDS = {}

    def __init__(self, **kw):
        for k, v in self.FIELDS.items():
            object.__setattr__(self, k, copy.copy(v))
        for k, v in kw.items():
            setattr(self, k, v)

    def __setattr__(self, k, v):
        if k not in self.FIELDS:
            raise AttributeError(f'{type(self).__name__} has no field {k}')
        object.__setattr__(self, k, v)


class VehicleLocalPosition(_Msg):
    FIELDS = dict(timestamp=0, xy_valid=False, z_valid=False, v_xy_valid=False, v_z_valid=False,
                  x=0.0, y=0.0, z=0.0, vx=0.0, vy=0.0, vz=0.0, heading=0.0,
                  dist_bottom=0.0, dist_bottom_valid=False)


class VehicleAttitude(_Msg):
    FIELDS = dict(timestamp=0, q=[1.0, 0.0, 0.0, 0.0])


class VehicleStatus(_Msg):
    FIELDS = dict(timestamp=0, arming_state=1, nav_state=2, failsafe=False, pre_flight_checks_pass=True)
    ARMING_STATE_DISARMED = 1
    ARMING_STATE_ARMED = 2
    NAVIGATION_STATE_POSCTL = 2
    NAVIGATION_STATE_AUTO_LOITER = 4
    NAVIGATION_STATE_OFFBOARD = 14
    NAVIGATION_STATE_AUTO_LAND = 18


class VehicleLandDetected(_Msg):
    FIELDS = dict(timestamp=0, landed=True, maybe_landed=True, ground_contact=True)


class OffboardControlMode(_Msg):
    FIELDS = dict(timestamp=0, position=False, velocity=False, acceleration=False, attitude=False,
                  body_rate=False, thrust_and_torque=False, direct_actuator=False)


class TrajectorySetpoint(_Msg):
    FIELDS = dict(timestamp=0, position=[0.0] * 3, velocity=[0.0] * 3, acceleration=[0.0] * 3,
                  jerk=[0.0] * 3, yaw=0.0, yawspeed=0.0)


class VehicleCommand(_Msg):
    FIELDS = dict(timestamp=0, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0,
                  param7=0.0, command=0, target_system=0, target_component=0, source_system=0,
                  source_component=0, confirmation=0, from_external=False)
    VEHICLE_CMD_NAV_LAND = 21
    VEHICLE_CMD_NAV_TAKEOFF = 22
    VEHICLE_CMD_DO_SET_MODE = 176
    VEHICLE_CMD_COMPONENT_ARM_DISARM = 400


class BatteryStatus(_Msg):
    FIELDS = dict(timestamp=0, connected=False, voltage_v=0.0, remaining=-1.0, warning=0)
    WARNING_NONE = 0
    WARNING_LOW = 1
    WARNING_CRITICAL = 2
    WARNING_EMERGENCY = 3
    WARNING_FAILED = 4


class VehicleGlobalPosition(_Msg):
    FIELDS = dict(timestamp=0, lat=0.0, lon=0.0, alt=0.0, lat_lon_valid=False, alt_valid=False, eph=0.0, epv=0.0)


class SensorGps(_Msg):
    FIELDS = dict(timestamp=0, latitude_deg=0.0, longitude_deg=0.0, fix_type=0, eph=0.0, epv=0.0,
                  satellites_used=0)

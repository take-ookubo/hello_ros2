"""Speed-torque and current limits of the joint modules.

Each module is modelled at the joint (after its gearbox) by straight lines
through the published points:

- RMD (MYACTUATOR, 48 V): the voltage limits the speed along the line
  through the no-load speed and the rated point (rated torque at rated
  speed); above that line the module cannot reach the speed. The torque
  is capped at the peak torque, and the RMS torque over a move is
  compared with the rated torque (heating).
- DYNAMIXEL (12 V, the recommended supply): the DC motor line from the
  no-load speed to the stall torque. ROBOTIS gives no continuous rating,
  so the RMS torque is compared with the provisional value MODULE_TORQUE
  of pose_cycle, as before. modules(14.8) gives the DYNAMIXEL points at
  14.8 V instead: the top of their input range (10.0-14.8 V), so a
  regulated supply, not a 4S battery (16.8 V when full).

Current is torque / torque constant (the module's own constant; for
DYNAMIXEL stall torque / stall current), plus the standby current. The
torque is also capped by the peak current: for the X8 2.4 N·m/A x 43.8 A
is 105 N·m, below its listed 120 N·m peak.

Only motoring (torque and speed in the same direction) is checked
against the speed line: when the joint brakes, the back-EMF helps and the
peak torque (current) is the limit. Gearbox losses and temperature are
not modelled, and the lines are approximations of the published curves.

Sources (2026-11-10): myactuator.us product pages of RMD-X8-P20-120 and
RMD-X6-P20-60; emanual.robotis.com pages of XM540-W270 and XM430-W350 (12 V and 14.8 V
columns).
"""
import math
from dataclasses import dataclass

RPM = 2 * math.pi / 60


@dataclass(frozen=True)
class Module:
    name: str
    voltage: float          # supply [V]
    no_load_speed: float    # [rad/s]
    line_torque: float      # torque where the speed line reaches line_speed [N·m]
    line_speed: float       # [rad/s]
    peak_torque: float      # short-time torque [N·m]
    continuous_torque: float  # RMS reference [N·m]
    torque_constant: float  # [N·m/A]
    peak_current: float     # [A]
    standby_current: float = 0.0   # [A]
    continuous_is_rated: bool = True   # False: provisional value, not published

    def speed_limit(self, torque):
        """Highest speed [rad/s] at which the module can give torque (motoring)."""
        slope = (self.no_load_speed - self.line_speed) / self.line_torque
        return max(0.0, self.no_load_speed - slope * abs(torque))

    @property
    def torque_cap(self):
        """Peak torque, or less if the peak current cannot give it."""
        return min(self.peak_torque, self.torque_constant * self.peak_current)

    def available_torque(self, speed, motoring=True):
        """Largest torque [N·m] at speed [rad/s]."""
        if not motoring:
            return self.torque_cap
        slope = (self.no_load_speed - self.line_speed) / self.line_torque
        return max(0.0, min(self.torque_cap, (self.no_load_speed - abs(speed)) / slope))

    def current(self, torque):
        return self.standby_current + abs(torque) / self.torque_constant


X8 = Module('RMD-X8-P20-120', 48.0, 158 * RPM, 43.0, 127 * RPM, 120.0, 43.0, 2.4, 43.8)
X6 = Module('RMD-X6-P20-60', 48.0, 176 * RPM, 20.0, 153 * RPM, 50.0, 20.0, 2.1, 29.1)
# DYNAMIXEL at 12 V: the speed line ends at the stall torque (zero speed).
XM540 = Module('XM540-W270 (12 V)', 12.0, 30 * RPM, 10.6, 0.0, 10.6, 6.0, 10.6 / 4.4, 4.4,
               standby_current=0.04, continuous_is_rated=False)
XM430 = Module('XM430-W350 (12 V)', 12.0, 46 * RPM, 4.1, 0.0, 4.1, 2.5, 4.1 / 2.3, 2.3,
               standby_current=0.04, continuous_is_rated=False)

# The same at 14.8 V: XM540 12.9 N·m, 5.5 A, 37 rpm; XM430 4.8 N·m, 2.7 A, 57 rpm.
# The provisional continuous torques are kept (heating is not voltage-bound).
XM540_14V8 = Module('XM540-W270 (14.8 V)', 14.8, 37 * RPM, 12.9, 0.0, 12.9, 6.0, 12.9 / 5.5,
                    5.5, standby_current=0.04, continuous_is_rated=False)
XM430_14V8 = Module('XM430-W350 (14.8 V)', 14.8, 57 * RPM, 4.8, 0.0, 4.8, 2.5, 4.8 / 2.7, 2.7,
                    standby_current=0.04, continuous_is_rated=False)

DXL_VOLTAGES = (12.0, 14.8)


def modules(dxl_voltage=12.0):
    """Joint -> Module, with the DYNAMIXEL wrist at dxl_voltage (12 or 14.8 V)."""
    if dxl_voltage == 12.0:
        xm540, xm430 = XM540, XM430
    elif dxl_voltage == 14.8:
        xm540, xm430 = XM540_14V8, XM430_14V8
    else:
        raise ValueError(f'DYNAMIXEL voltage must be one of {DXL_VOLTAGES}')
    return {'joint1': X6, 'joint2': X8, 'joint3': X6,
            'joint4': xm540, 'joint5': xm540, 'joint6': xm430}


MODULES = modules(12.0)


def check(module, speeds, torques, dt):
    """Limits of one joint over a sampled move.

    speeds, torques: joint speed [rad/s] and torque [N·m] at each sample,
    dt apart. Returns a dict:
      line_use: largest |torque| / available torque at that speed, over
        the motoring samples (> 1: the module cannot follow),
      speed_use: largest |speed| / no-load speed,
      peak_use: largest |torque| / torque cap (peak torque or peak current),
      current_use: largest current / peak current,
      rms_use: RMS torque / continuous torque,
      peak_current, rms_current [A],
      worst: (speed, torque) of the sample with the largest line_use.
    """
    line_use, worst = 0.0, (0.0, 0.0)
    for w, tau in zip(speeds, torques):
        if w * tau <= 0.0:
            continue        # braking or standing: only the torque cap applies
        avail = module.available_torque(w)
        use = math.inf if avail <= 0.0 else abs(tau) / avail
        if use > line_use:
            line_use, worst = use, (w, tau)
    n = max(1, len(torques))
    rms = math.sqrt(sum(t * t for t in torques) / n)
    currents = [module.current(t) for t in torques]
    return {
        'line_use': line_use,
        'speed_use': max((abs(w) for w in speeds), default=0.0) / module.no_load_speed,
        'peak_use': max((abs(t) for t in torques), default=0.0) / module.torque_cap,
        'current_use': max(currents, default=0.0) / module.peak_current,
        'rms_use': rms / module.continuous_torque,
        'peak_current': max(currents, default=module.standby_current),
        'rms_current': math.sqrt(sum(c * c for c in currents) / n),
        'worst': worst,
        'duration': dt * len(torques),
    }


def supply_power(modules, speeds, torques):
    """Mechanical power [W] the motors deliver at one sample, per supply voltage.

    Only positive (motoring) power is counted; braking power is assumed
    to be absorbed (not fed back). Motor losses are not included, so the
    supply has to give more than this.
    """
    out = {}
    for m, w, tau in zip(modules, speeds, torques):
        out[m.voltage] = out.get(m.voltage, 0.0) + max(0.0, w * tau)
    return out


def over(result):
    """True if the module cannot give the move: speed line, torque or current."""
    return (result['line_use'] > 1.0 or result['peak_use'] > 1.0
            or result['current_use'] > 1.0)


def describe(joint, module, result):
    """One line: line use (worst sample), speed / peak / RMS use, currents.

    The RMS use is marked * when the continuous torque is provisional.
    """
    w, tau = result['worst']
    flag = ' OVER' if over(result) else ''
    star = '' if module.continuous_is_rated else '*'
    return (f'{joint[-1]} {module.name.split()[0]}: line {100 * result["line_use"]:3.0f}%{flag}'
            f' at {abs(w):.2f} rad/s {abs(tau):.2f} N·m'
            f' (can {module.available_torque(w, w * tau > 0):.2f})'
            f' | speed {100 * result["speed_use"]:3.0f}%'
            f' | peak {100 * result["peak_use"]:3.0f}%'
            f' | RMS {100 * result["rms_use"]:3.0f}%{star}'
            f' | I {result["peak_current"]:.2f}/{result["rms_current"]:.2f} A')

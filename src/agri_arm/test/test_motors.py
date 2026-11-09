import math

import pytest

from agri_arm.motors import MODULES, RPM, X8, XM430, XM540, check


def test_rmd_line_through_published_points():
    assert X8.speed_limit(0.0) == pytest.approx(158 * RPM)
    assert X8.speed_limit(43.0) == pytest.approx(127 * RPM)
    assert X8.available_torque(127 * RPM) == pytest.approx(43.0)
    assert X8.available_torque(0.0) == pytest.approx(105.12)   # capped by the peak current
    assert X8.current(43.0) == pytest.approx(17.9, abs=0.1)   # rated 17.6 A


def test_dynamixel_line_from_stall_to_no_load():
    assert XM540.available_torque(0.0) == pytest.approx(10.6)
    assert XM540.available_torque(30 * RPM) == pytest.approx(0.0)
    assert XM540.available_torque(15 * RPM) == pytest.approx(5.3)
    assert XM540.current(10.6) == pytest.approx(4.44)
    # Braking: only the peak torque limits.
    assert XM540.available_torque(3.0, motoring=False) == 10.6


def test_wrist_limits_leave_torque():
    """The URDF velocity limits leave a usable torque on the DYNAMIXEL joints."""
    assert XM540.available_torque(2.0) > 3.5
    assert XM430.available_torque(3.0) > 1.4
    assert XM540.available_torque(3.0) < 0.5            # the old 3.0 rad/s limit


def test_check_flags_a_move_over_the_line():
    speeds = [0.0, 1.0, 3.0, 1.0, 0.0]
    torques = [0.5, 0.6, 0.7, 0.6, 0.5]
    r = check(MODULES['joint5'], speeds, torques, 0.01)
    assert r['line_use'] > 1.0
    assert r['worst'] == (3.0, 0.7)
    assert r['speed_use'] == pytest.approx(3.0 / (30 * RPM))
    braking = check(MODULES['joint5'], speeds, [-t for t in torques], 0.01)
    assert braking['line_use'] < 0.1
    assert r['rms_current'] == pytest.approx(math.sqrt(sum(
        (0.04 + t / (10.6 / 4.4)) ** 2 for t in torques) / 5))


def test_peak_current_caps_the_torque():
    # 2.4 N·m/A x 43.8 A = 105 N·m, below the listed 120 N·m peak.
    assert X8.torque_cap == pytest.approx(105.12)
    r = check(X8, [0.1], [110.0], 0.01)
    assert r['current_use'] > 1.0 and r['peak_use'] > 1.0


def test_braking_is_not_on_the_speed_line():
    r = check(XM540, [2.0, 2.0], [-5.0, -5.0], 0.01)
    assert r['line_use'] == 0.0
    assert r['peak_use'] == pytest.approx(5.0 / XM540.torque_cap)

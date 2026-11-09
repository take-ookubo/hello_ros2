"""Re-check joint torques in MuJoCo with the motors' rotor inertia (armature).

Gazebo has no way to add the rotor inertia of a geared motor. On a real
joint it acts like extra inertia of (rotor inertia) x (gear ratio)^2 about
the joint axis, which can be much larger than the link itself. MuJoCo adds
it to the joint as "armature". This script loads the same URDF into MuJoCo
and runs the moves of dynamics_test in two ways:

1. Inverse dynamics: the torque each joint needs to follow the quintic
   trajectory exactly, with and without armature. It is compared with the
   rated (continuous) and peak torque of each module.
2. Forward simulation with the PID gains of controllers_effort.yaml and a
   stiffer set, with and without armature, at a 1 ms control step, to see
   whether the low Gazebo gains are still needed once armature is present.

With armature the inverse-dynamics torques are also checked against the
modules' speed-torque lines and currents (motors.py).

With --trajectories FILE it checks recorded MoveIt trajectories instead
of the test moves: harvest_demo writes them with trajectory_log:=FILE
(one JSON object per move). Moves made while holding a fruit get the
fruit's mass (--fruit-mass) at the tool on top of --payload.

Run it inside the container (no Gazebo or ROS graph needed):
  ros2 run agri_arm mujoco_dynamics [--payload 0.35] [--speeds 0.5 1.0]
  ros2 run agri_arm mujoco_dynamics --trajectories /tmp/harvest_traj.jsonl
"""
import argparse
import json
import math
import os

import mujoco
import numpy as np
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from agri_arm.dynamics_test import MOVES, duration_for, quintic
from agri_arm.motors import MODULES, check, describe, over, supply_power
from agri_arm.pose_cycle import JOINTS, MODULE_TORQUE

# Reflected rotor inertia at the joint [kg·m²].
# RMD: "Inertia" on the MYACTUATOR product pages (X8-P20-120: 1.5 kg·cm²,
# X6-P20-60: 0.66 kg·cm²), taken as motor-side rotor inertia, times the
# gear ratio 19.612 squared.
# DYNAMIXEL: ROBOTIS does not publish it. We use the values identified by
# Rhoban's BAM project for the similar MX-106 (0.0266, 225:1) and MX-64
# (0.0120, 200:1) and scale them by the gear ratio squared for XM540-W270
# (272.5:1) and XM430-W350 (353.5:1). The XM430 motor is smaller than the
# MX-64's, so joint6 is likely an overestimate.
RMD_RATIO = 19.612
ARMATURE = {
    'joint1': 0.66e-4 * RMD_RATIO**2,          # RMD-X6-P20-60, about 0.025
    'joint2': 1.5e-4 * RMD_RATIO**2,           # RMD-X8-P20-120, about 0.058
    'joint3': 0.66e-4 * RMD_RATIO**2,          # RMD-X6-P20-60
    'joint4': 0.0266 * (272.5 / 225) ** 2,     # XM540-W270, about 0.039
    'joint5': 0.0266 * (272.5 / 225) ** 2,     # XM540-W270
    'joint6': 0.0120 * (353.5 / 200) ** 2,     # XM430-W350, about 0.037
}
# Short-time peak torque [N·m]: RMD peak torque, DYNAMIXEL stall torque at 12 V.
PEAK_TORQUE = {'joint1': 50.0, 'joint2': 120.0, 'joint3': 50.0,
               'joint4': 10.6, 'joint5': 10.6, 'joint6': 4.1}

DT = 0.001      # physics and control step [s], as in Gazebo
TAIL = 0.5      # keep simulating after the trajectory ends [s]
SETTLE = 1.0    # hold the start pose before the move [s]
# Stiffer gains that only work with armature (joint2/joint3 unchanged).
STIFF = {'joint1': (800.0, 15.0), 'joint4': (150.0, 4.0),
         'joint5': (150.0, 4.0), 'joint6': (60.0, 2.0)}


def load_model(payload):
    share = get_package_share_directory('agri_arm')
    urdf = xacro.process_file(
        os.path.join(share, 'urdf', 'agri_arm.urdf.xacro'),
        # The lumped hand (same mass), so the model has only the arm joints.
        mappings={'payload': str(float(payload)), 'hand': 'false'}).toxml()
    # Keep the URDF inertias as they are and merge fixed links.
    urdf = urdf.replace('</robot>', '<mujoco><compiler discardvisual="true" '
                        'fusestatic="true" balanceinertia="false"/></mujoco></robot>')
    model = mujoco.MjModel.from_xml_string(urdf)
    model.opt.timestep = DT
    model.opt.integrator = mujoco.mjtIntegrator.mjINT_EULER
    names = [model.joint(i).name for i in range(model.njnt)]
    if names != JOINTS:
        raise RuntimeError(f'unexpected joints in MuJoCo model: {names}')
    return model


def set_armature(model, on):
    model.dof_armature[:] = [ARMATURE[j] if on else 0.0 for j in JOINTS]


def load_gains():
    path = os.path.join(get_package_share_directory('agri_arm'),
                        'config', 'controllers_effort.yaml')
    with open(path) as f:
        gains = yaml.safe_load(f)['arm_controller']['ros__parameters']['gains']
    return {j: dict(gains[j]) for j in JOINTS}


def stiff_gains(base):
    gains = {j: dict(g) for j, g in base.items()}
    for j, (p, d) in STIFF.items():
        gains[j]['p'], gains[j]['d'] = p, d
    return gains


def trajectory(start, goal, scale):
    """Quintic reference sampled at DT: (T, positions, velocities, accelerations)."""
    start, goal = np.array(start), np.array(goal)
    T = duration_for(start, goal, scale)
    t = np.arange(0.0, T + DT / 2, DT)
    p, v, a = quintic(np.clip(t / T, 0.0, 1.0))
    d = goal - start
    return (T, start + np.outer(p, d), np.outer(v, d) / T, np.outer(a, d) / T**2)


def inverse_torque(model, data, q, qd, qdd, damping, friction):
    """Torque that follows the reference exactly, including joint damping and friction."""
    tau = np.empty_like(q)
    for k in range(len(q)):
        data.qpos[:], data.qvel[:], data.qacc[:] = q[k], qd[k], qdd[k]
        mujoco.mj_inverse(model, data)
        tau[k] = data.qfrc_inverse
    # The URDF damping and Coulomb friction are left out of mj_inverse
    # (dof_damping / dof_frictionloss are zero there) and added here.
    tau += damping * qd + friction * np.sign(qd)
    return tau


def simulate(model, gains, q, qd, offset=None):
    """PID like joint_trajectory_controller, torque clipped to u_clamp.

    The arm starts at q[0] (+ offset), holds it for SETTLE, follows the
    reference and holds the end for TAIL. Returns the applied torques,
    position errors and velocities from the start of the move, or None
    if the arm ran away.
    """
    data = mujoco.MjData(model)
    data.qpos[:] = q[0] if offset is None else q[0] + offset
    mujoco.mj_forward(model, data)
    p = np.array([gains[j]['p'] for j in JOINTS])
    i = np.array([gains[j]['i'] for j in JOINTS])
    d = np.array([gains[j]['d'] for j in JOINTS])
    i_lim = np.array([gains[j]['i_clamp_max'] for j in JOINTS])
    u_lim = np.array([gains[j]['u_clamp_max'] for j in JOINTS])
    hold = int(SETTLE / DT)
    tail = int(TAIL / DT)
    ref_q = np.vstack([np.repeat(q[:1], hold, 0), q, np.repeat(q[-1:], tail, 0)])
    ref_qd = np.vstack([np.zeros((hold, 6)), qd, np.zeros((tail, 6))])
    integral = np.zeros(6)
    taus, errs, vels = [], [], []
    for k in range(len(ref_q)):
        e = ref_q[k] - data.qpos
        ed = ref_qd[k] - data.qvel
        integral = np.clip(integral + i * e * DT, -i_lim, i_lim)
        tau = np.clip(p * e + integral + d * ed, -u_lim, u_lim)
        data.qfrc_applied[:] = tau
        mujoco.mj_step(model, data)
        if not np.all(np.isfinite(data.qpos)) or np.max(np.abs(e)) > 1.0:
            return None
        if k >= hold:      # record from the start of the move, as in Gazebo
            taus.append(tau)
            errs.append(e)
            vels.append(data.qvel.copy())
    return np.array(taus), np.array(errs), np.array(vels)


def home_hold(model, gains):
    """Hold home after a 0.02 rad nudge on every joint.

    At home joint1, joint4 and joint6 share one vertical axis and the
    inertia about it is tiny without armature; this is where Gazebo needed
    the low gains. Returns (max error, max speed) over the last second,
    or None if the arm ran away.
    """
    hold = np.zeros((int(3.0 / DT), 6))
    result = simulate(model, gains, hold, hold, offset=np.full(6, 0.02))
    if result is None:
        return None
    _, errs, vels = result
    last = int(1.0 / DT)
    return np.max(np.abs(errs[-last:]), 0), np.max(np.abs(vels[-last:]), 0)


def resample(times, q, qd, qdd=None, step=0.005):
    """Resample a trajectory every step (and at its end) the way
    joint_trajectory_controller interpolates it: quintic segments when the
    points have accelerations, cubic when they have only velocities.
    Returns (positions, velocities, accelerations)."""
    times, q, qd = np.asarray(times, float), np.asarray(q, float), np.asarray(qd, float)
    cubic = qdd is None
    qdd = np.zeros_like(q) if cubic else np.asarray(qdd, float)
    t = np.arange(times[0], times[-1], step)
    t = np.append(t, times[-1])
    k = np.clip(np.searchsorted(times, t, side='right') - 1, 0, len(times) - 2)
    h = (times[k + 1] - times[k])[:, None]
    tau = t[:, None] - times[k][:, None]
    p0, v0, a0 = q[k], qd[k], qdd[k]
    p1, v1, a1 = q[k + 1], qd[k + 1], qdd[k + 1]
    # p(tau) = sum c_i tau^i with the end conditions; a0 = a1 = 0 and the
    # tau^5 term vanishing gives back the cubic.
    c0, c1, c2 = p0, v0, a0 / 2
    d = p1 - p0 - v0 * h - a0 * h**2 / 2
    e = v1 - v0 - a0 * h
    f = a1 - a0
    c3 = (20 * d - 8 * e * h + f * h**2) / (2 * h**3)
    c4 = (-30 * d + 14 * e * h - 2 * f * h**2) / (2 * h**4)
    c5 = (12 * d - 6 * e * h + f * h**2) / (2 * h**5)
    if cubic:
        # cubic Hermite through positions and velocities
        c2 = (3 * d - e * h) / h**2
        c3 = (-2 * d + e * h) / h**3
        c4 = c5 = np.zeros_like(c3)
    pos = c0 + c1 * tau + c2 * tau**2 + c3 * tau**3 + c4 * tau**4 + c5 * tau**5
    vel = c1 + 2 * c2 * tau + 3 * c3 * tau**2 + 4 * c4 * tau**3 + 5 * c5 * tau**4
    acc = 2 * c2 + 6 * c3 * tau + 12 * c4 * tau**2 + 20 * c5 * tau**3
    return pos, vel, acc


def motor_report(tau, qd, dt, indent='    '):
    """Lines of the speed-torque and current check for one move, and
    whether any joint is over its speed-torque line.

    Per joint: line = largest torque / torque the module can give at that
    speed (motoring samples), speed = of no-load speed, peak = of the
    torque cap (peak torque, or peak current x torque constant), RMS = of
    continuous torque (* provisional), I = peak / RMS current. OVER: over
    the line, the torque cap or the peak current."""
    lines, any_over = [], False
    for i, j in enumerate(JOINTS):
        result = check(MODULES[j], qd[:, i], tau[:, i], dt)
        any_over |= over(result)
        lines.append(indent + describe(j, MODULES[j], result))
    modules = [MODULES[j] for j in JOINTS]
    peak = {}
    for k in range(len(tau)):
        for volts, watts in supply_power(modules, qd[k], tau[k]).items():
            peak[volts] = max(peak.get(volts, 0.0), watts)
    lines.append(indent + 'peak mechanical power, losses not included: '
                 + ', '.join(f'{v:.0f} V {w:.1f} W ({w / v:.2f} A)'
                             for v, w in sorted(peak.items(), reverse=True)))
    return lines, any_over


def check_trajectories(path, payload, fruit_mass):
    """Inverse dynamics with armature and the motor check for recorded moves."""
    models = {}

    def model_for(mass):
        if mass not in models:
            m = load_model(mass)
            damping, friction = m.dof_damping.copy(), m.dof_frictionloss.copy()
            m.dof_damping[:] = 0.0
            m.dof_frictionloss[:] = 0.0
            set_armature(m, True)
            models[mass] = (m, mujoco.MjData(m), damping, friction)
        return models[mass]

    over_moves = []
    with open(path) as f:
        moves = [json.loads(line) for line in f if line.strip()]
    print(f'{len(moves)} moves from {path}, payload {payload} kg, '
          f'fruit {fruit_mass} kg while carrying, armature on')
    for move in moves:
        order = [move['joints'].index(j) for j in JOINTS]
        pts = move['points']
        times = [p['t'] for p in pts]
        if len(times) < 2:
            continue
        q = np.array([[p['q'][i] for i in order] for p in pts])
        qd = np.array([[p['qd'][i] for i in order] for p in pts])
        qdd = (np.array([[p['qdd'][i] for i in order] for p in pts])
               if all(p.get('qdd') for p in pts) else None)
        pos, vel, acc = resample(times, q, qd, qdd)
        mass = payload + (fruit_mass if move.get('carrying') else 0.0)
        model, data, damping, friction = model_for(mass)
        tau = inverse_torque(model, data, pos, vel, acc, damping, friction)
        print(f'\n== {move["name"]} ({times[-1]:.2f} s'
              + (', carrying a fruit' if move.get('carrying') else '') + ')')
        lines, over = motor_report(tau, vel, 0.005)
        print('\n'.join(lines))
        if over:
            over_moves.append(move['name'])
    print('\nmoves over a speed-torque line: ' + (', '.join(over_moves) or 'none'))


def pct(x, j, table):
    return f'{x:.1f}({100 * x / table[j]:.0f}%)'


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--payload', type=float, default=0.0,
                        help='extra mass at the tool tip [kg]')
    parser.add_argument('--speeds', type=float, nargs='+', default=[0.25, 0.5, 1.0],
                        help='fractions of the URDF velocity limit')
    parser.add_argument('--trajectories', help='JSON lines from harvest_demo trajectory_log')
    parser.add_argument('--fruit-mass', type=float, default=0.15,
                        help='[kg] added while a recorded move carries a fruit')
    args = parser.parse_args()
    if not (math.isfinite(args.payload) and args.payload >= 0.0):
        parser.error('--payload must be a finite value >= 0')
    if not all(math.isfinite(x) and 0.0 < x <= 2.0 for x in args.speeds):
        parser.error('--speeds must be in (0, 2]')

    if args.trajectories:
        check_trajectories(args.trajectories, args.payload, args.fruit_mass)
        return

    model = load_model(args.payload)
    damping = model.dof_damping.copy()
    friction = model.dof_frictionloss.copy()
    # Inverse dynamics runs on a copy without passive forces (added by hand).
    inv_model = load_model(args.payload)
    inv_model.dof_damping[:] = 0.0
    inv_model.dof_frictionloss[:] = 0.0
    inv_data = mujoco.MjData(inv_model)
    base = load_gains()
    controllers = {'gazebo gains': base, 'stiff gains': stiff_gains(base)}

    print(f'payload {args.payload} kg, armature [kg·m²]: '
          + ' '.join(f'{j[-1]}:{ARMATURE[j]:.3f}' for j in JOINTS))
    print('torque is peak N·m (% of MODULE_TORQUE: RMD rated, DYNAMIXEL provisional '
          'continuous value); RMS over the move in [] (same base); '
          '"peak" shows % of the short-time peak torque.')
    for name, start, goal in MOVES:
        for scale in args.speeds:
            T, q, qd, qdd = trajectory(start, goal, scale)
            print(f'\n== {name}, speed {scale:.2f} (T {T:.2f} s, '
                  f'peak accel rad/s²: '
                  + ' '.join(f'{x:.1f}' for x in np.max(np.abs(qdd), 0)) + ')')
            for arm in (False, True):
                set_armature(inv_model, arm)
                tau = inverse_torque(inv_model, inv_data, q, qd, qdd, damping, friction)
                peak = np.max(np.abs(tau), 0)
                rms = np.sqrt(np.mean(tau**2, 0))
                print(f'  inverse, armature {"on " if arm else "off"}: '
                      + ' '.join(
                          f'{i + 1}:{pct(peak[i], j, MODULE_TORQUE)}'
                          f'[{100 * rms[i] / MODULE_TORQUE[j]:.0f}%]'
                          f'/peak{100 * peak[i] / PEAK_TORQUE[j]:.0f}%'
                          for i, j in enumerate(JOINTS)))
                if arm:
                    lines, _ = motor_report(tau, qd, DT)
                    print('\n'.join(lines))
            for arm in (False, True):
                set_armature(model, arm)
                for label, gains in controllers.items():
                    result = simulate(model, gains, q, qd)
                    head = f'  sim {label}, armature {"on " if arm else "off"}:'
                    if result is None:
                        print(f'{head} UNSTABLE')
                        continue
                    taus, errs, _ = result
                    peak = np.max(np.abs(taus), 0)
                    sat = np.mean(np.abs(taus) >= 0.98 * np.array(
                        [gains[j]['u_clamp_max'] for j in JOINTS]), 0)
                    print(f'{head} peak N·m '
                          + ' '.join(f'{i + 1}:{peak[i]:.1f}'
                                     + (f'(sat{100 * sat[i]:.0f}%)' if sat[i] else '')
                                     for i in range(6))
                          + ' | max err rad '
                          + ' '.join(f'{x:.3f}' for x in np.max(np.abs(errs), 0)))

    print('\n== home hold after a 0.02 rad nudge (last 1 s: max err rad / max speed rad/s)')
    for arm in (False, True):
        set_armature(model, arm)
        for label, gains in controllers.items():
            result = home_hold(model, gains)
            head = f'  {label}, armature {"on " if arm else "off"}:'
            if result is None:
                print(f'{head} UNSTABLE (ran away)')
                continue
            err, vel = result
            state = 'oscillating' if np.max(vel) > 0.05 else 'stable'
            print(f'{head} {state} | '
                  + ' '.join(f'{i + 1}:{err[i]:.4f}/{vel[i]:.2f}' for i in range(6)))


if __name__ == '__main__':
    main()

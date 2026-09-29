#!/usr/bin/env python3
"""Sanity-check the IMU -> Madgwick -> EKF pipeline from a recorded bag.

Current configuration:
- Raw IMU topic: /imu_broadcaster/imu
- Madgwick output: /imu/data
- EKF output: /odometry/filtered
- imu0_relative: false
- EKF fuses roll, pitch, yaw
- EKF fuses angular velocity x, y, z
- URDF has base_link -> imu = 180 deg rotation about X

The script:
1. Sorts all messages by header timestamp.
2. Converts IMU-frame data into base_link for EKF comparisons.
3. SLERPs Madgwick orientation onto exact EKF timestamps.
4. Checks Madgwick tilt against accelerometer.
5. Checks EKF angular velocity against gyro.
6. Checks EKF absolute orientation against Madgwick.
7. Estimates stationary gyro covariance.
8. Estimates stationary accelerometer covariance.
9. Estimates stationary Madgwick orientation covariance.
10. Prints ready-to-copy sensor_msgs/Imu covariance arrays.
11. Plots EKF-minus-Madgwick orientation residuals.
12. Plots EKF-minus-gyro angular-rate residuals.
13. Checks for short EKF-only orientation step spikes that can look like RViz jitter.

Usage:
    python3 check_ekf.py
    python3 check_ekf.py <bag_path>
"""

import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# Topics / paths
RAW = '/imu_broadcaster/imu'
MAD = '/imu/data'
EKF = '/odometry/filtered'

SCRIPT_DIR = Path(__file__).resolve().parent

# EKF test configuration
IMU_RELATIVE = False
FUSE_ANGULAR_RATE = True

# Covariance estimation settings
COV_GYRO_MAX = 0.05          # rad/s
COV_ACCEL_G_TOL = 0.15       # m/s^2

# Trim the start/end of the stationary segment to avoid picking up transition motion.
COV_EDGE_TRIM = 0.50         # seconds

# Warn if the longest stationary interval is shorter than this.
COV_MIN_DURATION = 2.0       # seconds


# Load bag
def find_bag():
    if len(sys.argv) > 1:
        return sys.argv[1]

    folders = sorted(
        p.parent
        for p in SCRIPT_DIR.glob('*/metadata.yaml')
    )
    if folders:
        return str(folders[-1])

    files = sorted(SCRIPT_DIR.glob('*.mcap'))
    if files:
        return str(files[-1])

    sys.exit(
        f'No bag found in {SCRIPT_DIR}. '
        'Record one there or pass a path.'
    )


def read_bag(path):
    reader = SequentialReader()
    reader.open(
        StorageOptions(
            uri=path,
            storage_id='mcap'
        ),
        ConverterOptions('', '')
    )

    topic_types = {
        t.name: t.type
        for t in reader.get_all_topics_and_types()
    }

    out = {
        RAW: [],
        MAD: [],
        EKF: []
    }

    while reader.has_next():
        topic, data, _ = reader.read_next()

        if topic not in out:
            continue

        msg_type = get_message(topic_types[topic])
        out[topic].append(
            deserialize_message(data, msg_type)
        )
    return out


# Basic helpers
def stamp(msg):
    return (
        msg.header.stamp.sec
        + msg.header.stamp.nanosec * 1e-9
    )


def q_arr(q):
    """ROS quaternion -> numpy [w, x, y, z]."""
    return np.array([
        q.w,
        q.x,
        q.y,
        q.z
    ], dtype=float)


def q_normalize(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n == 0:
        raise ValueError('Zero-length quaternion')
    return q / n


def q_conj(q):
    return np.array([
        q[0],
        -q[1],
        -q[2],
        -q[3]
    ])


def q_inv(q):
    q = np.asarray(q, dtype=float)
    norm_sq = np.dot(q, q)

    if norm_sq == 0:
        raise ValueError('Cannot invert zero quaternion')
    return q_conj(q) / norm_sq


def q_mul(a, b):
    """Hamilton product using [w, x, y, z]."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b

    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])


def q_rotate_vector(q, v):
    """Rotate vector v using quaternion q."""
    q = q_normalize(q)
    vq = np.array([
        0.0,
        v[0],
        v[1],
        v[2]
    ])

    result = q_mul(
        q_mul(q, vq),
        q_inv(q)
    )
    return result[1:]


def q_to_rpy(q):
    """Quaternion -> roll, pitch, yaw in degrees."""
    q = q_normalize(q)
    w, x, y, z = q

    roll = np.arctan2(
        2.0 * (w*x + y*z),
        1.0 - 2.0 * (x*x + y*y)
    )

    pitch = np.arcsin(
        np.clip(2.0 * (w*y - z*x), -1.0, 1.0)
    )

    yaw = np.arctan2(
        2.0 * (w*z + x*y),
        1.0 - 2.0 * (y*y + z*z)
    )

    return np.degrees([roll, pitch, yaw])


def q_angle_deg(a, b):
    """Smallest 3D rotation between two orientations."""
    a = q_normalize(a)
    b = q_normalize(b)

    # q and -q represent the same orientation.
    dot = np.clip(abs(np.dot(a, b)), 0.0, 1.0)

    return np.degrees(2.0 * np.arccos(dot))


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def verdict(ok):
    return 'PASS' if ok else 'CHECK'


# Quaternion interpolation
def q_slerp(q0, q1, u):
    """
    Spherical linear interpolation.
    u = 0 -> q0
    u = 1 -> q1
    """
    q0 = q_normalize(q0)
    q1 = q_normalize(q1)

    dot = np.dot(q0, q1)

    # Take shortest path through quaternion space.
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    dot = np.clip(dot, -1.0, 1.0)

    # Nearly identical -> linear interpolation is safer.
    if dot > 0.9995:
        q = q0 + u * (q1 - q0)
        return q_normalize(q)

    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * u

    s0 = np.sin(theta_0 - theta) / sin_theta_0
    s1 = np.sin(theta) / sin_theta_0

    return q_normalize(s0*q0 + s1*q1)


def interpolate_quaternions(t_ref, q_ref, t_query):
    """
    SLERP q_ref(t_ref) onto t_query.
    t_ref must be monotonically increasing.
    """
    out = []

    for tq in t_query:
        if tq <= t_ref[0]:
            out.append(q_ref[0])
            continue

        if tq >= t_ref[-1]:
            out.append(q_ref[-1])
            continue

        right = np.searchsorted(t_ref, tq)
        left = right - 1

        t_left = t_ref[left]
        t_right = t_ref[right]
        dt = t_right - t_left

        if dt <= 0:
            out.append(q_ref[left])
            continue

        u = (tq - t_left) / dt
        out.append(
            q_slerp(q_ref[left], q_ref[right], u)
        )

    return np.array(out)


# Covariance helpers
def quaternion_mean(quaternions):
    """
    Markley quaternion average.
    Handles q / -q ambiguity.
    """
    A = np.zeros((4, 4))

    for q in quaternions:
        q = q_normalize(q)
        A += np.outer(q, q)

    A /= len(quaternions)

    eigenvalues, eigenvectors = np.linalg.eigh(A)
    q_mean = eigenvectors[:, np.argmax(eigenvalues)]

    return q_normalize(q_mean)


def quaternion_to_rotvec(q):
    """
    Quaternion error -> 3D rotation vector.
    Result is [rx, ry, rz] in radians.
    """
    q = q_normalize(q)

    # Use shortest equivalent quaternion.
    if q[0] < 0:
        q = -q

    w = np.clip(q[0], -1.0, 1.0)
    v = q[1:]
    sin_half = np.linalg.norm(v)

    if sin_half < 1e-12:
        # Small angle approximation: q ~= [1, r/2]
        return 2.0 * v

    angle = 2.0 * np.arctan2(sin_half, w)
    axis = v / sin_half

    return axis * angle


def find_longest_true_run(mask):
    """
    Return start/end indices of longest contiguous True run.
    Returned end index is exclusive.
    """
    best_start = None
    best_end = None
    best_len = 0
    start = None

    for i, value in enumerate(mask):
        if value and start is None:
            start = i

        is_last = i == len(mask) - 1

        if (not value or is_last) and start is not None:
            end = i + 1 if (value and is_last) else i
            length = end - start

            if length > best_len:
                best_start = start
                best_end = end
                best_len = length

            start = None

    return best_start, best_end


def format_ros_array(cov):
    return (
        '['
        + ', '.join(f'{x:.10e}' for x in cov.reshape(-1))
        + ']'
    )


def print_covariance(name, cov, units):
    print()
    print(f'{name} [{units}]:')

    with np.printoptions(precision=10, suppress=False):
        print(cov)
        print('    diagonal:')
        print(np.diag(cov))
        print('    standard deviation:')
        print(np.sqrt(np.diag(cov)))

    print('    ROS row-major:')
    print('    ' + format_ros_array(cov))


def check_monotonic(name, t):
    dt = np.diff(t)
    backwards = np.sum(dt < 0)
    repeated = np.sum(dt == 0)

    print(
        f'{name}: '
        f'{len(t)} samples, '
        f'{backwards} backwards jumps, '
        f'{repeated} repeated timestamps'
    )


# Static IMU mounting transform
#
# URDF: <origin xyz="0 0 0" rpy="${pi} 0 0"/>
#
# 180 deg about X:
#   IMU X ->  base_link X
#   IMU Y -> -base_link Y
#   IMU Z -> -base_link Z
#
# Quaternion order here is [w, x, y, z].
Q_BASE_IMU = q_normalize(
    np.array([0.0, 1.0, 0.0, 0.0])
)


# Load bag
bag_path = find_bag()
print(f'Reading bag: {bag_path}\n')

data = read_bag(bag_path)

for topic in (RAW, MAD, EKF):
    if not data[topic]:
        sys.exit(
            f'No messages on {topic}. '
            f'Check with: ros2 bag info {bag_path}'
        )

# Sort by message HEADER timestamp.
# Rosbag storage order is not guaranteed to be header-stamp order.
for topic in (RAW, MAD, EKF):
    data[topic] = sorted(data[topic], key=stamp)

# Common time origin
t0 = min(
    stamp(data[RAW][0]),
    stamp(data[MAD][0]),
    stamp(data[EKF][0])
)


# Raw IMU
tr = np.array([stamp(m) for m in data[RAW]]) - t0

acc_imu = np.array([
    [
        m.linear_acceleration.x,
        m.linear_acceleration.y,
        m.linear_acceleration.z
    ]
    for m in data[RAW]
])

gyr_imu = np.array([
    [
        m.angular_velocity.x,
        m.angular_velocity.y,
        m.angular_velocity.z
    ]
    for m in data[RAW]
])

# Rotate vectors into base_link
acc_base = np.array([
    q_rotate_vector(Q_BASE_IMU, v)
    for v in acc_imu
])

gyr_base = np.array([
    q_rotate_vector(Q_BASE_IMU, v)
    for v in gyr_imu
])

# Accelerometer-derived roll/pitch
acc_roll = np.degrees(
    np.arctan2(acc_base[:, 1], acc_base[:, 2])
)

acc_pitch = np.degrees(
    np.arctan2(
        -acc_base[:, 0],
        np.hypot(acc_base[:, 1], acc_base[:, 2])
    )
)


# Madgwick
tm = np.array([stamp(m) for m in data[MAD]]) - t0

qm_imu = np.array([
    q_normalize(q_arr(m.orientation))
    for m in data[MAD]
])

# Convert world -> imu into world -> base_link:
#   q_world_imu  = q_world_base * q_base_imu
#   q_world_base = q_world_imu * inverse(q_base_imu)
qm_base = np.array([
    q_normalize(
        q_mul(q, q_inv(Q_BASE_IMU))
    )
    for q in qm_imu
])

rpy_m_base = np.array([q_to_rpy(q) for q in qm_base])


# EKF
te = np.array([stamp(m) for m in data[EKF]]) - t0

qe = np.array([
    q_normalize(q_arr(m.pose.pose.orientation))
    for m in data[EKF]
])

rpy_e = np.array([q_to_rpy(q) for q in qe])

rate_e = np.array([
    [
        m.twist.twist.angular.x,
        m.twist.twist.angular.y,
        m.twist.twist.angular.z
    ]
    for m in data[EKF]
])


# Timestamp diagnostics
print('--- Timestamp diagnostic ---')
check_monotonic('RAW', tr)
check_monotonic('MAD', tm)
check_monotonic('EKF', te)

print(f'RAW range: {tr[0]:.3f} -> {tr[-1]:.3f} s')
print(f'MAD range: {tm[0]:.3f} -> {tm[-1]:.3f} s')
print(f'EKF range: {te[0]:.3f} -> {te[-1]:.3f} s')
print()


# SLERP Madgwick onto EKF timestamps
qm_at_ekf = interpolate_quaternions(tm, qm_base, te)
rpy_m_at_ekf = np.array([q_to_rpy(q) for q in qm_at_ekf])


# Initial quaternion diagnostic
initial_error = q_angle_deg(qe[0], qm_at_ekf[0])

print('--- Initial quaternion diagnostic ---')
print('Madgwick interpolated q:', qm_at_ekf[0])
print('EKF q0               :', qe[0])
print(f'Initial ABS error      : {initial_error:.6f} deg')
print()


# [1] Madgwick vs accelerometer
still = (
    np.abs(np.linalg.norm(acc_base, axis=1) - 9.81) < 0.3
) & (
    np.linalg.norm(gyr_base, axis=1) < 0.2
)

m_roll_at_raw = np.degrees(
    np.interp(
        tr,
        tm,
        np.unwrap(np.radians(rpy_m_base[:, 0]))
    )
)

m_pitch_at_raw = np.interp(tr, tm, rpy_m_base[:, 1])

if still.sum() > 0:
    roll_err = wrap_deg(m_roll_at_raw[still] - acc_roll[still])
    pitch_err = wrap_deg(m_pitch_at_raw[still] - acc_pitch[still])

    roll_rms = np.sqrt(np.mean(roll_err ** 2))
    pitch_rms = np.sqrt(np.mean(pitch_err ** 2))
else:
    roll_rms = np.nan
    pitch_rms = np.nan

madgwick_ok = roll_rms < 3.0 and pitch_rms < 3.0

print(
    f'[1] Madgwick vs accel ({still.sum()} still samples)            '
    f'{verdict(madgwick_ok)}'
)
print(
    f'    roll RMS {roll_rms:.2f} deg, '
    f'pitch RMS {pitch_rms:.2f} deg (want < 3)'
)


# [2] EKF angular rate vs gyro
if FUSE_ANGULAR_RATE:
    gyr_at_ekf = np.array([
        np.interp(te, tr, gyr_base[:, i])
        for i in range(3)
    ]).T

    rate_err = gyr_at_ekf - rate_e
    rms_all = np.sqrt(np.mean(rate_err ** 2, axis=0))

    slow = np.linalg.norm(gyr_at_ekf, axis=1) < 0.5

    if slow.sum() > 0:
        rms_slow = np.sqrt(np.mean(rate_err[slow] ** 2, axis=0))
    else:
        rms_slow = np.full(3, np.nan)

    rate_ok = (
        np.nanmax(rms_all) < 0.15
        and np.nanmax(rms_slow) < 0.02
    )

    print(
        '[2] EKF rate vs gyro                                  '
        f'{verdict(rate_ok)}'
    )
    print(f'    RMS all (x,y,z):  {np.round(rms_all, 6)} rad/s')
    print(f'    RMS slow motion:  {np.round(rms_slow, 6)} rad/s')

    # Positive residual = EKF - measurement
    rate_residual = rate_e - gyr_at_ekf
    rate_abs_p95 = np.percentile(np.abs(rate_residual), 95, axis=0)
    rate_abs_max = np.max(np.abs(rate_residual), axis=0)

    print(f'    95th |EKF - gyro|: {np.round(rate_abs_p95, 6)} rad/s')
    print(f'    max  |EKF - gyro|: {np.round(rate_abs_max, 6)} rad/s')
else:
    print(
        '[2] EKF rate vs gyro                                  '
        'N/A'
    )
    print('    angular velocity fusion is disabled')

    gyr_at_ekf = None
    rate_err = None
    rate_residual = None


# [3] Absolute EKF vs Madgwick orientation
ang_abs = np.array([
    q_angle_deg(qe[i], qm_at_ekf[i])
    for i in range(len(te))
])

mean_ang = np.mean(ang_abs)
median_ang = np.median(ang_abs)
p95_ang = np.percentile(ang_abs, 95)
p99_ang = np.percentile(ang_abs, 99)
max_ang = np.max(ang_abs)
i_max = np.argmax(ang_abs)

orientation_ok = mean_ang < 2.0 and p95_ang < 5.0

print(
    '[3] EKF vs Madgwick ABSOLUTE orientation               '
    f'{verdict(orientation_ok)}'
)
print(f'    mean       {mean_ang:.6f} deg')
print(f'    median     {median_ang:.6f} deg')
print(f'    95th pct   {p95_ang:.6f} deg')
print(f'    99th pct   {p99_ang:.6f} deg')
print(f'    max        {max_ang:.6f} deg at t = {te[i_max]:.3f} s')

# Euler component residuals are useful for identifying which visible axis
# is twitching in RViz, but do not trust them close to pitch = +/-90 deg.
orientation_rpy_residual = wrap_deg(rpy_e - rpy_m_at_ekf)

gimbal_eval = (
    np.abs(rpy_e[:, 1]) > 80.0
) | (
    np.abs(rpy_m_at_ekf[:, 1]) > 80.0
)
non_gimbal = ~gimbal_eval

if np.any(non_gimbal):
    orientation_rpy_rms = np.sqrt(
        np.mean(orientation_rpy_residual[non_gimbal] ** 2, axis=0)
    )
    orientation_rpy_p95 = np.percentile(
        np.abs(orientation_rpy_residual[non_gimbal]),
        95,
        axis=0
    )
else:
    orientation_rpy_rms = np.full(3, np.nan)
    orientation_rpy_p95 = np.full(3, np.nan)

print(
    '    non-gimbal Euler residual RMS '
    f'(roll,pitch,yaw): {np.round(orientation_rpy_rms, 6)} deg'
)
print(
    '    non-gimbal Euler |residual| p95 '
    f'(roll,pitch,yaw): {np.round(orientation_rpy_p95, 6)} deg'
)


# RViz jitter / transient diagnostic
# If the EKF makes a large one-frame rotation that is not present in the
# time-aligned Madgwick orientation, that is a strong candidate for
# visible RViz jitter.
ekf_step_deg = np.zeros(len(te))
madgwick_step_deg = np.zeros(len(te))

for i in range(1, len(te)):
    ekf_step_deg[i] = q_angle_deg(qe[i - 1], qe[i])
    madgwick_step_deg[i] = q_angle_deg(qm_at_ekf[i - 1], qm_at_ekf[i])

step_excess_deg = ekf_step_deg - madgwick_step_deg
i_step = int(np.argmax(step_excess_deg))

print()
print('--- RViz jitter / transient diagnostic ---')
print(f'Largest EKF one-step rotation: {np.max(ekf_step_deg):.6f} deg')
print(
    'Largest Madgwick one-step rotation at EKF timestamps: '
    f'{np.max(madgwick_step_deg):.6f} deg'
)
print(
    'Largest EKF step excess over Madgwick: '
    f'{step_excess_deg[i_step]:.6f} deg at t = {te[i_step]:.3f} s'
)
print(f'    EKF step      = {ekf_step_deg[i_step]:.6f} deg')
print(f'    Madgwick step = {madgwick_step_deg[i_step]:.6f} deg')
print(f'    quat error    = {ang_abs[i_step]:.6f} deg')
print(f'    gimbal region = {bool(gimbal_eval[i_step])}')


# Max-error diagnostic
print()
print('--- Max orientation error diagnostic ---')
print(f't = {te[i_max]:.6f} s')
print(f'orientation error = {ang_abs[i_max]:.6f} deg')
print('EKF q      =', qe[i_max])
print('Madgwick q =', qm_at_ekf[i_max])

print()
print('Samples around max error:')

for n in range(max(0, i_max - 3), min(len(te), i_max + 4)):
    if FUSE_ANGULAR_RATE:
        rate_text = (
            '  rate_res='
            + np.array2string(
                rate_residual[n],
                precision=5,
                suppress_small=False
            )
        )
    else:
        rate_text = ''

    print(
        f't={te[n]:8.3f}  '
        f'err={ang_abs[n]:8.6f} deg  '
        f'ekf_step={ekf_step_deg[n]:8.4f} deg  '
        f'mad_step={madgwick_step_deg[n]:8.4f} deg  '
        f'pitch={rpy_e[n, 1]:8.3f} deg'
        f'{rate_text}'
    )


# [4] EKF drift over first 3 seconds
n = np.searchsorted(te, te[0] + 3.0)

if n > 10:
    drift = wrap_deg(rpy_e[n - 1] - rpy_e[0])
    drift_ok = abs(drift[0]) < 0.5 and abs(drift[1]) < 0.5

    print(
        '[4] EKF drift over first 3 s                          '
        f'{verdict(drift_ok)}'
    )
    print(
        f'    roll {drift[0]:.3f}, '
        f'pitch {drift[1]:.3f}, '
        f'yaw {drift[2]:.3f} deg'
    )


# [5] Stationary covariance estimation
#
# IMPORTANT: estimate sensor_msgs/Imu covariance in the IMU frame,
# so use acc_imu, gyr_imu and qm_imu rather than base_link data.
print()
print('--- Stationary covariance estimation ---')

cov_stationary = (
    np.abs(np.linalg.norm(acc_imu, axis=1) - 9.81) < COV_ACCEL_G_TOL
) & (
    np.linalg.norm(gyr_imu, axis=1) < COV_GYRO_MAX
)

cov_i0, cov_i1 = find_longest_true_run(cov_stationary)

gyro_cov = None
accel_cov = None
orientation_cov = None

if cov_i0 is None:
    print('No contiguous stationary interval found.')
else:
    raw_start = tr[cov_i0]
    raw_end = tr[cov_i1 - 1]
    raw_duration = raw_end - raw_start

    print(
        f'Longest stationary candidate: '
        f'{raw_start:.3f} -> {raw_end:.3f} s ({raw_duration:.3f} s)'
    )

    if raw_duration < COV_MIN_DURATION:
        print(f'WARNING: interval shorter than {COV_MIN_DURATION:.1f} s.')

    cov_start = raw_start + COV_EDGE_TRIM
    cov_end = raw_end - COV_EDGE_TRIM

    if cov_end <= cov_start:
        print('WARNING: interval too short to trim. Using full interval.')
        cov_start = raw_start
        cov_end = raw_end

    raw_cov_mask = (
        (tr >= cov_start)
        & (tr <= cov_end)
        & cov_stationary
    )

    print(f'Covariance interval:       {cov_start:.3f} -> {cov_end:.3f} s')
    print(f'Raw stationary samples:    {raw_cov_mask.sum()}')

    # Gyroscope covariance
    gyro_samples = gyr_imu[raw_cov_mask]

    if len(gyro_samples) >= 2:
        gyro_mean = np.mean(gyro_samples, axis=0)
        gyro_cov = np.cov(gyro_samples.T, ddof=1)

        print()
        print('Gyro stationary mean / bias estimate [rad/s]:')
        print(gyro_mean)

        print_covariance(
            'Angular velocity covariance',
            gyro_cov,
            'rad^2/s^2'
        )
    else:
        print('Not enough gyro samples for covariance.')

    # Accelerometer covariance
    accel_samples = acc_imu[raw_cov_mask]

    if len(accel_samples) >= 2:
        accel_mean = np.mean(accel_samples, axis=0)
        accel_cov = np.cov(accel_samples.T, ddof=1)

        print()
        print('Accelerometer stationary mean [m/s^2]:')
        print(accel_mean)

        print_covariance(
            'Linear acceleration covariance',
            accel_cov,
            '(m/s^2)^2'
        )
    else:
        print('Not enough accelerometer samples for covariance.')

    # Madgwick orientation covariance
    mad_cov_mask = (tm >= cov_start) & (tm <= cov_end)
    orientation_samples = qm_imu[mad_cov_mask]

    print()
    print(f'Madgwick stationary samples: {len(orientation_samples)}')

    if len(orientation_samples) >= 10:
        q_mean = quaternion_mean(orientation_samples)

        # Small orientation errors around the mean:
        #   q_sample = q_mean * q_error
        #   q_error  = inverse(q_mean) * q_sample
        # converted to a rotation vector [rx, ry, rz] in radians.
        orientation_errors = np.array([
            quaternion_to_rotvec(
                q_mul(q_inv(q_mean), q)
            )
            for q in orientation_samples
        ])

        orientation_cov = np.cov(orientation_errors.T, ddof=1)
        orientation_std_rad = np.sqrt(np.diag(orientation_cov))
        orientation_std_deg = np.degrees(orientation_std_rad)

        print('Mean Madgwick quaternion [w,x,y,z]:')
        print(q_mean)
        print('Orientation standard deviation [rad]:')
        print(orientation_std_rad)
        print('Orientation standard deviation [deg]:')
        print(orientation_std_deg)

        print_covariance(
            'Orientation covariance',
            orientation_cov,
            'rad^2'
        )
    else:
        print('Not enough Madgwick samples for orientation covariance.')


# Ready-to-copy sensor_msgs/Imu covariance arrays
print()
print('================================================')
print('sensor_msgs/Imu covariance values')
print('================================================')

for label, cov in (
    ('orientation_covariance', orientation_cov),
    ('angular_velocity_covariance', gyro_cov),
    ('linear_acceleration_covariance', accel_cov)
):
    print()
    if cov is not None:
        print(f'{label}:')
        print(format_ros_array(cov))
    else:
        print(f'{label}: unavailable')

# Important yaw warning
if orientation_cov is not None:
    print()
    print('NOTE: Madgwick is running without magnetometer.')
    print(
        'The measured yaw covariance above represents only '
        'short-term stationary noise.'
    )
    print(
        'It does NOT represent long-term yaw drift or '
        'absolute heading accuracy.'
    )


# Plots
gimbal = np.abs(rpy_e[:, 1]) > 80.0

fig, ax = plt.subplots(
    6,
    1,
    sharex=True,
    figsize=(13, 15)
)

# Plot 1: Madgwick vs accelerometer
ax[0].plot(tr, acc_roll, '.', ms=1, alpha=0.3, label='accel roll')
ax[0].plot(tm, rpy_m_base[:, 0], label='Madgwick roll')
ax[0].plot(tr, acc_pitch, '.', ms=1, alpha=0.3, label='accel pitch')
ax[0].plot(tm, rpy_m_base[:, 1], label='Madgwick pitch')

ax[0].set_ylabel('deg\nabsolute')
ax[0].set_title('Madgwick vs accelerometer — base_link frame')
ax[0].legend(loc='upper right', fontsize=8)
ax[0].grid(alpha=0.2)

# Plot 2: Madgwick vs EKF orientation
ax[1].plot(te, rpy_m_at_ekf[:, 0], label='Madgwick roll')
ax[1].plot(te, rpy_m_at_ekf[:, 1], label='Madgwick pitch')
ax[1].plot(te, rpy_m_at_ekf[:, 2], label='Madgwick yaw')
ax[1].plot(te, rpy_e[:, 0], '--', label='EKF roll')
ax[1].plot(te, rpy_e[:, 1], '--', label='EKF pitch')
ax[1].plot(te, rpy_e[:, 2], '--', label='EKF yaw')

ax[1].fill_between(
    te,
    -180,
    180,
    where=gimbal,
    alpha=0.15,
    label='|EKF pitch| > 80°'
)

ax[1].set_ylabel('deg\nabsolute')
ax[1].set_title('Absolute orientation: Madgwick vs EKF')
ax[1].legend(loc='upper right', fontsize=8, ncol=2)
ax[1].grid(alpha=0.2)

# Plot 3: Angular rate
for i, axis_name in enumerate('xyz'):
    ax[2].plot(tr, gyr_base[:, i], alpha=0.5, label=f'gyro {axis_name}')
    ax[2].plot(te, rate_e[:, i], '--', alpha=0.7, label=f'EKF {axis_name}')

ax[2].set_ylabel('rad/s')
ax[2].set_title('Angular rate: EKF vs gyro — base_link')
ax[2].legend(loc='upper right', fontsize=8, ncol=2)
ax[2].grid(alpha=0.2)

# Plot 4: Quaternion orientation error
ax[3].plot(te, ang_abs, label='absolute quaternion error')
ax[3].axhline(2.0, linestyle='--', alpha=0.5, label='2°')
ax[3].axhline(5.0, linestyle='--', alpha=0.5, label='5°')

plot_max = max(10.0, float(np.max(ang_abs)) * 1.05)

ax[3].fill_between(
    te,
    0,
    plot_max,
    where=gimbal,
    alpha=0.15,
    label='Euler |pitch| > 80°'
)

ax[3].axvline(te[i_max], linestyle=':', alpha=0.6, label='_max_error')
ax[3].set_ylim(0, plot_max)
ax[3].set_ylabel('orientation\nerror (deg)')
ax[3].set_title('Absolute quaternion error — SLERP time aligned')
ax[3].legend(loc='upper right', fontsize=8)
ax[3].grid(alpha=0.2)

# Plot 5: Euler component residuals
for i, axis_name in enumerate(('roll', 'pitch', 'yaw')):
    ax[4].plot(
        te,
        orientation_rpy_residual[:, i],
        label=f'EKF - Madgwick {axis_name}'
    )

ax[4].axhline(0.0, linestyle='--', alpha=0.4)
ax[4].axvline(te[i_max], linestyle=':', alpha=0.6)

residual_plot_max = max(
    5.0,
    float(np.nanpercentile(np.abs(orientation_rpy_residual), 99.5)) * 1.25
)

ax[4].fill_between(
    te,
    -residual_plot_max,
    residual_plot_max,
    where=gimbal_eval,
    alpha=0.15,
    label='Euler |pitch| > 80 deg'
)

ax[4].set_ylim(-residual_plot_max, residual_plot_max)
ax[4].set_ylabel('deg')
ax[4].set_title('Orientation component residuals: EKF - Madgwick')
ax[4].legend(loc='upper right', fontsize=8, ncol=2)
ax[4].grid(alpha=0.2)

# Plot 6: Angular-rate residuals
if FUSE_ANGULAR_RATE:
    for i, axis_name in enumerate('xyz'):
        ax[5].plot(
            te,
            rate_residual[:, i],
            label=f'EKF - gyro {axis_name}'
        )

    ax[5].axhline(0.0, linestyle='--', alpha=0.4)
    ax[5].axvline(te[i_max], linestyle=':', alpha=0.6)
    ax[5].legend(loc='upper right', fontsize=8, ncol=3)
else:
    ax[5].text(
        0.5,
        0.5,
        'Angular-rate fusion disabled',
        transform=ax[5].transAxes,
        ha='center',
        va='center'
    )

ax[5].set_ylabel('rad/s')
ax[5].set_xlabel('time (s)')
ax[5].set_title('Angular-rate residuals: EKF - transformed raw gyro')
ax[5].grid(alpha=0.2)

plt.tight_layout()
plt.show()
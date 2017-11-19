# Reference: https://stackoverflow.com/questions/4870393/rotating-coordinate-system-via-a-quaternion

from math import *


def euler_to_quat(euler):
    """
    # Converts an Euler angle vector into a Quaternion vector
    # Reference: http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm
    :param euler: [bank, heading, attitude] Euler angles
    :rtype: list: [w,x,y,z] Quaternion
    """
    cx = cos(euler[0] / 2)
    cy = cos(euler[1] / 2)
    cz = cos(euler[2] / 2)
    sx = sin(euler[0] / 2)
    sy = sin(euler[1] / 2)
    sz = sin(euler[2] / 2)

    w = cy * cz * cx - sy * sz * sx
    x = sy * sz * cx + cy * cz * sx
    y = sy * cz * cx + cy * sz * sx
    z = cy * sz * cx - sy * cz * sx

    quaternion = [w, x, y, z]
    return quaternion


# @todo double check this function was c&p'd correctly
def quat_to_dcm(quat):
    """
    # Reference: http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
    :param quat: Quaternion vector
    :return: Discrete Cosine (rotation) Matrix
    """
    q = quat
    # @todo fuck the auto-indenting as the matrix is now unreadable DX
    dcm = [
        [1 - 2 * q[3] * q[3] - 2 * q[2] * q[2], -2 * q[3] * q[0] + 2 * q[2] * q[1], 2 * q[2] * q[0] + 2 * q[3] * q[1]],
        [2 * q[1] * q[2] + 2 * q[0] * q[3], 1 - 2 * q[3] * q[3] - 2 * q[1] * q[1], 2 * q[3] * q[2] - 2 * q[1] * q[0]],
        [2 * q[1] * q[3] - 2 * q[0] * q[2], 2 * q[2] * q[3] + 2 * q[0] * q[1], 1 - 2 * q[2] * q[2] - 2 * q[1] * q[1]]
    ]
    return dcm


def normalize(v, tolerance=0.00001):
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = sqrt(mag2)
        v = tuple(n / mag for n in v)
    return v


def q_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return w, x, y, z


def q_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def qv_mult(q1, v1):
    q2 = (0.0,) + v1
    return q_mult(q_mult(q1, q2), q_conjugate(q1))[1:]


def axisangle_to_q(v, theta):
    v = normalize(v)
    x, y, z = v
    theta /= 2
    w = cos(theta)
    x = x * sin(theta)
    y = y * sin(theta)
    z = z * sin(theta)
    return w, x, y, z


def q_to_axisangle(q):
    w, v = q[0], q[1:]
    theta = acos(w) * 2.0
    return normalize(v), theta


















































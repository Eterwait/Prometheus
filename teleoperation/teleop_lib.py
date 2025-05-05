import struct
from datetime import datetime

import numpy as np
from numpy.linalg import norm
from scipy.spatial.transform import Rotation as R
from pyrr import quaternion as qv
import copy

# UR3 control settings
velocity = 0.5
acceleration = 0.5
dt = 1.0/500  # 2ms
lookahead_time = 0.2
gain = 100

# Joint limits
# Elbow = 0.5 2.0
# Wrist1 = -0.7 0.8 
# Wrist2 = 0.8 2.5
# Wrist3 = 504.4 507.4


def isInJointLimits(q):
    today = datetime.now()
    if q[2]<0.2 or q[2]>2.5:
        
        print(today.strftime('%H:%M:%S') , " - Elbow out of limit!")
        return False
    if q[3]<-1.1 or q[3]>0.9:
        print(today.strftime('%H:%M:%S') ," - Wrist1 out of limit!")
        return False
    if q[4]< 0.0 or q[4]> 2.7:
        print(today.strftime('%H:%M:%S') ," - Wrist2 out of limit!")
        return False
    if q[5]< 506.5 or q[5]> 511.5:
        print(today.strftime('%H:%M:%S') , " - Wrist3 out of limit!")
        return False
    return True

   
def quaternion_multiply(quaternion1, quaternion0, reverse = False):
    if reverse:
        x0, y0, z0, w0 = quaternion0
        x1, y1, z1, w1 = quaternion1
    else:
        w0, x0, y0, z0 = quaternion0
        w1, x1, y1, z1 = quaternion1

    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

def hand_to_ur(hand, base_hand, base_robot, sens = 1.0):
    ur = np.zeros(6)
    vec = np.array([0., 0., 0.0819])

    # position
    ur[0] = base_robot[0] - (hand['x'][0] - base_hand['x'][0])/sens
    ur[1] = base_robot[1] + (hand['z'][0] - base_hand['z'][0])/sens
    ur[2] = base_robot[2] + (hand['y'][0] - base_hand['y'][0])/sens
    
    # orientation
    qh0 = [base_hand[axis][0] for axis in ['r_x', 'r_y', 'r_z', 'r_w']]
    q = [hand[axis][0] for axis in ['r_x', 'r_y', 'r_z', 'r_w']]
    q_inv = qv.inverse(q)
    qh = quaternion_multiply(q_inv, qh0, reverse = True)

    qh_init = copy.deepcopy(qh)
    qh[2] = -qh_init[3]
    qh[1] = qh_init[2]
    qh[3] = qh_init[1]
    qh[0] = -qh_init[0]

    rot = R.from_rotvec(base_robot[3:6])
    qr0 = rot.as_quat()
    qr = quaternion_multiply(qh, qr0)

    ur[3:6] = R.from_quat(qr).as_rotvec()

    # positional offset
    rot2 = R.from_quat(qr)
    base_vec = rot.apply(vec)
    new_vec = rot2.apply(vec)
    dif = new_vec - base_vec
    ur[0:3] += dif
    
    return ur

# gripper control
def crc32mpeg2(data):
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte << 24
        for _ in range(8):
            if crc & 0x80000000:
                crc = (crc << 1) ^ 0x04C11DB7
            else:
                crc <<= 1
            crc &= 0xFFFFFFFF
    return crc

def read_pose(port):
    flag_message_error = False
    package = bytearray(b"i")
    port.flushInput()
    port.write(package)
    # Получаем данные из порта
    data = port.read(8)
    # Проверяем, что данные получены и их размер равен 8 байт
    if data and len(data) == 8:
        teleop_position = np.frombuffer(data[:4], dtype=np.uint32)[0]
        CRC_STM = np.frombuffer(data[4:], dtype=np.uint32)[0]
        # high-endian это очень важно (!)
        bytes_for_crc = bytearray(struct.pack(">I", teleop_position))
        crc32_python = crc32mpeg2(bytes_for_crc)
        if crc32_python != CRC_STM:
            flag_message_error = True
        else:
            teleop_position = round(teleop_position / 3)
            if teleop_position >= 255:
                teleop_position=254
            if teleop_position == 33:
                teleop_position=0
            return teleop_position
    else:
        flag_message_error = True
        teleop_position=0
        return teleop_position


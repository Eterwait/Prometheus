from time import sleep
from time import time
from serial import Serial
from serial.tools.list_ports import comports
from datetime import datetime
# print(datetime.now())

import pose_openvr_wrapper
import rtde_control
import rtde_receive
import robotiq_gripper

import sys
import teleop_lib as lib
import keyboard 
import socket
import struct


def run(new_trajectory = True):

    if new_trajectory:
        command = 2
        sock.sendall(struct.pack('i' , command))
        
    base_hand = pyopenvr_wrapper.sample(hand_tracker, samples_count=1)
    base_robot = rtde_r.getActualTCPPose()

    while(1):
        # save the observation
        command = 1
        sock.sendall(struct.pack('i' , command))

        hand = pyopenvr_wrapper.sample(hand_tracker, samples_count=1)
        ur_pose = lib.hand_to_ur(hand, base_hand, base_robot)
        if rtde_c.isPoseWithinSafetyLimits(ur_pose):
            q = rtde_c.getInverseKinematics(ur_pose)
            # if lib.isInJointLimits(q):
            rtde_c.servoJ(q, lib.velocity, lib.acceleration, lib.dt, lib.lookahead_time, lib.gain)

        pos = lib.read_pose()
        if (pos < gripper._max_position - 30) and (pos > gripper._min_position + 30):
            gripper.move(pos, 200, 10)
            sleep(0.01)

        try: 
            if keyboard.is_pressed('s'):
                print('---------------------------------')
                print('Program stopped. Possible options:')
                print('e - exit program')
                print('b - move to base pose')
                print('r - run from current pose')
                break  # finishing the loop 
        except: 
            break

def base():
    # Go to base pose
    rtde_c.servoJ([1.7286394834518433,
    -1.5598433653460901,
    1.5497121810913086,
    0.05845451354980469,
    1.6997259855270386,
    508.9877680103683], lib.velocity, lib.acceleration, lib.dt, lib.lookahead_time, lib.gain)


port_name = "COM3"
unity_host, unity_port = "127.0.0.1", 25001

# change this variables befor usage!
hand_tracker = 'tracker_0'
robot_ip = "192.168.1.110"


# open COM port
# for port in comports():
#     print('COM port: ', port)

# port = Serial(port_name, 230400, timeout=1)
# if not port.isOpen():
#     print('Device is not connected to COM port!')
#     sys.exit(1)


pyopenvr_wrapper = pose_openvr_wrapper.OpenvrWrapper('config.json')
print('Connected devices: ', pyopenvr_wrapper.devices)
if hand_tracker not in pyopenvr_wrapper.devices.keys():
        print(f'Hand tracker not connected to PC!')
        sys.exit(1)

try:
    rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
    rtde_c = rtde_control.RTDEControlInterface(robot_ip)
    gripper = robotiq_gripper.RobotiqGripper()
except:
    print('Could not connect to robot!')
    sys.exit(1) # ToDo !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

gripper.connect(robot_ip, 63352)
if not gripper.is_active():
    try:
        gripper.activate()
    except TimeoutError:
        pass


sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((unity_host, unity_port))

# go to the base pose
base()
timeout = False

sleep(1)
print('Starting teleoperation...')

run()
   

while(1):
    try: 
        if keyboard.is_pressed('e'):
            break  # finishing the loop 
    except: 
        break

    try: 
        if keyboard.is_pressed('b'):
            print('---------------------------------')
            print('Move to base pose...')
            print('Program stopped. Possible options:')
            print('e - exit program')
            print('b - move to base pose')
            print('r - run new trajectory from current pose')
            base()
            timeout = True
    except: 
        break
        
    try: 
        if keyboard.is_pressed('r'):
            print('---------------------------------')
            print('Program running...')
            run()
    except: 
        break

    if timeout:
        timeout = False
        sleep(0.1)
        


print('Exit program...')
sock.close()
    



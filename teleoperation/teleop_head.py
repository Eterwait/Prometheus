import socket
import json
import numpy as np
import pose_openvr_wrapper
import time
import sys
import keyboard

head_tracker = 'hmd_1'

def send_json_message(
    socket,
    json_message,
) -> None:
    """Send json packet to server"""
    server_ip = "192.168.1.100"
    server_port = 8000
    message = (json.dumps(json_message) + '\n').encode()
    socket.sendto(message, (server_ip, server_port))
    print(f'{len(message)} bytes sent')

def run(angle_1 = 0, angle_2 = 0):
    start = time.time()
    count = 0
    base_pitch = pyopenvr_wrapper.sample(head_tracker, samples_count=1)['pitch'][0]
    base_roll = pyopenvr_wrapper.sample(head_tracker, samples_count=1)['roll'][0]
    while True:
        data1 = angle_1 + (pyopenvr_wrapper.sample(head_tracker, samples_count=1)['pitch'][0] - base_pitch)
        data2 = angle_2 + (base_roll - pyopenvr_wrapper.sample(head_tracker, samples_count=1)['roll'][0])
        if ((data1 > -90) and (data1 < 90)):
            json_message = {'id': 1, 'angle': data1}            
            send_json_message(client_socket, json_message)
        if ((data2 > -40) and (data2 < 70)):
            json_message = {'id': 2, 'angle': data2}
            send_json_message(client_socket, json_message)
            
        count+=1
        try:  # used try so that if user pressed other than the given key error will not be shown 
            if keyboard.is_pressed('p'):  # if key 'q' is pressed
                print(count / (time.time() - start))
                break  # finishing the loop 
        except: 
            break
    return data1, data2 
        # time.sleep(0.02)

def base():
    json_message = {'id': 1, 'angle': 0}
    send_json_message(client_socket, json_message)
    json_message = {'id': 2, 'angle': 0}
    send_json_message(client_socket, json_message)


pyopenvr_wrapper = pose_openvr_wrapper.OpenvrWrapper('config.json')
if head_tracker not in pyopenvr_wrapper.devices.keys():
    print(f'Head tracker not connected to PC!')
    sys.exit()

client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

angle_1, angle_2 = run()

while(1):
    print('Program stopped. Possible options:')
    print('e - exit program')
    print('b - move to base pose')
    print('r - run from current pose')

    command = input('Enter your choise: ')
    if command == 'e':
        print('Exit program...')
        break
    if command == 'base':
        base()
        angle_1, angle_2 = 0, 0
    if command == 'run':
        angle_1, angle_2 = run(angle_1, angle_2)

client_socket.close()



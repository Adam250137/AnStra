import serial
import requests
import time
import json

url = 'http://naukawszkole.support:9000'
app_token = 'fhgo454rfm!@H'

def send_command(communication, string):
    communication.write(bytes(string, 'ascii'))

def diode(communication):
    response = requests.get(url + '/execute/led_toggle', params={'token' : app_token})
    led_status = response.json()['led_status']
    if led_status == 1: send_command(communication, 'TOGGLE')

def execute(communication):
    response = requests.get(url + '/execute', params={'token' : app_token})
    if response.status_code != 200: 
        print('Connection to server failed!')
        return

    content = response.json()
    if content['error'] == 'bad token!':
        print('Bad token!')
        return

    if content['error'] == 'no changes': return

    match content['changed_parameter']:
        case 'angles':
            s1 = content['servo']['servo_1']
            s2 = content['servo']['servo_2']
            send_command(communication, f'SET_ANGLES {s1} {s2}')
            print(f'SET_ANGLES {s1} {s2}')
            return
        case 'target':
            latitude = content['target']['latitude']
            longitude = content['target']['longitude']
            height = content['target']['height']
            send_command(communication, f'SET_TARGET {latitude} {longitude} {height}')
            print(f'SET_TARGET {latitude} {longitude} {height}')
            return

def search(data, parameter, pos):
    start = data.find(parameter, pos)
    end = data.find(';', start)
    return data[start + len(parameter) + 1 : end]

def update_angles_target(communication):
    data = str(communication.readline())
    print(data)

    pos = data.find('SEND_ANGLES')
    if  pos != -1:
        s1 = search(data, 's1', pos)
        s2 = search(data, 's2', pos)
        params={'token' : app_token, 'user' : 'anstra', 'angle_1' : s1, 'angle_2' : s2}
        requests.post(url + '/angles', params=params)

    pos = data.find('SEND_TARGET')
    if pos != -1:
        latitude = search(data, 'latitude', pos)
        longitude = search(data, 'longitude', pos)
        height = search(data, 'height', pos)
        params={'token' : app_token, 'user' : 'anstra', 'latitude' : latitude, 'longitude' : longitude, 'height' : height}
        requests.post(url + '/target', params=params)

def update_location(communication):
    send_command(communication, 'GET_POSITION')
    data = communication.readline()
    pos = data.find('SEND_POSITION')
    if pos != -1:
        latitude = search(data, 'latitude', pos)
        longitude = search(data, 'longitude', pos)
        height = search(data, 'height', pos)
        params={'token' : app_token, 'latitude' : latitude, 'longitude' : longitude, 'height' : height}
        requests.post(url + '/station', params=params)

port = input('Podaj port do komunikacji: ')
communication = serial.Serial(port, 115200)

while(True):
   
    diode(communication)
    update_angles_target(communication)
    execute(communication)
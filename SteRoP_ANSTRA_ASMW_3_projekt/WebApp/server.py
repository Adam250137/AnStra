import uvicorn
from fastapi import FastAPI

servo1_angle = ''
servo2_angle = ''
station_latitude = ''
station_longitude = ''
station_height = ''
target_latitude = ''
target_longitude = ''
target_height = ''

changed = False
changed_parameter = None
app_token = 'fhgo454rfm!@H'

led_status = 0

anstra_app = FastAPI()

if __name__ == "__main__":
    uvicorn.run("server:anstra_app", host="0.0.0.0", port=9000)

@anstra_app.get('/anstra_overview')
async def get_parameters():
    global servo1_angle, servo2_angle, target_latitude, target_longitude, target_height, station_latitude, station_longitude, station_height
    response = {
        'servo' : {'servo_1' : servo1_angle, 'servo_2' : servo2_angle}, 
        'station' : {'station_latitude' : station_latitude, 'station_longitude' : station_longitude, 'station_height' : station_height}, 
        'target' : {'target_latitude' : target_latitude, 'target_longitude' : target_longitude, 'target_height' : target_height} }
    return response

@anstra_app.post('/angles')
async def set_angles(token, user, angle_1, angle_2):
    if token != app_token: return {'error' : 'bad token!'}
    global servo1_angle, servo2_angle, changed, changed_parameter
    if changed == True: return {'error' : 'operation in progress'}
    servo1_angle = angle_1
    servo2_angle = angle_2
    
    if user == 'stranger':
        changed = True
        changed_parameter = 'angles'

    return {'error' : 'no error', 'servo' : {'servo_1' : servo1_angle, 'servo_2' : servo2_angle}}

@anstra_app.post('/target')
async def set_target_position(token, user, latitude, longitude, height):
    if token != app_token: return {'error' : 'bad token!'}
    global target_latitude, target_longitude, target_height, changed, changed_parameter
    if changed == True: return {'error' : 'operation in progress'}
    target_latitude = latitude
    target_longitude = longitude
    target_height = height
    
    if user == 'stranger':
        changed = True
        changed_parameter = 'target'

    return {'error': 'no error', 'target' : {'latitude' : target_latitude, 'longitude' : target_longitude, 'height' : target_height}}

@anstra_app.post('/station')
async def set_station_position(token, latitude, longitude, height):
    if token != app_token: return {'error' : 'bad token!'}
    global station_latitude, station_longitude, station_height
    station_latitude = latitude
    station_longitude = longitude
    station_height = height
    return {'error': 'no error', 'station' : {'latitude' : station_latitude, 'longitude' : station_longitude, 'height' : station_height}}

@anstra_app.get('/execute')
async def execute_set_values(token):
    if token != app_token: return {'error' : 'bad token!'}
    global servo1_angle, servo2_angle, target_latitude, target_longitude, target_height, changed, changed_parameter
    if changed == False: return {'error' : 'no changes'}
    match changed_parameter:
        case 'angles' :
            response = {'error' : 'no error', 'changed_parameter' : 'angles', 'servo' : {'servo_1' : servo1_angle, 'servo_2' : servo2_angle}}
        case 'target' :
            response = {'error' : 'no error', 'changed_parameter' : 'target', 'target' : {'latitude' : target_latitude, 'longitude' : target_longitude, 'height' : target_height}}
    changed = False
    changed_parameter = None
    return response

# testowanie z diodą LED

@anstra_app.post('/led_toggle')
async def led_toggle(token):
    if token != app_token: return {'led_status' : 'Bad token!'}
    global led_status
    led_status = 1
    return {'led_status' : led_status}

@anstra_app.get('/execute/led_toggle')
async def execute_led_toggle(token):
    if token != app_token: return {'led_status' : 'Bad token!'}
    global led_status
    response = led_status
    led_status = 0
    return {'led_status' : response}
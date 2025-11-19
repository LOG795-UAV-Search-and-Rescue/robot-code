# import base_ctrl library
from base_ctrl import BaseController
import threading
import yaml, os
import subprocess
import sys

def set_default_sink(device_name):
    try:
        command = ['pacmd', 'set-default-sink', device_name]
        subprocess.run(command, check=True)
        print(f"Default sink set to '{device_name}' successfully.")
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")

device_name = "alsa_output.usb-Solid_State_System_Co._Ltd._USB_PnP_Audio_Device_000000000000-00.analog-stereo"
set_default_sink(device_name)

# JETSON ORIN NX BASE CONTROLLER
base = BaseController('/dev/ttyTHS1', 115200)

threading.Thread(target=lambda: base.breath_light(20), daemon=True).start()

# config file.
curpath = os.path.realpath(__file__)
thisPath = os.path.dirname(curpath)
CONFIG_PATH = thisPath + '/config.yaml'
PICTURES_PATH = thisPath + '/templates/pictures'
with open(CONFIG_PATH, 'r') as yaml_file:
    f = yaml.safe_load(yaml_file)

base.base_oled(0, f["base_config"]["robot_name"])
base.base_oled(1, f"sbc_version: {f['base_config']['sbc_version']}")
base.base_oled(2, f"{f['base_config']['main_type']}{f['base_config']['module_type']}")
base.base_oled(3, "Starting...")


# Import necessary modules
from flask import Flask, render_template, Response, request, jsonify, redirect, url_for, send_from_directory, send_file
from flask_socketio import SocketIO, emit
from werkzeug.utils import secure_filename
from aiortc import RTCPeerConnection, RTCSessionDescription
import socketserver
import json
import uuid
import asyncio
import time
import logging
import logging
import cv_ctrl
import audio_ctrl
import os_info

# Get system info
UPLOAD_FOLDER = thisPath + '/sounds/others'
si = os_info.SystemInfo()

# Create a Flask app instance
app = Flask(__name__)
# log = logging.getLogger('werkzeug')
# log.disabled = True
socketio = SocketIO(app)

# Set to keep track of RTCPeerConnection instances
active_pcs = {}

# Maximum number of active connections allowed
MAX_CONNECTIONS = 1

# Set to keep track of RTCPeerConnection instances
pcs = set()

# Camera funcs
cvf = cv_ctrl.OpencvFuncs(thisPath, base)

# Map funcs
from map_ctrl import MapController
map_ctrl = MapController(base)

cmd_actions = {
    f['code']['zoom_x1']: lambda: cvf.scale_ctrl(1),
    f['code']['zoom_x2']: lambda: cvf.scale_ctrl(2),
    f['code']['zoom_x4']: lambda: cvf.scale_ctrl(4),

    f['code']['pic_cap']: cvf.picture_capture,
    f['code']['vid_sta']: lambda: cvf.video_record(True),
    f['code']['vid_end']: lambda: cvf.video_record(False),

    f['code']['cv_none']: lambda: cvf.set_cv_mode(f['code']['cv_none']),
    f['code']['cv_moti']: lambda: cvf.set_cv_mode(f['code']['cv_moti']),
    f['code']['cv_face']: lambda: cvf.set_cv_mode(f['code']['cv_face']),
    f['code']['cv_objs']: lambda: cvf.set_cv_mode(f['code']['cv_objs']),
    f['code']['cv_clor']: lambda: cvf.set_cv_mode(f['code']['cv_clor']),
    f['code']['mp_hand']: lambda: cvf.set_cv_mode(f['code']['mp_hand']),
    f['code']['cv_auto']: lambda: cvf.set_cv_mode(f['code']['cv_auto']),
    f['code']['mp_face']: lambda: cvf.set_cv_mode(f['code']['mp_face']),
    f['code']['mp_pose']: lambda: cvf.set_cv_mode(f['code']['mp_pose']),

    f['code']['re_none']: lambda: cvf.set_detection_reaction(f['code']['re_none']),
    f['code']['re_capt']: lambda: cvf.set_detection_reaction(f['code']['re_capt']),
    f['code']['re_reco']: lambda: cvf.set_detection_reaction(f['code']['re_reco']),

    f['code']['mc_lock']: lambda: cvf.set_movtion_lock(True),
    f['code']['mc_unlo']: lambda: cvf.set_movtion_lock(False),

    f['code']['led_off']: lambda: cvf.head_light_ctrl(0),
    f['code']['led_aut']: lambda: cvf.head_light_ctrl(1),
    f['code']['led_ton']: lambda: cvf.head_light_ctrl(2),

    f['code']['release']: lambda: base.bus_servo_torque_lock(255, 0),
    f['code']['s_panid']: lambda: base.bus_servo_id_set(255, 2),
    f['code']['s_tilid']: lambda: base.bus_servo_id_set(255, 1),
    f['code']['set_mid']: lambda: base.bus_servo_mid_set(255),

    f['code']['base_of']: lambda: base.lights_ctrl(0, base.head_light_status),
    f['code']['base_on']: lambda: base.lights_ctrl(255, base.head_light_status),
    f['code']['head_ct']: lambda: cvf.head_light_ctrl(3),
    f['code']['base_ct']: base.base_lights_ctrl
}

cmd_feedback_actions = [f['code']['cv_none'], f['code']['cv_moti'],
                        f['code']['cv_face'], f['code']['cv_objs'],
                        f['code']['cv_clor'], f['code']['mp_hand'],
                        f['code']['cv_auto'], f['code']['mp_face'],
                        f['code']['mp_pose'], f['code']['re_none'],
                        f['code']['re_capt'], f['code']['re_reco'],
                        f['code']['mc_lock'], f['code']['mc_unlo'],
                        f['code']['led_off'], f['code']['led_aut'],
                        f['code']['led_ton'], f['code']['base_of'],
                        f['code']['base_on'], f['code']['head_ct'],
                        f['code']['base_ct']
                        ]

# cv info process
def process_cv_info(cmd):
    if cmd[f['fb']['detect_type']] != f['code']['cv_none']:
        print(cmd[f['fb']['detect_type']])
        pass

# Function to generate video frames from the camera
def generate_frames():
    while True:
        frame = cvf.frame_process()
        try:
            yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n') 
        except Exception as e:
            print("An [generate_frames] error occurred:", e)

# Function to generate map frames
def generate_map_frames():
    while True:
        # frame = map_ctrl.create_graph_as_bytes()
        frame = map_ctrl.lidar_frame_generate()
        try:
            yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n') 
        except Exception as e:
            print("An [generate_map_frames] error occurred:", e)

# Function to generate position frames
def generate_pos_frames():
    while True:
        frame = map_ctrl.create_pose_graph()
        try:
            yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n') 
        except Exception as e:
            print("An [generate_pos_frames] error occurred:", e)

# Route to render the HTML template
@app.route('/')
def index():
    audio_ctrl.play_random_audio("connected", False)
    return render_template('index.html')

@app.route('/config')
def get_config():
    with open(CONFIG_PATH, 'r') as file:
        yaml_content = file.read()
    return yaml_content

# get pictures and videos.
@app.route('/<path:filename>')
def serve_static(filename):
    return send_from_directory('templates', filename)

@app.route('/get_photo_names')
def get_photo_names():
    photo_files = sorted(os.listdir(PICTURES_PATH), key=lambda x: os.path.getmtime(os.path.join(PICTURES_PATH, x)), reverse=True)
    return jsonify(photo_files)

@app.route('/delete_photo', methods=['POST'])
def delete_photo():
    filename = request.form.get('filename')
    try:
        os.remove(os.path.join(PICTURES_PATH, filename))
        return jsonify(success=True)
    except Exception as e:
        print(e)
        return jsonify(success=False)

@app.route('/videos/<path:filename>')
def videos(filename):
    return send_from_directory(thisPath + '/templates/videos', filename)

@app.route('/get_video_names')
def get_video_names():
    video_files = sorted(
        [filename for filename in os.listdir(thisPath + '/templates/videos/') if filename.endswith('.mp4')],
        key=lambda filename: os.path.getctime(os.path.join(thisPath + '/templates/videos/', filename)),
        reverse=True
    )
    return jsonify(video_files)

@app.route('/delete_video', methods=['POST'])
def delete_video():
    filename = request.form.get('filename')
    try:
        os.remove(os.path.join(thisPath + '/templates/videos', filename))
        return jsonify(success=True)
    except Exception as e:
        print(e)
        return jsonify(success=False)

FRAME_MIME_TYPE = 'multipart/x-mixed-replace; boundary=frame'

# Route to stream video frames
@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype=FRAME_MIME_TYPE)

# Route to stream map frames
@app.route('/map_feed')
def map_feed():
    return Response(generate_map_frames(), mimetype=FRAME_MIME_TYPE)

# Route to stream position frames
@app.route('/pos_feed')
def pos_feed():
    return Response(generate_pos_frames(), mimetype=FRAME_MIME_TYPE)

# Go to position
@app.route('/go_to', methods=['POST'])
def go_to():
    x = float(request.json.get('x', 0))
    y = float(request.json.get('y', 0))
    map_ctrl.go_to(x, y)
    return jsonify({'success': True, 'message': f'Going to position ({x}, {y})'})

@app.route('/reset_pos', methods=['POST'])
def reset_pos():
    map_ctrl.reset_position()
    return jsonify({'success': True, 'message': 'Position reset to (0, 0)'})

@app.route('/stop', methods=['POST'])
def stop():
    map_ctrl.stop()
    return jsonify({'success': True, 'message': 'Stopping movement'})


# Video WebRTC

# Function to manage connections
def manage_connections(pc_id, pc):
    if len(active_pcs) >= MAX_CONNECTIONS:
        # If maximum connections reached, terminate the oldest connection
        oldest_pc_id = next(iter(active_pcs))
        old_pc = active_pcs.pop(oldest_pc_id)
        old_pc.close()

    # Add new connection to active connections
    active_pcs[pc_id] = pc

# Asynchronous function to handle offer exchange
async def offer_async():
    params = await request.json
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    # Create an RTCPeerConnection instance
    pc = RTCPeerConnection()

    # Generate a unique ID for the RTCPeerConnection
    pc_id = "PeerConnection(%s)" % uuid.uuid4()
    pc_id = pc_id[:8]

    # Manage connections
    manage_connections(pc_id, pc)

    # Create and set the local description
    await pc.createOffer(offer)
    await pc.setLocalDescription(offer)

    # Prepare the response data with local SDP and type
    response_data = {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}

    return jsonify(response_data)

# Wrapper function for running the asynchronous offer function
def offer():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    future = asyncio.run_coroutine_threadsafe(offer_async(), loop)
    return future.result()

# set product version
def set_version(input_main, input_module):
    base.base_json_ctrl({"T":900,"main":input_main,"module":input_module})
    if input_main == 1:
        cvf.info_update("RaspRover", (0,255,255), 0.36)
    elif input_main == 2:
        cvf.info_update("UGV Rover", (0,255,255), 0.36)
    elif input_main == 3:
        cvf.info_update("UGV Beast", (0,255,255), 0.36)
    if input_module == 0:
        cvf.info_update("No Module", (0,255,255), 0.36)
    elif input_module == 1:
        cvf.info_update("ARM", (0,255,255), 0.36)
    elif input_module == 2:
        cvf.info_update("PT", (0,255,255), 0.36)

# Route to handle the offer request
@app.route('/offer', methods=['POST'])
def offer_route():
    return offer()

@app.route('/getAudioFiles', methods=['GET'])
def get_audio_files():
    files = [f for f in os.listdir(UPLOAD_FOLDER) if os.path.isfile(os.path.join(UPLOAD_FOLDER, f)) and (f.endswith('.mp3') or f.endswith('.wav'))]
    return jsonify(files)

@app.route('/uploadAudio', methods=['POST'])
def upload_audio():
    if 'file' not in request.files:
        return jsonify({'error': 'No file part'})
    file = request.files['file']
    if file.filename == '':
        return jsonify({'error': 'No selected file'})
    if file:
        filename = secure_filename(file.filename)
        file.save(os.path.join(UPLOAD_FOLDER, filename))
        return jsonify({'success': 'File uploaded successfully'})

@app.route('/playAudio', methods=['POST'])
def play_audio():
    audio_file = request.form['audio_file']
    print(thisPath + '/sounds/others/' + audio_file)
    audio_ctrl.play_audio_thread(thisPath + '/sounds/others/' + audio_file)
    return jsonify({'success': 'Audio is playing'})

@app.route('/stop_audio', methods=['POST'])
def audio_stop():
    audio_ctrl.stop()
    return jsonify({'success': 'Audio stop'})




# Web socket

# info update single
def update_data_websocket_single():
    # {'T':1001,'L':0,'R':0,'r':0,'p':0,'v': 11,'pan':0,'tilt':0}
    try:
        socket_data = {
            f['fb']['picture_size']:si.pictures_size,
            f['fb']['video_size']:  si.videos_size,
            f['fb']['cpu_load']:    si.cpu_load,
            f['fb']['cpu_temp']:    si.cpu_temp,
            f['fb']['ram_usage']:   si.ram,
            f['fb']['wifi_rssi']:   si.wifi_rssi,

            f['fb']['led_mode']:    cvf.cv_light_mode,
            f['fb']['detect_type']: cvf.cv_mode,
            f['fb']['detect_react']:cvf.detection_reaction_mode,
            f['fb']['pan_angle']:   cvf.pan_angle,
            f['fb']['tilt_angle']:  cvf.tilt_angle,
            f['fb']['base_voltage']:base.base_data['v'],
            f['fb']['video_fps']:   cvf.video_fps,
            f['fb']['cv_movtion_mode']: cvf.cv_movtion_lock,
            f['fb']['base_light']:  base.base_light_status
        }
        socketio.emit('update', socket_data, namespace='/ctrl')
    except Exception as e:
        print("An [app.update_data_websocket_single] error occurred:", e)

# info feedback
def update_data_loop():
    base.base_oled(2, "F/J:5000/8888")
    start_time = time.time()
    time.sleep(1)
    while 1:
        update_data_websocket_single()
        eth0 = si.eth0_ip
        wlan = si.wlan_ip
        if eth0:
            base.base_oled(0, f"E:{eth0}")
        else:
            base.base_oled(0, "E: No Ethernet")
        if wlan:
            base.base_oled(1, f"W:{wlan}")
        else:
            base.base_oled(1, f"W: NO {si.net_interface}")
        elapsed_time = time.time() - start_time
        hours = int(elapsed_time // 3600)
        minutes = int((elapsed_time % 3600) // 60)
        seconds = int(elapsed_time % 60)
        base.base_oled(3, f"{si.wifi_mode} {hours:02d}:{minutes:02d}:{seconds:02d} {si.wifi_rssi}dBm")
        time.sleep(5)

def base_data_loop():
    while True:
        data = base.feedback_data()
        # print("Base data:", data)

        map_ctrl.update(data)
        cvf.update_base_data(data)
        time.sleep(0.01)

def lidar_data_loop():
    while True:
        base.rl.lidar_data_recv()
        time.sleep(0.025)

@socketio.on('json', namespace='/json')
def handle_socket_json(json):
    try:
        base.base_json_ctrl(json)
        # print("Received JSON data via WebSocket:", json)
    except Exception as e:
        print("Error handling JSON data:", e)
        return

@socketio.on('message', namespace='/ctrl')
def handle_socket_cmd(message):
    try:
        json_data = json.loads(message)
        print("Received Message data via WebSocket:", json_data)
    except json.JSONDecodeError:
        print("Error decoding JSON.[app.handle_socket_cmd]")
        return
    cmd_a = float(json_data.get("A", 0))
    if cmd_a in cmd_actions:
        cmd_actions[cmd_a]()
    if cmd_a in cmd_feedback_actions:
        threading.Thread(target=update_data_websocket_single, daemon=True).start()



# commandline on boot
def cmd_on_boot():
    module_type = f['base_config']['module_type']
    speed_ratio_l = f['base_config']['speed_ratio_l']
    speed_ratio_r = f['base_config']['speed_ratio_r']
    cmd_list = [
        {"T":142,"cmd":20},                             # set feedback interval
        {"T":131,"cmd":1},                              # serial feedback flow on
        {"T":143,"cmd":0},                              # serial echo off
        {"T":4,"cmd":module_type},                      # select the module - 0:None, 1:RoArm-M2-S, 2:Gimbal
        {"T":300,"mode":0,"mac":"EF:EF:EF:EF:EF:EF"},   # the base won't be ctrl by esp-now broadcast cmd, but it can still recv broadcast megs.
        {"T":2,"P":20,"I":2000,"D":0,"L":255},          # base pid params
        {"T":133,"X":0,"Y":0,"SPD":0,"ACC":0},          # set inital position for gimbal
        {"T":-3},                                       # set default OLED text
        {"T":136,"cmd":3000},                           # set heartbeat interval
        {"T":138,"L":speed_ratio_l,"R":speed_ratio_r}   # set speed ratio
    ]

    print('{"T":4,"cmd":%d}' % module_type)
    for i in range(0, len(cmd_list)):
        base.base_json_ctrl(cmd_list[i])
        cvf.info_update(json.dumps(cmd_list[i]), (0,255,255), 0.36)
    set_version(f['base_config']['main_type'], f['base_config']['module_type'])


class UDPHandler(socketserver.BaseRequestHandler):
    mode_follow = True   # class variable
    cmd_triggered = False
    quality_min = 20.0  # minimum acceptable VIO quality
    drone_x = 0.0
    drone_y = 0.0
    last_good_x = 0.0
    last_good_y = 0.0

    def __init__(self, request, client_address, server):
        super().__init__(request, client_address, server)


    def handle(self):
        data = self.request[0].strip()
        msg = data.decode(errors="ignore").strip()
        # print(f"[UDP] Received message: {msg}")

        # === Handle Mode Commands ===
        if msg == "MODE_CONTINUOUS":
            UDPHandler.mode_follow = True
            UDPHandler.cmd_triggered = False
            # print("[MODE] Continuous follow mode activated.")
            return

        if msg == "MODE_COME_TO_ME":
            UDPHandler.mode_follow = False
            UDPHandler.cmd_triggered = False
            # print("[MODE] Come-To-Me mode activated. Rover will stop.")
            map_ctrl.stop()
            return

        if msg == "CMD_COME_TO_ME":
            UDPHandler.cmd_triggered = True
            # print(f"[CMD] Come-To-Me command triggered → going to last drone position ({self.last_good_x:.2f}, {self.last_good_y:.2f})")
            map_ctrl.go_to(self.last_good_x, self.last_good_y)
            return

        # === Otherwise: Pose Data ===
        parts = msg.split(",")
        if len(parts) < 4:
            # print(f"[WARN] Malformed UDP packet: {msg}")
            return

        ts, xd, yd, q = parts[:4]

        try:
            x = float(xd)
            y = float(yd)
        except ValueError:
            # print(f"[WARN] Invalid position values: {xd}, {yd}")
            x, y = 0, 0
        

        try:
            q = float(q)
        except ValueError:
            # print(f"[WARN] Invalid quality value: {q}")
            q = 0.0

        # Quality filter
        if q < self.quality_min:
            # print(f"[WARN] Low quality ({q:.0f}) — ignoring noisy data. (Last good: {self.last_good_x:.2f}, {self.last_good_y:.2f})")
            return

        # Update positions
        x, y, o = map_ctrl.get_position()
        print_replace(f"[DATA] Drone=({x:.2f}, {y:.2f}, Q={q:.0f}) | Rover=(x={x:.2f}, y={y:.2f}, o={o:.1f} rad)")

        self.drone_x, self.drone_y, self.quality = x, y, q
        self.last_good_x, self.last_good_y = x, y

        # Follow logic
        if UDPHandler.mode_follow and not UDPHandler.cmd_triggered:
            # print(f"[FOLLOW] Following drone → moving rover to ({x:.2f}, {y:.2f})")
            map_ctrl.go_to(x, y)
        else:
            pass
            # print(f"[FOLLOW] Follow disabled or CMD mode active — no movement command sent.")



        

def start_udp_server():
    # Replace with your desired UDP port
    udp_port = f['upd_control']['udp_port']
    with socketserver.UDPServer(("0.0.0.0", udp_port), UDPHandler) as server:
        print(f"UDP server listening on port {udp_port}")
        server.serve_forever()

LINE_CLEAR = '\x1b[2K'

def print_replace(string):
    sys.stdout.write(LINE_CLEAR + '\r' + string)
    sys.stdout.flush()

# Run the Flask app
if __name__ == "__main__":
    # breath light off
    base.change_breath_light_flag(False)
    
    # lights off
    base.lights_ctrl(255, 255)

    # set position to (0,0)
    map_ctrl.reset_position()
    
    # play a audio file in /sounds/robot_started/
    audio_ctrl.play_random_audio("robot_started", False)

    # update the size of videos and pictures
    si.update_folder(thisPath)

    # pt/arm looks forward
    if f['base_config']['module_type'] == 1:
        base.base_json_ctrl({"T":f['cmd_config']['cmd_arm_ctrl_ui'],"E":f['args_config']['arm_default_e'],"Z":f['args_config']['arm_default_z'],"R":f['args_config']['arm_default_r']})
    else:
        base.gimbal_ctrl(0, 0, 200, 10)

    # feedback loop starts
    si.start()
    si.resume()
    data_update_thread = threading.Thread(target=update_data_loop, daemon=True)
    data_update_thread.start()

    # base data update
    base_update_thread = threading.Thread(target=base_data_loop, daemon=True)
    base_update_thread.start()

    # lidar data update
    
    if base.use_lidar:
        lidar_update_thread = threading.Thread(target=lidar_data_loop, daemon=True)
        lidar_update_thread.start()

    # lights off
    base.lights_ctrl(0, 0)
    cmd_on_boot()

    # start UDP server in a separate thread
    udp_thread = threading.Thread(target=start_udp_server)
    udp_thread.daemon = True # Allow the thread to exit when the main program exits
    udp_thread.start()

    # run the main web app
    socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)

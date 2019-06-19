from flask import Flask, render_template, request, redirect, url_for
import subprocess
import signal
import time
import threading
import requests
app = Flask(__name__)

dynamic_list = False

servers = ['http://192.168.1.57:8080']
servo_list = []
motor_list = []

timer = None

control_process = None

def callback():
    global timer
    global servo_list
    global motor_list
    timer = threading.Timer(5.0, callback)
    timer.start()
    servos = []
    motors = []
    for server in servers:
        response = requests.get('{}/servos'.format(server))
        if response.status_code == 200:
            servos.extend(list(response.json()))
        response = requests.get('{}/motors'.format(server))
        if response.status_code == 200:
            motors.extend(list(response.json()))
    servo_list = servos
    motor_list = motors
    print("HELLO {}".format(servos))

if dynamic_list:
    timer = threading.Timer(1.0, callback)
    timer.start()
else:
    servo_list = ['Gripper', 'A_Hinge', 'A_Diff', 'B_Hinge', 'B_Diff']
    motor_list = []

@app.route('/startcontrol')
def start_control():
    global control_process
    control_process = subprocess.Popen(["python", "control.py"] + servers)
    return redirect(url_for('index'))

@app.route('/stopcontrol')
def stop_control():
    global control_process
    if control_process is not None:
        control_process.send_signal(signal.SIGINT)
        control_process.wait(5.0)
        control_process = None
    return redirect(url_for('index'))

@app.route('/setservo/<name>')
def set_servo(name):
    val = request.args["value"]
    for server in servers :
        requests.get('{}/servo'.format(server), params={"name": name, "value": val})
    return redirect(url_for('index'))

num = 0

@app.route('/control/<name>')
def control(name):
    return render_template('fine.html', name=name)

@app.route('/')
def index():
    global num
    num = num + 1
    status = "unknown"
    if control_process is None:
        status = "stopped"
    else :
        poll = control_process.poll()
        if poll is None:
            status = "running"
        else:
            status = "failed {}".format(poll)
    return render_template('index.html', servo_list=servo_list, status=status)


from smbus2 import SMBus
from gpiozero import Button, LED
import time
import signal
import sys
import subprocess
import requests

button = Button(18, pull_up=None, active_state=True, bounce_time=0.05)
leds = []
state = 0
current_grip = 0.5
grip_mul = 0.1
for p in [17, 27, 22]:
    leds.append(LED(p))
leds[0].on()

def inc():
    global state
    state = (state + 1) % len(leds)
    for i in range(len(leds)):
        if state == i:
            leds[i].on()
        else:
            leds[i].off()

button.when_pressed = inc


def read_pos_u16(bus, ident, offset):
    a = bus.read_byte_data(ident, offset)
    b = bus.read_byte_data(ident, offset+1)
    return ((a << 0x08) | b) >> 6

def read_pos_float(bus, ident, offset):
    return (read_pos_u16(bus, ident, offset) - 0x200)/0x200

def read_all(bus):
    out = [0.0, 0.0]
#    out[0] = read_pos_float(bus, 0x20, 0x3)
#    out[1] = read_pos_float(bus, 0x20, 0x5)
    out[0] = read_pos_float(bus, 0x21, 0x3)
    out[1] = read_pos_float(bus, 0x21, 0x5)
    for i in range(2) :
        if abs(out[i]) < 0.03 :
            out[i] = 0.0
    return out

def button_pressed(bus, ident):
    return bus.read_byte_data(ident, 0x08)
servers = sys.argv[1:]

bus = SMBus(1)
proc = subprocess.Popen(["./server"] + servers, stdin=subprocess.PIPE)


def handler(sig, frame):
    global bus
    global proc
    print("Exiting")
    proc.stdin.write(b'\nrest\n')
    proc.stdin.flush()
    time.sleep(0.3)
    proc.stdin.write(b'quit\n')
    #proc.stdin.flush()
    time.sleep(0.3)
    bus.close()
    sys.exit(0)

signal.signal(signal.SIGINT, handler)

mul = 3.0;

while True:
    try:
        inp = read_all(bus)
        print(inp)
        if state != 2:
            vals = [0.0, 0.0, 0.0, 0.0]
            if state == 0:
                vals[2] = -inp[0]
                vals[1] = inp[1]
            else:
                vals[3] = inp[0]
                vals[0] = inp[1]
            for i in range(4) :
                vals[i] *= mul
            proc.stdin.write(bytes("a {} {} {} {}\n".format(vals[0], vals[1], vals[2], vals[3]), "utf-8"))
            proc.stdin.flush()
        else:
            current_grip += inp[1] * grip_mul
            current_grip = max(0.0, min(1.0, current_grip))
            print("GRIP {}".format(current_grip))
            if inp[1] != 0.0:
                for serv in servers:
                    try:
                        requests.get("{}/servo?name=Gripper&value={}".format(serv, current_grip), timeout=0.05)
                    except:
                        pass
    except OSError:
        print("ERROR")
    time.sleep(0.2)

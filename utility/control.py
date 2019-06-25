from smbus2 import SMBus
import time
import signal
import sys
import subprocess

def read_pos_u16(bus, ident, offset):
    a = bus.read_byte_data(ident, offset)
    b = bus.read_byte_data(ident, offset+1)
    return ((a << 0x08) | b) >> 6

def read_pos_float(bus, ident, offset):
    return (read_pos_u16(bus, ident, offset) - 0x200)/0x200

def read_all(bus):
    out = [0.0, 0.0, 0.0, 0.0]
    out[0] = read_pos_float(bus, 0x20, 0x3)
    out[1] = read_pos_float(bus, 0x20, 0x5)
    out[2] = read_pos_float(bus, 0x21, 0x3)
    out[3] = read_pos_float(bus, 0x21, 0x5)
    for i in range(4) :
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
        vals = read_all(bus)
        print(vals)
        for i in range(4) :
            vals[i] *= mul
        proc.stdin.write(bytes("a {} {} {} {}\n".format(vals[0], vals[1], vals[2], vals[3]), "utf-8"))
        proc.stdin.flush()
    except OSError:
        print("ERROR")
    time.sleep(0.2)

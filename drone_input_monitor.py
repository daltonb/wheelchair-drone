import pyuv
import sys
import time
import signal
import evdev
import serial

debug = True

class DroneInputMonitor(object):

    HEADER1   = 0xAB
    HEADER2   = 0xCD
    LSTICK_X  = 0x01
    LSTICK_Y  = 0x02
    RSTICK_X  = 0x03
    RSTICK_Y  = 0x04
    TRIM      = 0x05
    TAKEOFF   = 0x06
    COMPASS   = 0x07
    SPEED     = 0x08

    def __init__(self):
        # main event loop
        self.loop = pyuv.Loop.default_loop()
        # set up Ctrl-C to exit
        pyuv.Signal(self.loop).start(self.sigint_callback, signal.SIGINT)

        self.ds4_init()
        self.wheelchair_js_init()
        self.drone_controller_init()

        self.ds4_poll_handle = pyuv.Poll(self.loop, self.ds4.fd).start(pyuv.UV_READABLE, self.ds4_callback)
        self.wheelchair_js_poll_handle = pyuv.Poll(self.loop, self.wheelchair_js.fd).start(pyuv.UV_READABLE, self.wheelchair_js_callback)
        self.idle_handle = pyuv.Idle(self.loop).start(self.idle_callback)

        # Start with PS4 controller (DualShock 4) overriding wheelchair joystick
        self.override = True

        # Assume wheelchair joystick starts off centered
        self.wheelchair_X_centered = self.wheelchair_Y_centered = True

    def start(self):
        # Start polling
        print("Starting event loop.")
        self.ds4_poll_handle.start(pyuv.UV_READABLE, self.ds4_callback)
        self.wheelchair_js_poll_handle.start(pyuv.UV_READABLE, self.wheelchair_js_callback)
        self.idle_handle.start(self.idle_callback)

        self.last_wheelchair_X_event = self.last_wheelchair_Y_event = time.time()

        # Run until Ctrl-C
        self.loop.run(pyuv.UV_RUN_DEFAULT)

    def ds4_callback(self, poll_handle, events, errorno):
        try:
            radius = 20 # dead zone
            for event in self.ds4.read():
                etype, ecode, val = event.type, event.code, event.value
                if etype == 3 and ecode == 0: # L stick X
                    self.send_cmd(self.LSTICK_X, int(val))
                    if debug: print('LSTICK_X', val)
                if etype == 3 and ecode == 1: # L stick Y
                    self.send_cmd(self.LSTICK_Y, int(val))
                    if debug: print('LSTICK_Y', val)
                if self.override:
                    if etype == 3 and ecode == 3: # R stick X
                        self.send_cmd(self.RSTICK_X, int(val))
                        if debug: print('RSTICK_X', val)
                    if etype == 3 and ecode == 4: # R stick Y
                        self.send_cmd(self.RSTICK_Y, int(val))
                        if debug: print('RSTICK_Y', val)
                if etype == 1 and ecode == 317: # L stick click
                    if val == 0 or val == 1:
                        self.send_cmd(self.TRIM, int(val))
                        if debug: print('TRIM', val)
                if etype == 1 and ecode == 304: # X button
                    if val == 0 or val == 1:
                        self.send_cmd(self.TAKEOFF, int(val))
                        if debug: print('TAKEOFF', val)
                if etype == 1 and ecode == 307: # Triangle button
                    if val == 0 or val == 1:
                        self.send_cmd(self.COMPASS, int(val))
                        if debug: print('COMPASS', val)
                if etype == 1 and ecode == 315: # Option button
                    if val == 0 or val == 1:
                        self.send_cmd(self.SPEED, int(val))
                        if debug: print('SPEED', val)
                if etype == 1 and ecode == 311: # R1 button
                    if val == 1:
                        if not self.override:
                            self.override = True
                        else:
                            self.override = False
                if etype == 1 and ecode == 313: # R2 button
                    if val == 1:
                        self.override = True
                    elif val == 0:
                        self.override = False

                if self.drone_controller.inWaiting():
                    self.drone_controller.readline()
        except IOError as e:
            if e.errno != errno.ENODEV: raise
            print("Controller disconnected")
            self.exit()

    def wheelchair_js_callback(self, poll_handle, events, errorno):
        try:
            for event in self.wheelchair_js.read():
                etype, ecode, val = event.type, event.code, event.value
                deadband = 10
                radius = 30
                if etype == 2 and ecode == 0:
                    if debug: print(etype, ecode, val)
                    if abs(val) < deadband:
                        val = 0
                        self.wheelchair_X_centered = True
                    else:
                        self.wheelchair_X_centered = False
                    if not self.override: self.send_cmd(self.RSTICK_X, self.range_map(val, -radius, radius, 0, 255))
                    if debug: print('wheelchair X', self.range_map(val, -radius, radius, 0, 255))
                    self.last_wheelchair_X_event = time.time()
                if etype == 2 and ecode == 1:
                    if debug: print(etype, ecode, val)
                    if abs(val) < deadband:
                        val = 0
                        self.wheelchair_Y_centered = True
                    else:
                        self.wheelchair_Y_centered = False
                    if not self.override: self.send_cmd(self.RSTICK_Y, self.range_map(val, -radius, radius, 0, 255))
                    if debug: print('wheelchair Y', self.range_map(val, -radius, radius, 0, 255))
                    self.last_wheelchair_Y_event = time.time()
        except IOError as e:
            if e.errno != errno.ENODEV: raise
            print("Controller disconnected")
            self.exit()

    def idle_callback(self, idle_handle):
        # auto-center wheelchair joystick if no data for > 0.1s
        # (otherwise drone will just keep on flying!)
        timeout = 0.1
        timestamp = time.time()
        if not self.wheelchair_X_centered and (timestamp - self.last_wheelchair_X_event > timeout):
            self.send_cmd(self.RSTICK_X, 128)
            if debug: print('wheelchair X', 128)
            self.wheelchair_X_centered = True
        if not self.wheelchair_Y_centered and (timestamp - self.last_wheelchair_Y_event > timeout):
            self.send_cmd(self.RSTICK_Y, 128)
            self.wheelchair_Y_centered = True

    def ds4_init(self):
        self.ds4 = None
        print("Scanning for PS4 controller...")
        while not self.ds4:
            devices = [evdev.InputDevice(dev) for dev in evdev.list_devices()]
            for dev in devices:
                if dev.name == "Sony Computer Entertainment Wireless Controller":
                    self.ds4 = dev
                    print("PS4 controller connected.")

    def wheelchair_js_init(self):
        self.wheelchair_js = None
        print("Scanning for wheelchair joystick...")
        while not self.wheelchair_js:
            devices = [evdev.InputDevice(dev) for dev in evdev.list_devices()]
            for dev in devices:
                if dev.name == "Mouse0660":
                    self.wheelchair_js = dev
                    print("Wheelchair joystick connected")
                    self.wheelchair_js.grab()

    def drone_controller_init(self):
        self.drone_controller = serial.Serial('/dev/ttyACM0', 115200)

    def send_cmd(self, cmd, val):
        self.drone_controller.write(chr(self.HEADER1))
        self.drone_controller.write(chr(self.HEADER2))
        self.drone_controller.write(chr(cmd))
        self.drone_controller.write(chr(val))

    def range_map(self, val, in_min, in_max, out_min, out_max):
        mapped_val = int(round((val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min))
        if mapped_val < out_min: return int(round(out_min))
        if mapped_val > out_max: return int(round(out_max))
        return mapped_val

    def sigint_callback(self, signal_handle, signum):
        self.exit()

    def exit(self):
        print("exiting")
        #self.wheelchair_js.ungrab()
        self.loop.stop()

if __name__ == '__main__':
    DroneInputMonitor().start()

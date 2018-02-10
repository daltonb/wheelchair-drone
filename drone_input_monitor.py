import pyuv
import sys
import time
import signal
import evdev
import serial

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
        # 
        self.ds4_poll_handle = pyuv.Poll(self.loop, self.ds4.fd).start(pyuv.UV_READABLE, self.ds4_callback)
        self.wheelchair_js_poll_handle = pyuv.Poll(self.loop, self.wheelchair_js.fd).start(pyuv.UV_READABLE, self.wheelchair_js_callback)
        self.idle_handle = pyuv.Idle(self.loop).start(self.idle_callback)
        # set up Ctrl-C to exit
        pyuv.Signal(self.loop).start(self.sigint_callback, signal.SIGINT)

        self.ds4_init()
        self.wheelchair_js_init()
        self.drone_controller_init()

        # Start with PS4 controller (DualShock 4) overriding wheelchair joystick
        self.override = True

        # Assume wheelchair joystick starts off centered
        self.wheelchair_X_centered = self.wheelchair_Y_centered = True

    def start(self):
        # Start polling
        self.ds4_poll_handle.start(pyuv.UV_READABLE, self.ds4_callback)
        self.wheelchair_js_poll_handle.start(pyuv.UV_READABLE, self.wheelchair_js_callback)
        self.idle_handle.start(self.idle_callback)

        self.last_wheelchair_X_event = self.last_wheelchair_Y_event = time.time()

        # Run until Ctrl-C
        self.loop.run(pyuv.UV_RUN_DEFAULT)

    def ds4_callback(self, poll_handle, events, errorno):
        try:
            radius = 20
            for event in self.ds4.read():
                etype, ecode, val = event.type, event.code, event.value
                if etype == 3 and ecode == 0:
                    self.send_cmd(self.LSTICK_X, int(val))
                    print 'LSTICK_X', val
                if etype == 3 and ecode == 1:
                    self.send_cmd(self.LSTICK_Y, int(val))
                    print 'LSTICK_Y', val
                if self.override:
                    if etype == 3 and ecode == 3:
                        self.send_cmd(self.RSTICK_X, int(val))
                        print 'RSTICK_X', val
                    if etype == 3 and ecode == 4:
                        self.send_cmd(self.RSTICK_Y, int(val))
                        print 'RSTICK_Y', val
                if etype == 1 and ecode == 317: # L stick click
                    if val == 0 or val == 1:
                        self.send_cmd(self.TRIM, int(val))
                        print 'TRIM', val
                if etype == 1 and ecode == 304: # X
                    if val == 0 or val == 1:
                        self.send_cmd(self.TAKEOFF, int(val))
                        print 'TAKEOFF', val
                if etype == 1 and ecode == 307: # TRIANGLE
                    if val == 0 or val == 1:
                        self.send_cmd(self.COMPASS, int(val))
                        print 'COMPASS', val
                if etype == 1 and ecode == 315: # OPTION
                    if val == 0 or val == 1:
                        self.send_cmd(self.SPEED, int(val))
                        print 'SPEED', val
                if etype == 1 and ecode == 311: # R1
                    if val == 1:
                        if not self.override:
                            self.override = True
                        else:
                            self.override = False
                if etype == 1 and ecode == 313: # R2
                    if val == 1:
                        self.override = True
                    elif val == 0:
                        self.override = False

                if self.drone_controller.inWaiting():
                    self.drone_controller.readline()
        except IOError as e:
            if e.errno != errno.ENODEV: raise
            print "Controller disconnected"
            self.exit()

    def wheelchair_js_callback(self, poll_handle, events, errorno):
        try:
            for event in self.wheelchair_js.read():
                etype, ecode, val = event.type, event.code, event.value
                deadband = 10
                radius = 30
                if etype == 2 and ecode == 0:
                    print etype, ecode, val
                    if abs(val) < deadband:
                        val = 0
                        self.wheelchair_X_centered = True
                    else:
                        self.wheelchair_X_centered = False
                    if not self.override: self.send_cmd(self.RSTICK_X, self.range_map(val, -radius, radius, 0, 255))
                    print 'wheelchair X', self.range_map(val, -radius, radius, 0, 255)
                    self.last_wheelchair_X_event = time.time()
                if etype == 2 and ecode == 1:
                    print etype, ecode, val
                    if abs(val) < deadband:
                        val = 0
                        self.wheelchair_Y_centered = True
                    else:
                        self.wheelchair_Y_centered = False
                    if not self.override: self.send_cmd(self.RSTICK_Y, self.range_map(val, -radius, radius, 0, 255))
                    print 'wheelchair Y', self.range_map(val, -radius, radius, 0, 255)
                    self.last_wheelchair_Y_event = time.time()
        except IOError as e:
            if e.errno != errno.ENODEV: raise
            print "Controller disconnected"
            self.exit()

    def idle_callback(self, idle_handle):
        timeout = 0.1
        timestamp = time.time()
        if not self.wheelchair_X_centered and (timestamp - self.last_wheelchair_X_event > timeout):
            self.send_cmd(self.RSTICK_X, 128)
            print 'wheelchair X', 128
            self.wheelchair_X_centered = True
        if not self.wheelchair_Y_centered and (timestamp - self.last_wheelchair_Y_event > timeout):
            self.send_cmd(self.RSTICK_Y, 128)
            self.wheelchair_Y_centered = True

    def ds4_init(self):
        self.ds4 = None
        while not self.ds4:
            print "Scanning for PS4 controller..."
            devices = [evdev.InputDevice(dev) for dev in evdev.list_devices()]
            for dev in devices:
                if dev.name == "Sony Computer Entertainment Wireless Controller":
                    self.ds4 = dev
                    print "PS4 controller connected."

    def wheelchair_js_init(self):
        self.wheelchair_js = None
        while not self.wheelchair_js:
            print "Scanning for wheelchair joystick..."
            devices = [evdev.InputDevice(dev) for dev in evdev.list_devices()]
            for dev in devices:
                if dev.name == "Mouse0660":
                    self.wheelchair_js = dev
                    print "Wheelchair joystick connected"
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
        print "exiting"
        #self.wheelchair_js.ungrab()
        self.loop.stop()

if __name__ == '__main__':
    DroneInputMonitor().start()

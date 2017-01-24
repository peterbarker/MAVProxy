#!/usr/bin/env python
'''Use a video-control script with the same interface as understood
by ArduPilot'''

import time, os, sys
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
import subprocess

class ArduPilotVideoScript(mp_module.MPModule):
    def __init__(self, mpstate):
        super(ArduPilotVideoScript, self).__init__(mpstate, "ArduPilotVideoScript")
        self.add_command('avs', self.cmd_videoscript, "ardupilot video script ops",
                         ["set"])

        self.avs_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),
              ('avs_script_path', str, "camera_control"), # assume in path
              ('trigger_channel', int, -1),
              ('trigger_low', int, 1200), # considered low below this
              ('trigger_high', int, 1800), # considered high above this
          ])

        self.trigger_state_high = 1
        self.trigger_state_low = 0
        self.trigger_value = -1
        self.old_trigger_value = -1
        self.trigger_state = -1

    def usage(self):
        return "Usage: avs set name=value"

    def cmd_videoscript(self, args):
        '''video script commands'''
        if len(args) == 0:
            print self.usage()
        elif args[0] == "set":
            self.avs_settings.command(args[1:])
        else:
            print(self.usage())

    def run_script(self, arguments):
        cmd = [ self.avs_settings.avs_script_path ]
        cmd.extend(arguments)
        print("Running (%s)" % str(cmd))
        subprocess.Popen(cmd)

    def trigger_low(self):
            self.run_script(['record','stop'])

    def trigger_high(self):
            self.run_script(['record','start'])

    # def trigger_low(self):
    #         self.run_script("stream stop")

    # def trigger_high(self):
    #         self.run_script("stream start")

    def check_rc_trigger(self):
        ''' this is set up so we only work on a transition'''
        if self.trigger_value == -1:
            return
        if self.old_trigger_value == -1:
            self.old_trigger_value = self.trigger_value
            return
        if abs(self.trigger_value - self.old_trigger_value) < 5:
            # allow RC input to wobble a little
            return

        # could add hysteresis here, but we only get messages at 10Hz anyway...
        print("Trigger_Value: %u\n", self.trigger_value)
        value = self.trigger_value
        self.old_trigger_value = self.trigger_value
        self.trigger_value = -1
        if (self.trigger_state != self.trigger_state_low and
            value < self.avs_settings.trigger_low):
            self.trigger_state = self.trigger_state_low
            self.trigger_low()
        elif (self.trigger_state == self.trigger_state_low and
              value > self.avs_settings.trigger_high):
            self.trigger_state = self.trigger_state_high
            self.trigger_high()

    def idle_task(self):
        '''called rapidly by mavproxy'''
        self.check_rc_trigger()

    def mavlink_packet(self, m):
        '''handle a mavlink packet'''
        mtype = m.get_type()
        if mtype == "RC_CHANNELS":
            if self.avs_settings.trigger_channel != -1:
                string = "m.chan%d_raw" % (self.avs_settings.trigger_channel)
                self.trigger_value = eval(string)

def init(mpstate):
    '''initialise module'''
    return ArduPilotVideoScript(mpstate)

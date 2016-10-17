#!/usr/bin/env python
'''
ESP Module
Peter Barker, October 2016

TODO: Fill this descrition in...

'''

import errno
import fnmatch
import os
import os.path
import Queue
import sys
import time
import struct

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings
from mavproxy_param import ParamState
from pymavlink import mavparm
from pymavlink import mavutil

class MAVSystem(mavutil.mavsource):
    def __init__(self, conn, target_system=None, target_component=1):
        super(MAVSystem,self).__init__(conn, target_system, target_component)
        self.mav = conn.mav


class esp_ParamState(ParamState):
    def __init__(self, params, logdir, vehicle_name, logfile):
        super(esp_ParamState,self).__init__(params, logdir, vehicle_name, logfile)
        self.params = params

    def emit_param_value(self, name, value):
        print("%-16.16s %s" % (name, value))

    def handle_command_show_encoded_string(self, conn, mpstate, args, name):
        v1 = self.params.get(name + str(1))
        v2 = self.params.get(name + str(2))
        v3 = self.params.get(name + str(3))
        v4 = self.params.get(name + str(4))

        if None in [v1, v2, v3, v4]:
            return

        x = struct.pack("ffff", v1, v2, v3, v4)
        chars = struct.unpack("cccccccccccccccc", x)
        v = str("".join(list(chars)))
        self.emit_param_value(name, v)

    def handle_command_show(self, conn, mpstate, args):
        wildcard = "*"
        if len(args):
            wildcard = args[0]
        # swiped from mavparm.py in pymavlink
        for p in sorted(self.params.keys()):
            if fnmatch.fnmatch(str(p).upper(), wildcard.upper()):
                handled_encoded_string = False
                for encoded_string_param in [ "WIFI_PASSWORD",
                                              "WIFI_STA",
                                              "WIFI_PWDSTA",
                                              "WIFI_SSID" ]:
                    if str(p).find(encoded_string_param) == 0:
                        if str(p).find(encoded_string_param+str(1)) == 0:
                            self.handle_command_show_encoded_string(conn, mpstate, args, encoded_string_param)
                        handled_encoded_string = True
                        break
                if handled_encoded_string:
                    continue
                if p in [ "WIFI_IPADDRESS", "WIFI_IPSTA", "WIFI_GATEWAYSTA", "WIFI_SUBNET_STA" ]:
                    x = struct.pack("f", self.params.get(p))
                    self.emit_param_value(p, "%u.%u.%u.%u" % struct.unpack("BBBB", x))
                    continue
                if p == "SW_VER":
                    x = struct.pack("f", self.params.get(p))
                    (a,b,c,d) = (struct.unpack("BBBB", x))
                    self.emit_param_value(p, "%u.%u.%u" % (d,c,b))
                    continue
                if p in [ "WIFI_CHANNEL", "WIFI_UDP_CPORT", "WIFI_UDP_HPORT", "UART_BAUDRATE"]:
                    x = struct.pack("f", self.params.get(p))
                    (a,b,c,d) = (struct.unpack("BBBB", x))
                    value = ((d<<16)+(c<<16)+(b<<8)+a)
                    self.emit_param_value(p, "%u" % value)
                    continue

                self.emit_param_value(str(p), str(self.params.get(p)))

    def handle_command(self, conn, mpstate, args):
        if args[0] == "show":
            return self.handle_command_show(conn, mpstate, args[1:])
        super(esp_ParamState, self).handle_command(conn, mpstate, args)

class esp(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(esp, self).__init__(mpstate, "esp", "")
        self.params = mavparm.MAVParmDict()
        self.param_state = esp_ParamState(self.params, self.logdir, self.vehicle_name, "mav-esp.parm")
        self.verbose = False

#            ('target-component', int, 1),
        self.esp_settings = mp_settings.MPSettings([
            ('verbose', bool, True),
            ('target_system', int, 1),
            ('target_component', int, mavutil.mavlink.MAV_COMP_ID_UDP_BRIDGE),
        ])
        self.add_command('esp', self.cmd_esp, "ESP module", ['status','set (LOGSETTING)'])

        self.conn = mavutil.mavsource(self.master, self.esp_settings.target_system, self.esp_settings.target_component)

    def usage(self):
        '''show help on command line options'''
        return "Usage: esp <status|set|param>"

    def cmd_esp(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print self.usage()
        elif args[0] == "status":
            print self.status()
        elif args[0] == "set":
            self.esp_settings.command(args[1:])
        elif args[0] == "param":
            self.param_state.handle_command(self.conn, None, args[1:])
        else:
            print self.usage()

    def status(self):
        '''returns information about module'''
        return "OK"

    def idle_task(self):
        '''called rapidly by mavproxy'''
        try:
            m = self.conn.to_source.get(block=False)
            if m._type == "BAD_DATA":
                return
            self.param_state.handle_mavlink_packet(self.conn, m)

        except Queue.Empty :
            return

def init(mpstate):
    '''initialise module'''
    return esp(mpstate)

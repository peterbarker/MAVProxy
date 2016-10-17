#!/usr/bin/env python
'''
ESP Module
Peter Barker, October 2016

This module simply serves as a starting point for your own MAVProxy module.

1. copy this module sidewise (e.g. "cp mavproxy_ESP.py mavproxy_coolfeature.py"
2. replace all instances of "ESP" with whatever your module should be called
(e.g. "coolfeature")

3. trim (or comment) out any functionality you do not need
'''

import errno
import fnmatch
import os
import os.path
import Queue
import sys
import time

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
        print("%-16.16s %f" % (name, value))

    def handle_command_show(self, conn, mpstate, args):
        wildcard = "*"
        if len(args):
            wildcard = args[0]
        # swiped from mavparm.py in pymavlink
        for p in sorted(self.params.keys()):
            if fnmatch.fnmatch(str(p).upper(), wildcard.upper()):
                if p == "WIFI_PASSWORD1":
                    for i in range(1,6):
                        v = v << 8
                        v |= self.get("WIFI_PASSWORD" + str(i))
                    pass
                    self.emit("WIFI_PASSWORD", v)
                else:
                    self.emit_param_value(str(p), self.get(p))

    def handle_command(self, conn, mpstate, args):
        if args[0] == "show":
            return self.handle_command_show(conn, mpstate, args)
        super(esp_ParamState, self).handle_command(conn, mpstate, args)

class esp(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(esp, self).__init__(mpstate, "esp", "")
        self.params = mavparm.MAVParmDict()
        self.param_state = ParamState(self.params, self.logdir, self.vehicle_name, "mav-esp.parm")
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

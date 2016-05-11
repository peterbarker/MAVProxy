#!/usr/bin/env python
'''
Avoidance reporting library
June 2015

This module interprets the mavlink COLLISION message and simply issues warnings based on those messages
'''

import logging
import os
import os.path
import threading
import types
import sys
from pymavlink import mavutil
import errno

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
import time
from MAVProxy.modules.lib import mp_settings


class mavproxy_avoidance(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module.  """
        super(mavproxy_avoidance, self).__init__(mpstate, "mavproxy_avoidance", "intpretation of COLLISION mavlink messages")
#        self.add_command('dataflash_logger', self.cmd_dataflash_logger, "dataflash logging control", ['status','start','stop','set (LOGSETTING)'])
#        self.add_completion_function('(LOGSETTING)', self.log_settings.completion)

#    def usage(self):
#        '''show help on a command line options'''
#        return "Usage: dataflash_logger <status|start|stop|set>"


    def mavlink_packet_collision(self, m):
        now = time.time()

        print("Got packet from (%d/%d)" % (m.get_srcSystem(), m.get_srcComponent()))
        print(" opaque id is %d" % (m.id,))
        print(" threat level is %s" % (m.threat_level,))
        print(" threat level is %s" % (str(mavutil.mavlink.enums['MAV_COLLISION_ACTION'][m.threat_level],)))

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        if m.get_type() == 'COLLISION':
            self.mavlink_packet_collision(m)

def init(mpstate):
    '''initialise module'''
    return mavproxy_avoidance(mpstate)

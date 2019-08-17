"""
    MAVProxy geofence module
"""
import time

from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings


class RallyFenceShim(mp_module.MPModule):
    '''a shim that brings out common functionality in the fence/rally shim
     modules which are used to switch between old protocols and the
    MISSION_ITEM protocol'''
    def __init__(self, mpstate, name, *args, **kwargs):
        super(RallyFenceShim, self).__init__(mpstate, name, *args, **kwargs)
        self.name = name
        self.using_new_module = False
        self.system_info_by_sysid = dict()
        self.childmodule = self.create_oldmodule(mpstate)

        self.shim_settings = mp_settings.MPSettings([
            ('verbose', bool, False),
            ('force_old_modules', bool, False),
        ])
        self.add_command('%s_shim' % self.name,
                         self.cmd_shim,
                         "Rally module shim commands",
                         ['set (SETTING)'])
        self.add_completion_function('(SETTING)',
                                     self.shim_settings.completion)

        self.fallforward_impossible = False

    def unload(self):
        '''unload child module if we are unloaded'''
        super(RallyFenceShim, self).unload()
        self.childmodule.unload()

    def usage(self):
        return "%s shim set (SETTING)" % self.name

    def cmd_shim(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "set":
            self.shim_settings.command(args[1:])
        else:
            print(self.usage())

    def create_oldmodule(self, mpstate):
        c = self.oldmodule_class()
        return c(mpstate)

    def create_newmodule(self, mpstate):
        c = self.newmodule_class()
        return c(mpstate)

    def trigger_fallback(self):
        print("Falling back to old module (%s)" % str(self.newmodule_class()))
        self.childmodule.unload()
        self.childmodule = self.create_oldmodule(self.mpstate)
        self.using_new_module = False

    def trigger_fallforward(self, source_system):
        print("Falling forward to new module (%s)" % str(self.newmodule_class()))
        self.system_info_by_sysid[source_system]["send_version_requests"] = 0
        self.system_info_by_sysid[source_system]["capable"] = True
        if self.fallforward_impossible:
            return
        if not self.master.mavlink20():
            self.fallforward_impossible = True
            return
        if self.using_new_module:
            return
        if self.shim_settings.force_old_modules:
            return
        try:
            c = self.newmodule_class().loader_class()
        except AttributeError as e:
            print("pymavlink too old - not falling forward to new module")
            self.fallforward_impossible = True
            return
        self.childmodule.unload()
        self.childmodule = self.create_newmodule(self.mpstate)
        self.using_new_module = True

    def reset_system_info(self, source_system):
        self.system_info_by_sysid[source_system] = {
            'uptime': 0,
            'last_version_request': 0,
            'send_version_requests': 0,
            'request_message_ok': True,
            'request_autopilot_capabilities_ok': True,
            'capable': None,
        }

    def __getattr__(self, name):
        return getattr(self.childmodule, name)

    def send_version_request(self, system):
        info = self.system_info_by_sysid[system]
        if info["request_message_ok"]:
            self.master.mav.command_long_send(
                system,
                0,  # target component
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                0,
                mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
                1, 0, 0, 0, 0, 0, 0)
        if info["request_autopilot_capabilities_ok"]:
            self.master.mav.command_long_send(
                system,
                0,  # target component
                mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
                0,
                1,  # 1: Request autopilot version
                1, 0, 0, 0, 0, 0, 0)

    def idle_task(self):
        it = getattr(self.childmodule, 'idle_task', None)
        if it is not None:
            it()

    def mavlink_packet(self, m):
        if self.using_new_module:
            if self.shim_settings.force_old_modules:
                self.trigger_fallback()
                return

        source_system = m.get_srcSystem()

        t = m.get_type()

        # monitor command acks to see if our requests are being
        # bounced by the autopilot:
        if t == "COMMAND_ACK":
            if m.result in (mavutil.mavlink.MAV_RESULT_UNSUPPORTED,
                            mavutil.mavlink.MAV_RESULT_FAILED,
                            mavutil.mavlink.MAV_RESULT_DENIED):
                if source_system in self.system_info_by_sysid:
                    info = self.system_info_by_sysid[source_system]
                    if m.command == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE:
                        print("request_message got bounced")
                        info["request_message_ok"] = False
                    if m.command == mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:  # NOQA: E501
                        print("request_autopilot_capabilities_ok got bounced")
                        info["request_autopilot_capabilities_ok"] = False

        if (source_system == self.target_system and
            (self.target_component == 0 or
             m.get_srcComponent() == self.target_component)):
            if source_system not in self.system_info_by_sysid:
                print("A new victim: %u" % source_system)
                self.reset_system_info(source_system)
                self.system_info_by_sysid[source_system]["send_version_requests"] = 5  # NOQA: E501

            info = self.system_info_by_sysid[source_system]

            # test to see if we can fall forward:
            if self.system_info_by_sysid[source_system]["capable"] is None:
                if t == "AUTOPILOT_VERSION":
                    print("Got autopilot_version")
                    if m.capabilities & self.capability_bit_required():
                        self.trigger_fallforward(source_system)
                    # we have our answer - stop asking the question:
                    info["send_version_requests"] = 0

            # if a system goes away for more than ten seconds then we
            # consider our information about it void, and we may
            # re-probe it
            uptime = getattr(m, 'time_boot_ms', None)
            if uptime is not None and uptime != 0:  # 0 - some messages bad
                if info['uptime'] - uptime > 10000:
                    print("fence: system reboot detected")
                    self.reset_system_info(source_system)
                    info = self.system_info_by_sysid[source_system]
                    self.system_info_by_sysid[source_system]["capable"] = None
                    info["send_version_requests"] = 5
                info["uptime"] = uptime

            # not using new module; send requests if we haven't tried enough
            if info['send_version_requests'] > 0:
                now = time.time()
                if now - info['last_version_request'] > 1:
                    print("Sending version requests (%u)" %
                          info['send_version_requests'])
                    info['last_version_request'] = now
                    self.send_version_request(source_system)
                    info['send_version_requests'] -= 1

        # check to see if anybody there's a system using the new
        # module.  If not, fall back to the old module.  When flashing
        # forwards and backwards in firmware this may be useful.
        if self.using_new_module:
            found = False
            for source_system in self.system_info_by_sysid.keys():
                if self.system_info_by_sysid[source_system]["capable"]:
                    found = True
                    break
            if not found:
                self.trigger_fallback()

        # now pass the packet to the real module:
        self.childmodule.mavlink_packet(m)

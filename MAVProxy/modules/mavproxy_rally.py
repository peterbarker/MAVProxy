"""
    MAVProxy rally module
"""

from pymavlink import mavwp
from pymavlink import mavutil
import time, os, platform
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mission_item_protocol
from MAVProxy.modules.lib import rally_fence_shim_module

if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import *

class RallyModule(rally_fence_shim_module.RallyFenceShim):
    '''RallyModule - a shim/wrapper class around the rally module - which
    uses the MissionItemProtocol to upload fences - and the old module
    which uses the RALLY_POINT protocol to upload rally points.  It
    first uses the old RALLY_POINT protocol, but if it detects the
    vehicle can handle the new protocol (via
    AUTOPILOT_VERSION.capabilities) then it switches to the new
    module.
    '''

    def __init__(self, mpstate):
        '''initialise shim module - relying on base class for most of it'''
        super(RallyModule, self).__init__(mpstate, "rally", "rally point management (wrapper)", public = True)

    def oldmodule_class(self):
        '''if we shouldn't use the mission item protocol, use this legacy
        class instead'''
        return OldRallyModule

    def newmodule_class(self):
        '''if we should use the mission item protocol, use this class'''
        return NewRallyModule

    def capability_bit_required(self):
        '''returns bit found in AUTOPILOT_VERSION.capabilities that indicates
        uploading via mission item protocol is possible'''
        return mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY

class NewRallyModule(mission_item_protocol.MissionItemProtocolModule):
    '''uses common MISSION_ITEM protocol base class to provide rally
    upload/download
    '''

    def __init__(self, mpstate):
        '''initialise module; will raise AttributeError if pymavlink is too
        old to use'''
        # raise an attribute error if pymavlink is too old:
        mavwp.MissionItemProtocol_Rally
        super(NewRallyModule, self).__init__(mpstate,
                                             "rallynew",
                                             "rally point management (new)",
                                             public = True)

    def command_name(self):
        '''command-line command name'''
        return "rally"

    def cmd_rally_add(self, args):
        '''add a rally point at the last map click position'''
        if not self.check_have_list():
            return
        latlon = self.mpstate.click_location
        if latlon is None:
            print("No click position available")
            return

        if len(args) < 1:
            alt = self.settings.rallyalt
        else:
            alt = float(args[0])

        m = mavutil.mavlink.MAVLink_mission_item_int_message(
            self.target_system,
            self.target_component,
            0,    # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,    # frame
            mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,    # command
            0,    # current
            0,    # autocontinue
            0.0,  # param1,
            0.0,  # param2,
            0.0,  # param3
            0.0,  # param4
            int(latlon[0] * 1e7),  # x (latitude)
            int(latlon[1] * 1e7),  # y (longitude)
            alt,                   # z (altitude)
            self.mav_mission_type(),
        )
        self.append(m)
        self.send_all_items()

    def rally_point(self, i):
        '''return an instance of the old mavlink rally_point message for the
        item at offset i'''
        i = self.wploader.item(i)
        if i is None:
            return None
        lat = i.x
        lng = i.y
        alt = i.z
        if i.get_type() == "MISSION_ITEM":
            lat *= 1e7
            lng *= 1e7
            alt *= 100
        return mavutil.mavlink.MAVLink_rally_point_message(
            i.target_system,
            i.target_component,
            i.seq,
            self.wploader.count(),
            lat,
            lng,
            alt,
            0,
            0,
            0)

    def rally_count(self):
        '''return number of waypoints'''
        return self.wploader.count()

    def commands(self):
        '''returns map from command name to handling function'''
        ret = super(NewRallyModule, self).commands()
        ret.update({
            "alt": self.cmd_changealt, # backwards-compatability
            'add': self.cmd_rally_add,
        })
        return ret

    @staticmethod
    def loader_class():
        return mavwp.MissionItemProtocol_Rally

    def mav_mission_type(self):
        return mavutil.mavlink.MAV_MISSION_TYPE_RALLY

    def itemstype(self):
        '''returns description of items in the plural'''
        return 'rally items'

    def itemtype(self):
        '''returns description of item'''
        return 'rally item'

    def mavlink_packet(self, p):
        super(NewRallyModule, self).mavlink_packet(p)

    def gui_menu_items(self):
        ret = super(NewRallyModule, self).gui_menu_items()
        ret.extend([
            MPMenuItem('Add', 'Add', '# rally add ',
                       handler=MPMenuCallTextDialog(
                           title='Rally Altitude (m)',
                           default=100
                       )
            ),
        ])
        return ret

class OldRallyModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(OldRallyModule, self).__init__(mpstate, "rallyold", "rally point control (old)", public = True)
        self.rallyloader_by_sysid = {}
        self.add_command('rally', self.cmd_rally, "rally point control", ["<add|clear|land|list|move|remove|>",
                                    "<load|save> (FILENAME)"])
        self.have_list = False
        self.abort_alt = 50
        self.abort_first_send_time = 0
        self.abort_previous_send_time = 0
        self.abort_ack_received = True

        self.menu_added_console = False
        self.menu_added_map = False
        if mp_util.has_wxpython:
            self.menu = MPMenuSubMenu('Rally',
                                  items=[MPMenuItem('Clear', 'Clear', '# rally clear'),
                                         MPMenuItem('List', 'List', '# rally list'),
                                         MPMenuItem('Load', 'Load', '# rally load ',
                                                    handler=MPMenuCallFileDialog(flags=('open',),
                                                                                 title='Rally Load',
                                                                                 wildcard='*.rally')),
                                         MPMenuItem('Save', 'Save', '# rally save ',
                                                    handler=MPMenuCallFileDialog(flags=('save', 'overwrite_prompt'),
                                                                                 title='Rally Save',
                                                                                 wildcard='*.rally')),
                                         MPMenuItem('Add', 'Add', '# rally add ',
                                                    handler=MPMenuCallTextDialog(title='Rally Altitude (m)',
                                                                                 default=100))])

    @property
    def rallyloader(self):
        '''rally loader by system ID'''
        if not self.target_system in self.rallyloader_by_sysid:
            self.rallyloader_by_sysid[self.target_system] = mavwp.MAVRallyLoader(self.settings.target_system,
                                                                                 self.settings.target_component)
        return self.rallyloader_by_sysid[self.target_system]

    def last_change(self):
        '''return time of last changes made to rally points'''
        return self.rallyloader.last_change

    def rally_count(self):
        '''return number of waypoints'''
        return self.rallyloader.rally_count()

    def rally_point(self, i):
        '''return instance of mavutil.mavlink.MAVLink_rally_point_message'''
        return self.rallyloader.rally_point(i)

    def set_last_change(self, time):
        '''can be used to cause map redraws'''
        self.rallyloader.last_change = time

    def idle_task(self):
        '''called on idle'''
        if self.module('console') is not None:
            if not self.menu_added_console:
                self.menu_added_console = True
                self.module('console').add_menu(self.menu)
        else:
            self.menu_added_console = False

        if self.module('map') is not None:
            if not self.menu_added_map:
                self.menu_added_map = True
                self.module('map').add_menu(self.menu)
        else:
            self.menu_added_map = False

        '''handle abort command; it is critical that the AP to receive it'''
        if self.abort_ack_received is False:
            #only send abort every second (be insistent, but don't spam)
            if (time.time() - self.abort_previous_send_time > 1):
                self.master.mav.command_long_send(self.settings.target_system,
                    self.settings.target_component,
                    mavutil.mavlink.MAV_CMD_DO_GO_AROUND,
                    0, int(self.abort_alt), 0, 0, 0, 0, 0, 0,)
                self.abort_previous_send_time = time.time()

            #try to get an ACK from the plane:
            if self.abort_first_send_time == 0:
                self.abort_first_send_time = time.time()
            elif time.time() - self.abort_first_send_time > 10: #give up after 10 seconds
                print("Unable to send abort command!\n")
                self.abort_ack_received = True


    def cmd_rally_add(self, args):
        '''handle rally add'''
        if len(args) < 1:
            alt = self.settings.rallyalt
        else:
            alt = float(args[0])

        if len(args) < 2:
            break_alt = self.settings.rally_breakalt
        else:
            break_alt = float(args[1])

        if len(args) < 3:
            flag = self.settings.rally_flags
        else:
            flag = int(args[2])
            #currently only supporting autoland values:
            #True (nonzero) and False (zero)
            if (flag != 0):
                flag = 2

        if not self.have_list:
            print("Please list rally points first")
            return

        if (self.rallyloader.rally_count() > 4):
            print("Only 5 rally points possible per flight plan.")
            return

        latlon = self.mpstate.click_location
        if latlon is None:
            print("No click position available")
            return

        land_hdg = 0.0

        self.rallyloader.create_and_append_rally_point(latlon[0] * 1e7, latlon[1] * 1e7, alt, break_alt, land_hdg, flag)
        self.send_rally_points()
        print("Added Rally point at %s %f %f, autoland: %s" % (str(latlon), alt, break_alt, bool(flag & 2)))

    def cmd_rally_alt(self, args):
        '''handle rally alt change'''
        if (len(args) < 2):
            print("Usage: rally alt RALLYNUM newAlt <newBreakAlt>")
            return
        if not self.have_list:
            print("Please list rally points first")
            return

        idx = int(args[0])
        if idx <= 0 or idx > self.rallyloader.rally_count():
            print("Invalid rally point number %u" % idx)
            return

        new_alt = int(args[1])
        new_break_alt = None
        if (len(args) > 2):
            new_break_alt = int(args[2])

        self.rallyloader.set_alt(idx, new_alt, new_break_alt)
        self.send_rally_point(idx-1)
        self.fetch_rally_point(idx-1)
        self.rallyloader.reindex()

    def cmd_rally_move(self, args):
        '''handle rally move'''
        if len(args) < 1:
            print("Usage: rally move RALLYNUM")
            return
        if not self.have_list:
            print("Please list rally points first")
            return

        idx = int(args[0])
        if idx <= 0 or idx > self.rallyloader.rally_count():
            print("Invalid rally point number %u" % idx)
            return

        rpoint = self.rallyloader.rally_point(idx-1)

        latlon = self.mpstate.click_location
        if latlon is None:
            print("No click position available")
            return

        oldpos = (rpoint.lat*1e-7, rpoint.lng*1e-7)
        self.rallyloader.move(idx, latlon[0], latlon[1])
        self.send_rally_point(idx-1)
        p = self.fetch_rally_point(idx-1)
        if p.lat != int(latlon[0]*1e7) or p.lng != int(latlon[1]*1e7):
            print("Rally move failed")
            return
        self.rallyloader.reindex()
        print("Moved rally point from %s to %s at %fm" % (str(oldpos), str(latlon), rpoint.alt))


    def cmd_rally(self, args):
        '''rally point commands'''
        #TODO: add_land arg
        if len(args) < 1:
            self.print_usage()
            return

        elif args[0] == "add":
            self.cmd_rally_add(args[1:])

        elif args[0] == "move":
            self.cmd_rally_move(args[1:])

        elif args[0] == "clear":
            self.rallyloader.clear()
            self.mav_param.mavset(self.master,'RALLY_TOTAL',0,3)

        elif args[0] == "remove":
            if not self.have_list:
                print("Please list rally points first")
                return
            if (len(args) < 2):
                print("Usage: rally remove RALLYNUM")
                return
            self.rallyloader.remove(int(args[1]))
            self.send_rally_points()

        elif args[0] == "list":
            self.list_rally_points()
            self.have_list = True

        elif args[0] == "load":
            if (len(args) < 2):
                print("Usage: rally load filename")
                return

            try:
                self.rallyloader.load(args[1].strip('"'))
            except Exception as msg:
                print("Unable to load %s - %s" % (args[1], msg))
                return

            self.send_rally_points()
            self.have_list = True

            print("Loaded %u rally points from %s" % (self.rallyloader.rally_count(), args[1]))

        elif args[0] == "save":
            if (len(args) < 2):
                print("Usage: rally save filename")
                return

            self.rallyloader.save(args[1].strip('"'))

            print("Saved rally file %s" % args[1])

        elif args[0] == "alt":
            self.cmd_rally_alt(args[1:])

        elif args[0] == "land":
            if (len(args) >= 2 and args[1] == "abort"):
                self.abort_ack_received = False
                self.abort_first_send_time = 0

                self.abort_alt = self.settings.rally_breakalt
                if (len(args) >= 3):
                    self.abort_alt = int(args[2])

            else:
                self.master.mav.command_long_send(self.settings.target_system,
                        self.settings.target_component,
                        mavutil.mavlink.MAV_CMD_DO_RALLY_LAND,
                        0, 0, 0, 0, 0, 0, 0, 0)

        else:
            self.print_usage()

    def mavlink_packet(self, m):
        '''handle incoming mavlink packet'''
        type = m.get_type()
        if type in ['COMMAND_ACK']:
            if m.command == mavutil.mavlink.MAV_CMD_DO_GO_AROUND:
                if (m.result == 0 and self.abort_ack_received == False):
                    self.say("Landing Abort Command Successfully Sent.")
                    self.abort_ack_received = True
                elif (m.result != 0 and self.abort_ack_received == False):
                    self.say("Landing Abort Command Unsuccessful.")

            elif m.command == mavutil.mavlink.MAV_CMD_DO_RALLY_LAND:
                if (m.result == 0):
                    self.say("Landing.")

    def unload(self):
        self.remove_command("rally")
        if self.module('console') is not None and self.menu_added_console:
            self.menu_added_console = False
            self.module('console').remove_menu(self.menu)
        if self.module('map') is not None and self.menu_added_map:
            self.menu_added_map = False
            self.module('map').remove_menu(self.menu)
        super(OldRallyModule, self).unload()

    def send_rally_point(self, i):
        '''send rally points from fenceloader'''
        p = self.rallyloader.rally_point(i)
        p.target_system = self.target_system
        p.target_component = self.target_component
        self.master.mav.send(p)

    def send_rally_points(self):
        '''send rally points from rallyloader'''
        self.mav_param.mavset(self.master,'RALLY_TOTAL',self.rallyloader.rally_count(),3)

        for i in range(self.rallyloader.rally_count()):
            self.send_rally_point(i)

    def fetch_rally_point(self, i):
        '''fetch one rally point'''
        self.master.mav.rally_fetch_point_send(self.target_system,
                                                    self.target_component, i)
        tstart = time.time()
        p = None
        while time.time() - tstart < 1:
            p = self.master.recv_match(type='RALLY_POINT', blocking=False)
            if p is not None:
                break
            time.sleep(0.1)
            continue
        if p is None:
            self.console.error("Failed to fetch rally point %u" % i)
            return None
        return p

    def list_rally_points(self):
        self.rallyloader.clear()
        rally_count = self.mav_param.get('RALLY_TOTAL',0)
        if rally_count == 0:
            print("No rally points")
            return
        for i in range(int(rally_count)):
            p = self.fetch_rally_point(i)
            if p is None:
                return
            self.rallyloader.append_rally_point(p)

        for i in range(self.rallyloader.rally_count()):
            p = self.rallyloader.rally_point(i)
            self.console.writeln("lat=%f lng=%f alt=%f break_alt=%f land_dir=%f autoland=%f" % (p.lat * 1e-7, p.lng * 1e-7, p.alt, p.break_alt, p.land_dir, int(p.flags & 2!=0) ))

        if self.logdir is not None:
            fname = 'ral.txt'
            if self.target_system > 1:
                fname = 'ral_%u.txt' % self.target_system
            ral_file_path = os.path.join(self.logdir, fname)
            self.rallyloader.save(ral_file_path)
            print("Saved rally points to %s" % ral_file_path)

    def print_usage(self):
        print("Usage: rally <list|load|land|save|add|remove|move|clear|alt>")

def init(mpstate):
    '''initialise module'''
    return RallyModule(mpstate)

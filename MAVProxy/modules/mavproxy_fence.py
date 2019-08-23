"""
    MAVProxy geofence module
"""
import copy
import os, time, platform
import math

from pymavlink import mavextra
from pymavlink import mavwp, mavutil
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mission_item_protocol
from MAVProxy.modules.lib import rally_fence_shim_module

if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import *


def handle_sys_status(m, module):
    '''function to handle SYS_STATUS packets, used by both old and new module'''
    bits = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE

    present = ((m.onboard_control_sensors_present & bits) == bits)
    if module.present == False and present == True:
        module.say("fence present")
    elif module.present == True and present == False:
        module.say("fence removed")
    module.present = present

    enabled = ((m.onboard_control_sensors_enabled & bits) == bits)
    if module.enabled == False and enabled == True:
        module.say("fence enabled")
    elif module.enabled == True and enabled == False:
        module.say("fence disabled")
    module.enabled = enabled

    healthy = ((m.onboard_control_sensors_health & bits) == bits)
    if module.healthy == False and healthy == True:
        module.say("fence OK")
    elif module.healthy == True and healthy == False:
        module.say("fence breach")
    module.healthy = healthy

    #console output for fence:
    if not module.present:
        module.console.set_status('Fence', 'FEN', row=0, fg='black')
    elif module.enabled == False:
        module.console.set_status('Fence', 'FEN', row=0, fg='grey')
    elif module.enabled == True and module.healthy == True:
        module.console.set_status('Fence', 'FEN', row=0, fg='green')
    elif module.enabled == True and module.healthy == False:
        module.console.set_status('Fence', 'FEN', row=0, fg='red')

class FenceModule(rally_fence_shim_module.RallyFenceShim):
    '''FenceModule - a shim/wrapper class around the new module - which
    uses the MissionItemProtocol to upload fences - and the old module
    which uses the FENCE_POINT protocol to upload fence points.  It
    first uses the old FENCE_POINT protocol, but if it detects the
    vehicle can handle the new protocol (via
    AUTOPILOT_VERSION.capabilities) then it switches to the new
    module.
    '''

    def __init__(self, mpstate):
        '''initialise shim module - relying on base class for most of it'''
        super(FenceModule, self).__init__(mpstate, "fence", "fence point management (wrapper)", public = True)

    def oldmodule_class(self):
        '''if we shouldn't use the mission item protocol, use this legacy
        class instead'''
        return OldFenceModule

    def newmodule_class(self):
        '''if we should use the mission item protocol, use this class'''
        return NewFenceModule

    def capability_bit_required(self):
        '''returns bit found in AUTOPILOT_VERSION.capabilities that indicates
        uploading via mission item protocol is possible'''
        return mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE


class NewFenceModule(mission_item_protocol.MissionItemProtocolModule):
    '''uses common MISSION_ITEM protocol base class to provide fence
    upload/download
    '''

    def __init__(self, mpstate):
        '''initialise module; will raise AttributeError if pymavlink is too
        old to use'''
        mavwp.MissionItemProtocol_Fence # raise an attribute error if pymavlink is too old
        super(NewFenceModule, self).__init__(mpstate, "fencenew", "fence point management (new)", public = True)
        self.present = False
        self.enabled = False
        self.healthy = True

    def command_name(self):
        '''command-line command name'''
        return "fence"

    # def fence_point(self, i):
    #     return self.wploader.fence_point(i)

    def count(self):
        '''return number of waypoints'''
        return self.wploader.count()

    def circles_of_type(self, t):
        '''return a list of Circle fences of a specific type - a single
        MISSION_ITEM'''
        ret = []
        loader = self.wploader
        for i in range(0, loader.count()):
            p = loader.item(i)
            if p is None:
                print("Bad loader item (%u)" % i)
                return []
            if p.command != t:
                continue
            ret.append(p)
        return ret

    def inclusion_circles(self):
        '''return a list of Circle inclusion fences - a single MISSION_ITEM each'''
        return self.circles_of_type(mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION)

    def exclusion_circles(self):
        '''return a list of Circle exclusion fences - a single MISSION_ITEM each'''
        return self.circles_of_type(mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION)

    def polygons_of_type(self, t):
        '''return a list of polygon fences of a specific type - each a list of
        items'''
        ret = []
        loader = self.wploader
        state_outside = 99
        state_inside = 98
        current_polygon = []
        current_expected_length = 0
        state = state_outside
        for i in range(0, loader.count()):
            p = loader.item(i)
            if p is None:
                print("Bad loader item (%u)" % i)
                return []
            if p.command == t:
                # sanity checks:
                if p.param1 < 3:
                    print("Bad vertex count (%u) in seq=%u" % (p.param1, p.seq))
                    continue
                if state == state_outside:
                    # starting a new polygon
                    state = state_inside
                    current_expected_length = p.param1
                    current_polygon = []
                    current_polygon.append(p)
                    continue
                if state == state_inside:
                    # if the count is different and we're in state
                    # inside then the current polygon is invalid.
                    # Discard it.
                    if p.param1 != current_expected_length:
                        print("Short polygon found, discarding")
                        current_expected_length = p.param1
                        current_polygon = []
                        current_polygon.append(p)
                        continue
                    current_polygon.append(p)
                    if len(current_polygon) == current_expected_length:
                        ret.append(current_polygon)
                        state = state_outside
                    continue
                print("Unknown state (%s)" % str(state))
            else:
                if state == state_inside:
                    if len(current_polygon) != current_expected_length:
                        print("Short polygon found")
                    else:
                        ret.append(current_polygon)
                    state = state_outside
                    continue
                if state == state_outside:
                    continue
                print("Unknown state (%s)" % str(state))
        return ret

    def inclusion_polygons(self):
        '''return a list of polygon inclusion fences - each a list of items'''
        return self.polygons_of_type(mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION)

    def exclusion_polygons(self):
        '''return a list of polygon exclusion fences - each a list of items'''
        return self.polygons_of_type(mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION)

    @staticmethod
    def loader_class():
        return mavwp.MissionItemProtocol_Fence

    def mav_mission_type(self):
        return mavutil.mavlink.MAV_MISSION_TYPE_FENCE

    def itemstype(self):
        '''returns description of items in the plural'''
        return 'fence items'

    def itemtype(self):
        '''returns description of item'''
        return 'fence item'

    def mavlink_packet(self, m):
        if m.get_type() == 'SYS_STATUS':
            handle_sys_status(m, self)
        super(NewFenceModule, self).mavlink_packet(m)

    def fence_draw_callback(self, points):
        '''callback from drawing a fence'''
        if len(points) < 3:
            print("Fence draw cancelled")
            return
        items = []
        for p in points:
            m = mavutil.mavlink.MAVLink_mission_item_int_message(
                self.target_system,
                self.target_component,
                0,    # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL,    # frame
                self.drawing_fence_type,    # command
                0,    # current
                0,    # autocontinue
                len(points), # param1,
                0.0,  # param2,
                0.0,  # param3
                0.0,  # param4
                int(p[0]*1e7),  # x (latitude)
                int(p[1]*1e7),  # y (longitude)
                0,                     # z (altitude)
                self.mav_mission_type(),
            )
            items.append(m)

        self.append(items)
        self.send_all_items()
        self.wploader.last_change = time.time()
        print("Reset last changed time to %f" % self.wploader.last_change)

    def cmd_draw(self, args):
        '''convenience / compatability / slow learner command to work like the
        old module - i.e. a single inclusion polyfence'''
        # TODO: emit this only if there actually are complex fences:
        if not self.check_have_list():
            return

        if len(args) == 0:
            print("WARNING!  You want 'fence draw inc' or 'fence draw exc'")
            return
        if args[0] in ("inc", "inclusion"):
            self.drawing_fence_type = mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
            draw_colour = (128,128,255)
        elif args[0] in ("exc", "exclusion"):
            self.drawing_fence_type = mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
            draw_colour = (255,128,128)
        else:
            print("fence draw <inc|inclusion|exc|exclusion>")
            return

        if not 'draw_lines' in self.mpstate.map_functions:
            print("No map drawing available")
            return

        self.mpstate.map_functions['draw_lines'](self.fence_draw_callback,
                                                 colour=draw_colour)
        print("Drawing fence on map")

    def cmd_addcircle(self, args):
        '''adds a circle to the map click position of specific type/radius'''
        if not self.check_have_list():
            return
        if len(args) < 2:
            print("Need 2 arguments")
            return
        t = args[0]
        radius = float(args[1])

        latlon = self.mpstate.click_location
        if latlon is None:
            print("No click position available")
            return

        if t in ["inclusion", "inc"]:
            command = mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
        elif t in ["exclusion", "exc"]:
            command = mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION
        else:
            print("%s is not one of inclusion|exclusion" % t)
            return

        m = mavutil.mavlink.MAVLink_mission_item_int_message(
            self.target_system,
            self.target_component,
            0,    # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL,    # frame
            command,    # command
            0,    # current
            0,    # autocontinue
            radius, # param1,
            0.0,  # param2,
            0.0,  # param3
            0.0,  # param4
            int(latlon[0] * 1e7),  # x (latitude)
            int(latlon[1] * 1e7),  # y (longitude)
            0,                     # z (altitude)
            self.mav_mission_type(),
        )
        self.append(m)
        self.send_all_items()

    def cmd_addpoly(self, args):
        '''adds a number of waypoints equally spaced around a circle around click 
        point'''
        if not self.check_have_list():
            return
        if len(args) < 1:
            print("Need at least 1 argument")
            return
        t = args[0]
        count = 4
        radius = 20
        rotation = 0
        if len(args) > 1:
            radius = float(args[1])
        if len(args) > 2:
            count = int(args[2])
        if len(args) > 3:
            rotation = float(args[3])

        if count < 3:
            print("Invalid count (%s)" % str(count))
            return
        if radius <= 0:
            print("Invalid radius (%s)" % str(radius))
            return

        latlon = self.mpstate.click_location
        if latlon is None:
            print("No map click position available")
            return

        if t in ["inclusion", "inc"]:
            command = mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
        elif t in ["exclusion", "exc"]:
            command = mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
        else:
            print("%s is not one of inclusion|exclusion" % t)
            return

        items = []
        for i in range(0, count):
            (lat, lon) = mavextra.gps_newpos(latlon[0],
                                             latlon[1],
                                             360/float(count)*i + rotation,
                                             radius)

            m = mavutil.mavlink.MAVLink_mission_item_int_message(
                self.target_system,
                self.target_component,
                0,    # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL,    # frame
                command,    # command
                0,    # current
                0,    # autocontinue
                count, # param1,
                0.0,  # param2,
                0.0,  # param3
                0.0,  # param4
                int(lat*1e7),  # x (latitude)
                int(lon*1e7),  # y (longitude)
                0,                     # z (altitude)
                self.mav_mission_type(),
            )
            items.append(m)

        for m in items:
            self.append(m)
        self.send_all_items()

    def remove(self, wp):
        '''deny remove on fence - requires renumbering etc etc'''
        print("remove is not currently supported for fence")
        if not self.check_have_list():
            return

    def cmd_remove(self, args):
        '''deny remove on fence - requires renumbering etc etc'''
        print("remove is not currently supported for fence.  Try removepolygon_point or removecircle")
        if not self.check_have_list():
            return

    def removecircle(self, seq):
        '''remove circle at offset seq'''
        if not self.check_have_list():
            return
        item = self.wploader.item(seq)
        if item is None:
            print("No item %s" % str(seq))
            return

        if (item.command != mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION and
            item.command != mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION):
            print("Item %u is not a circle" % seq)
            return
        self.wploader.remove(item)
        self.send_all_items()

    def removepolygon_point(self, polygon_start_seq, item_offset):
        '''removes item at offset item_offset from the polygon starting at
        polygon_start_seq'''
        if not self.check_have_list():
            return

        items_to_set = []

        first_item = self.wploader.item(polygon_start_seq)
        if (first_item.command != mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION and
            first_item.command != mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION):
            print("Item %u is not a polygon vertex" % seq)
            return
        original_count = int(first_item.param1)
        if item_offset >= original_count:
            print("Out-of-range point")
            return
        if original_count <= 3:
            print("Too few points to remove one")
            return

        dead_item_walking = self.wploader.item(polygon_start_seq + item_offset)

        # must reduce count in each of the polygons:
        for i in range(int(first_item.param1)):
            item = self.wploader.item(polygon_start_seq+i)
            if int(item.param1) != original_count:
                print("Invalid polygon starting at %u (count=%u), point %u (count=%u)" %
                      (polygon_start_seq, original_count, i, int(item.param1)))
                return
            item.param1 = item.param1 - 1
            items_to_set.append(item)

        for item in items_to_set:
            w = item
            self.wploader.set(w, w.seq)

        self.wploader.remove(dead_item_walking)
        self.send_all_items()

    def addpolygon_point(self, polygon_start_seq, item_offset):
        '''adds item at offset item_offset into the polygon starting at
        polygon_start_seq'''

        if not self.check_have_list():
            return

        items_to_set = []

        first_item = self.wploader.item(polygon_start_seq)
        if (first_item.command != mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION and
            first_item.command != mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION):
            print("Item %u is not a polygon vertex" % seq)
            return
        original_count = int(first_item.param1)
        if item_offset >= original_count:
            print("Out-of-range point")
            return

        # increase count in each of the polygon vertexes:
        for i in range(int(first_item.param1)):
            item = self.wploader.item(polygon_start_seq+i)
            if int(item.param1) != original_count:
                print("Invalid polygon starting at %u (count=%u), point %u (count=%u)" %
                      (polygon_start_seq, original_count, i, int(item.param1)))
                return
            item.param1 = item.param1 + 1
            items_to_set.append(item)

        for item in items_to_set:
            w = item
            self.wploader.set(w, w.seq)

        old_item = self.wploader.item(polygon_start_seq + item_offset)
        new_item = copy.copy(old_item)
        # reset latitude and longitude of new item to be half-way
        # between it and the preceeding point
        if item_offset == 0:
            prev_item_offset = original_count-1
        else:
            prev_item_offset = item_offset - 1
        prev_item = self.wploader.item(polygon_start_seq + prev_item_offset)
        new_item.x = (old_item.x + prev_item.x)/2
        new_item.y = (old_item.y + prev_item.y)/2
        self.wploader.insert(polygon_start_seq + item_offset, new_item)
        print("Sending all items")
        self.send_all_items()

    def removepolygon(self, seq):
        '''remove polygon at offset seq'''
        if not self.check_have_list():
            return
        first_item = self.wploader.item(seq)
        if (first_item.command != mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION and
            first_item.command != mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION):
            print("Item %u is not a polygon vertex" % seq)
            return

        items_to_remove = []
        for i in range(int(first_item.param1)):
            item = self.wploader.item(seq+i)
            if item is None:
                print("No item %s" % str(i))
                return
            if item.param1 != first_item.param1:
                print("Invalid polygon starting at %u (count=%u), point %u (count=%u)" %
                      (seq, int(first_item.param1), i, int(item.param1)))
                return
            if (item.command != mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION and
                item.command != mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION):
                print("Item %u point %u is not a polygon vertex" % (seq, i))
                return
            items_to_remove.append(item)

        self.remove(items_to_remove)
        self.send_all_items()

    def cmd_movepolypoint(self, args):
        '''moves item at offset item_offset in polygon starting at
        polygon_start_seqence to map click point'''
        if not self.check_have_list():
            return

        if len(args) < 2:
            print("Need first polygon point and vertex offset")
            return
        polygon_start_seq = int(args[0])
        item_offset = int(args[1])
        first_item = self.wploader.item(polygon_start_seq)
        if first_item is None:
            print("No item at %u" % polygon_start_seq)
            return
        if (first_item.command != mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION and
            first_item.command != mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION):
            print("Item %u is not a polygon vertex" % seq)
            return
        original_count = int(first_item.param1)
        if item_offset >= original_count:
            print("Out-of-range point")
            return

        latlon = self.mpstate.click_location
        if latlon is None:
            print("No map click position available")
            return

        moving_item = self.wploader.item(polygon_start_seq + item_offset)
        moving_item.x = latlon[0]
        moving_item.y = latlon[1]
        if moving_item.get_type() == "MISSION_ITEM_INT":
            moving_item.x *= 1e7
            moving_item.y *= 1e7

        print("moving item to (%f %f)" % (moving_item.x, moving_item.y))

        self.wploader.set(moving_item, moving_item.seq)
        self.send_single_waypoint(moving_item.seq)

    def set_fence_enabled(self, do_enable):
        '''Enable or disable fence'''
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE,
            0,
            do_enable,
            0,
            0,
            0,
            0,
            0,
            0)

    def cmd_enable(self, args):
        '''enable fence'''
        self.set_fence_enabled(1)

    def cmd_disable(self, args):
        '''disable fence'''
        self.set_fence_enabled(0)

    def commands(self):
        '''returns map from command name to handling function'''
        ret = super(NewFenceModule, self).commands()
        ret.update({
            'addcircle' : (self.cmd_addcircle, ["<inclusion|inc|exclusion|exc>", "RADIUS"]),
            'addpoly' : (self.cmd_addpoly, ["<inclusion|inc|exclusion|exc>", "<radius>" "<pointcount>", "<rotation>"]),
            'movepolypoint' : (self.cmd_movepolypoint, ["POLY_FIRSTPOINT", "POINT_OFFSET"]),
            'enable' : self.cmd_enable,
            'disable' : self.cmd_disable,
            'draw' : self.cmd_draw,
        })
        return ret

class OldFenceModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(OldFenceModule, self).__init__(mpstate, "fenceold", "geo-fence management", public = True)
        self.fenceloader_by_sysid = {}
        self.last_fence_breach = 0
        self.last_fence_status = 0
        self.present = False
        self.enabled = False
        self.healthy = True
        self.add_command('fence', self.cmd_fence,
                         "geo-fence management",
                         ["<draw|list|clear|enable|disable|move|remove>",
                          "<load|save> (FILENAME)"])

        self.have_list = False

        if self.continue_mode and self.logdir is not None:
            fencetxt = os.path.join(self.logdir, 'fence.txt')
            if os.path.exists(fencetxt):
                self.fenceloader.load(fencetxt)
                self.have_list = True
                print("Loaded fence from %s" % fencetxt)

        self.menu_added_console = False
        self.menu_added_map = False
        if mp_util.has_wxpython:
            self.menu = MPMenuSubMenu('Fence',
                                  items=[MPMenuItem('Clear', 'Clear', '# fence clear'),
                                         MPMenuItem('List', 'List', '# fence list'),
                                         MPMenuItem('Load', 'Load', '# fence load ',
                                                    handler=MPMenuCallFileDialog(flags=('open',),
                                                                                 title='Fence Load',
                                                                                 wildcard='*.fen')),
                                         MPMenuItem('Save', 'Save', '# fence save ',
                                                    handler=MPMenuCallFileDialog(flags=('save', 'overwrite_prompt'),
                                                                                 title='Fence Save',
                                                                                 wildcard='*.fen')),
                                         MPMenuItem('Draw', 'Draw', '# fence draw')])

    def last_change(self):
        '''returns time the fence last changed'''
        return self.fenceloader.last_change

    @property
    def fenceloader(self):
        '''fence loader by sysid'''
        if not self.target_system in self.fenceloader_by_sysid:
            self.fenceloader_by_sysid[self.target_system] = mavwp.MAVFenceLoader()
        return self.fenceloader_by_sysid[self.target_system]

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

    def mavlink_packet(self, m):
        '''handle and incoming mavlink packet'''
        if m.get_type() == "FENCE_STATUS":
            self.last_fence_breach = m.breach_time
            self.last_fence_status = m.breach_status
        elif m.get_type() in ['SYS_STATUS']:
            handle_sys_status(m, self)

    def set_fence_enabled(self, do_enable):
        '''Enable or disable fence'''
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE, 0,
            do_enable, 0, 0, 0, 0, 0, 0)

    def cmd_fence_move(self, args):
        '''handle fencepoint move'''
        if len(args) < 1:
            print("Usage: fence move FENCEPOINTNUM")
            return
        if not self.have_list:
            print("Please list fence points first")
            return

        idx = int(args[0])
        if idx <= 0 or idx > self.fenceloader.count():
            print("Invalid fence point number %u" % idx)
            return

        latlon = self.mpstate.click_location
        if latlon is None:
            print("No map click position available")
            return

        # note we don't subtract 1, as first fence point is the return point
        self.fenceloader.move(idx, latlon[0], latlon[1])
        if self.send_fence():
            print("Moved fence point %u" % idx)

    def cmd_fence_remove(self, args):
        '''handle fencepoint remove'''
        if len(args) < 1:
            print("Usage: fence remove FENCEPOINTNUM")
            return
        if not self.have_list:
            print("Please list fence points first")
            return

        idx = int(args[0])
        if idx <= 0 or idx > self.fenceloader.count():
            print("Invalid fence point number %u" % idx)
            return

        # note we don't subtract 1, as first fence point is the return point
        self.fenceloader.remove(idx)
        if self.send_fence():
            print("Removed fence point %u" % idx)
        else:
            print("Failed to remove fence point %u" % idx)

    def cmd_fence(self, args):
        '''fence commands'''
        if len(args) < 1:
            self.print_usage()
            return

        if args[0] == "enable":
            self.set_fence_enabled(1)
        elif args[0] == "disable":
            self.set_fence_enabled(0)
        elif args[0] == "load":
            if len(args) != 2:
                print("usage: fence load <filename>")
                return
            self.load_fence(args[1])
        elif args[0] == "list":
            self.list_fence(None)
        elif args[0] == "move":
            self.cmd_fence_move(args[1:])
        elif args[0] == "remove":
            self.cmd_fence_remove(args[1:])
        elif args[0] == "save":
            if len(args) != 2:
                print("usage: fence save <filename>")
                return
            self.list_fence(args[1])
        elif args[0] == "show":
            if len(args) != 2:
                print("usage: fence show <filename>")
                return
            self.fenceloader.load(args[1])
            self.have_list = True
        elif args[0] == "draw":
            if not 'draw_lines' in self.mpstate.map_functions:
                print("No map drawing available")
                return
            self.mpstate.map_functions['draw_lines'](self.fence_draw_callback)
            print("Drawing fence on map")
        elif args[0] == "clear":
            self.param_set('FENCE_TOTAL', 0, 3)
        else:
            self.print_usage()

    def load_fence(self, filename):
        '''load fence points from a file'''
        try:
            self.fenceloader.target_system = self.target_system
            self.fenceloader.target_component = self.target_component
            self.fenceloader.load(filename.strip('"'))
        except Exception as msg:
            print("Unable to load %s - %s" % (filename, msg))
            return
        print("Loaded %u geo-fence points from %s" % (self.fenceloader.count(), filename))
        self.send_fence()

    def send_fence(self):
        '''send fence points from fenceloader'''
        # must disable geo-fencing when loading
        self.fenceloader.target_system = self.target_system
        self.fenceloader.target_component = self.target_component
        self.fenceloader.reindex()
        action = self.get_mav_param('FENCE_ACTION', mavutil.mavlink.FENCE_ACTION_NONE)
        self.param_set('FENCE_ACTION', mavutil.mavlink.FENCE_ACTION_NONE, 3)
        self.param_set('FENCE_TOTAL', self.fenceloader.count(), 3)
        for i in range(self.fenceloader.count()):
            p = self.fenceloader.point(i)
            self.master.mav.send(p)
            p2 = self.fetch_fence_point(i)
            if p2 is None:
                self.param_set('FENCE_ACTION', action, 3)
                return False
            if (p.idx != p2.idx or
                abs(p.lat - p2.lat) >= 0.00003 or
                abs(p.lng - p2.lng) >= 0.00003):
                print("Failed to send fence point %u" % i)
                self.param_set('FENCE_ACTION', action, 3)
                return False
        self.param_set('FENCE_ACTION', action, 3)
        return True

    def fetch_fence_point(self ,i):
        '''fetch one fence point'''
        self.master.mav.fence_fetch_point_send(self.target_system,
                                                    self.target_component, i)
        tstart = time.time()
        p = None
        while time.time() - tstart < 3:
            p = self.master.recv_match(type='FENCE_POINT', blocking=False)
            if p is not None:
                break
            time.sleep(0.1)
            continue
        if p is None:
            self.console.error("Failed to fetch point %u" % i)
            return None
        return p

    def fence_draw_callback(self, points):
        '''callback from drawing a fence'''
        self.fenceloader.clear()
        if len(points) < 3:
            return
        self.fenceloader.target_system = self.target_system
        self.fenceloader.target_component = self.target_component
        bounds = mp_util.polygon_bounds(points)
        (lat, lon, width, height) = bounds
        center = (lat+width/2, lon+height/2)
        self.fenceloader.add_latlon(center[0], center[1])
        for p in points:
            self.fenceloader.add_latlon(p[0], p[1])
        # close it
        self.fenceloader.add_latlon(points[0][0], points[0][1])
        self.send_fence()
        self.have_list = True

    def list_fence(self, filename):
        '''list fence points, optionally saving to a file'''
        self.fenceloader.clear()
        count = self.get_mav_param('FENCE_TOTAL', 0)
        if count == 0:
            print("No geo-fence points")
            return
        for i in range(int(count)):
            p = self.fetch_fence_point(i)
            if p is None:
                return
            self.fenceloader.add(p)

        if filename is not None:
            try:
                self.fenceloader.save(filename.strip('"'))
            except Exception as msg:
                print("Unable to save %s - %s" % (filename, msg))
                return
            print("Saved %u geo-fence points to %s" % (self.fenceloader.count(), filename))
        else:
            for i in range(self.fenceloader.count()):
                p = self.fenceloader.point(i)
                self.console.writeln("lat=%f lng=%f" % (p.lat, p.lng))
        if self.status.logdir is not None:
            fname = 'fence.txt'
            if self.target_system > 1:
                fname = 'fence_%u.txt' % self.target_system
            fencetxt = os.path.join(self.status.logdir, fname)
            self.fenceloader.save(fencetxt.strip('"'))
            print("Saved fence to %s" % fencetxt)
        self.have_list = True

    def print_usage(self):
        print("usage: fence <enable|disable|list|load|save|clear|draw|move|remove>")

    def unload(self):
        self.remove_command("fence")
        if self.module('console') is not None and self.menu_added_console:
            self.menu_added_console = False
            self.module('console').remove_menu(self.menu)
        if self.module('map') is not None and self.menu_added_map:
            self.menu_added_map = False
            self.module('map').remove_menu(self.menu)
        super(OldFenceModule, self).unload()

def init(mpstate):
    '''initialise module'''
    return FenceModule(mpstate)

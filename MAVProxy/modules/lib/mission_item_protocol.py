#!/usr/bin/env python

'''
base class for modules generally transfering items using the MISSION_ITEM protocol
'''

import time, os, fnmatch, copy, platform
import re

from pymavlink import mavutil, mavwp
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.mavproxy_map import mp_elevation

if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import *

class MissionItemProtocolModule(mp_module.MPModule):
    def __init__(self, mpstate, name, description, **args):
        super(MissionItemProtocolModule, self).__init__(mpstate, name, description, **args)
        self.add_command(self.command_name(),
                         self.cmd_wp,
                         '%s management' % self.itemtype(),
                         self.completions()
        )

        self.wp_op = None
        self.wp_requested = {}
        self.wp_received = {}
        self.wp_save_filename = None
        self.wploader_by_sysid = {}
        self.loading_waypoints = False
        self.loading_waypoint_lasttime = time.time()
        self.last_waypoint = 0
        self.wp_period = mavutil.periodic_event(0.5)
        self.undo_wp = None
        self.undo_type = None
        self.undo_wp_idx = -1
        self.wploader.expected_count = 0

        if self.continue_mode and self.logdir is not None:
            waytxt = os.path.join(mpstate.status.logdir, self.save_filename())
            if os.path.exists(waytxt):
                self.wploader.load(waytxt)
                print("Loaded %s from %s" % (self.itemstype(), waytxt))

        self.elevation_map = mp_elevation.ElevationModel()

        self.init_gui_menus()


    def gui_menu_items(self):
        return [
            MPMenuItem('Clear', 'Clear', '# %s clear' % self.command_name()),
            MPMenuItem('List', 'List', '# %s list' % self.command_name()),
            MPMenuItem('Load', 'Load', '# %s load ' % self.command_name(),
                       handler=MPMenuCallFileDialog(
                           flags=('open',),
                           title='%s Load' % self.mission_type_string(),
                           wildcard='*.txt')),
            MPMenuItem('Save', 'Save', '# %s save ' % self.command_name(),
                       handler=MPMenuCallFileDialog(
                           flags=('save', 'overwrite_prompt'),
                           title='%s Save' % self.mission_type_string(),
                           wildcard='*.txt')),
            MPMenuItem('Undo', 'Undo', '# %s undo' %  self.command_name()),
        ]

    def mission_type_string(self):
        loader_class_string = str(self.loader_class())
        m = re.search("'([^']*)'", loader_class_string)
        if m is None:
            raise ValueError("Failed to match %s" % loader_class_string)
        cname = m.group(1)
        items = cname.split("_")
        return items[-1]

    def init_gui_menus(self):
        '''initialise menus for console and map'''
        self.menu_added_console = False
        self.menu_added_map = False
        if not mp_util.has_wxpython:
            return

        self.menu = MPMenuSubMenu(self.mission_type_string(),
                                  items=self.gui_menu_items())

    def completions(self):
        '''form up MAVProxy-style completion strings used for tab completion'''
        cs = self.commands()
        no_arguments = []
        command_argument_buckets = {}
        for c in cs:
            value = cs[c]
            if type(value) == tuple:
                (function, arguments) = value
                args_string = " ".join(arguments)
                if args_string not in command_argument_buckets:
                    command_argument_buckets[args_string] = []
                command_argument_buckets[args_string].append(c)
            else:
                no_arguments.append(c)

        ret = []
        if len(no_arguments):
            ret.append("<" + "|".join(sorted(no_arguments)) + ">")
        for k in command_argument_buckets:
            ret.append("<" + "|".join(sorted(command_argument_buckets[k])) + "> " + k)
        return ret

    def unload(self):
        self.remove_command(self.command_name())
        if self.module('console') is not None and self.menu_added_console:
            self.menu_added_console = False
            self.module('console').remove_menu(self.menu)
        if self.module('map') is not None and self.menu_added_map:
            self.menu_added_map = False
            self.module('map').remove_menu(self.menu)
        super(MissionItemProtocolModule, self).unload()

    def create_loader(self):
        c = self.loader_class()
        return c()

    def last_change(self):
        return self.wploader.last_change

    def check_have_list(self):
        if self.last_change() == 0:
            print("Please list %s items first" % self.command_name())
            return False
        return True

    def index_from_0(self):
        '''e.g. rally points etc are indexed from 1 from the user interface
        perspective'''
        return False

    def save_filename_base(self):
        return self.itemstype().replace(" ", "-")

    def save_filename(self):
        return self.save_filename_base() + ".txt"

    @property
    def wploader(self):
        '''per-sysid wploader'''
        if self.target_system not in self.wploader_by_sysid:
            self.wploader_by_sysid[self.target_system] = self.create_loader()
        return self.wploader_by_sysid[self.target_system]

    def missing_wps_to_request(self):
        ret = []
        tnow = time.time()
        next_seq = self.wploader.count()
        for i in range(5):
            seq = next_seq+i
            if seq+1 > self.wploader.expected_count:
                continue
            if seq in self.wp_requested and tnow - self.wp_requested[seq] < 2:
                continue
            ret.append(seq)
        return ret

    def send_wp_requests(self, wps=None):
        '''send some more WP requests'''
        if wps is None:
            wps = self.missing_wps_to_request()
        tnow = time.time()
        for seq in wps:
            # print("REQUESTING %u/%u (%u)" % (seq,
            # self.wploader.expected_count, i))
            self.wp_requested[seq] = tnow
            if self.settings.wp_use_mission_int:
                self.master.mav.mission_request_send(
                    self.target_system,
                    self.target_component,
                    seq,
                    mission_type=self.mav_mission_type())
            else:
                self.master.waypoint_request_send(seq)

    def cmd_status(self, args):
        '''show status of wp download'''
        try:
            print("Have %u of %u %s" %
                  (self.wploader.count()+len(self.wp_received),
                   self.wploader.expected_count,
                   self.itemstype()))
        except Exception:
            print("Have %u %s" %
                  (self.wploader.count()+len(self.wp_received), self.itemstype()))

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        mtype = m.get_type()
        if mtype in ('MISSION_COUNT', 'MISSION_REQUEST', 'MISSION_ITEM'):
            t = getattr(m,
                        'mission_type',
                        mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            if self.mav_mission_type() != t:
                return
        if mtype in ['WAYPOINT_COUNT', 'MISSION_COUNT']:
            self.wploader.expected_count = m.count
            if self.wp_op is None:
                self.console.error("No %s load started" % self.itemtype())
                pass
            else:
                self.wploader.clear()
                self.console.writeln("Requesting %u %s t=%s now=%s" % (
                    m.count,
                    self.itemstype(),
                    time.asctime(time.localtime(m._timestamp)),
                    time.asctime()))
                self.send_wp_requests()

        elif mtype in ['WAYPOINT', 'MISSION_ITEM'] and self.wp_op is not None:
            if m.seq < self.wploader.count():
                print("DUPLICATE %u" % m.seq)
                return
            if m.seq+1 > self.wploader.expected_count:
                self.console.writeln("Unexpected %s number %u - want %u" % (
                    self.itemtype(),
                    m.seq,
                    self.wploader.count()))
            self.wp_received[m.seq] = m
            next_seq = self.wploader.count()
            while next_seq in self.wp_received:
                m = self.wp_received.pop(next_seq)
                self.wploader.add(m)
                next_seq += 1
            if self.wploader.count() != self.wploader.expected_count:
                # print("m.seq=%u expected_count=%u" %
                # (m.seq, self.wploader.expected_count))
                self.send_wp_requests()
                return
            if self.wp_op == 'list':
                for i in range(self.wploader.count()):
                    w = self.wploader.wp(i)
                    print(("%u %u %.10f %.10f %f p1=%.1f p2=%.1f p3=%.1f "
                          "p4=%.1f cur=%u auto=%u") % (
                        w.command, w.frame, w.x, w.y, w.z,
                        w.param1, w.param2, w.param3, w.param4,
                        w.current, w.autocontinue))
                if self.logdir is not None:
                    fname = self.save_filename()
                    if m.get_srcSystem() != 1:
                        fname = '%s_%u.txt' % (self.save_filename_base(),
                                               m.get_srcSystem(),)
                    waytxt = os.path.join(self.logdir, fname)
                    self.save_waypoints(waytxt)
                    print("Saved %s to %s" % (self.itemstype(), waytxt))
                self.loading_waypoints = False
            elif self.wp_op == "save":
                self.save_waypoints(self.wp_save_filename)
            self.wp_op = None
            self.wp_requested = {}
            self.wp_received = {}

        elif mtype in ["WAYPOINT_REQUEST", "MISSION_REQUEST"]:
            self.process_waypoint_request(m, self.master)

    def idle_request_waypoints(self):
        if not self.wp_period.trigger():
            return
        # cope with packet loss fetching mission
        if self.master is None:
            return
        if self.master.time_since('MISSION_ITEM') < 2:
            return
        expected = getattr(self.wploader, 'expected_count', None)
        if expected is None:
            return
        if self.wploader.count() >= expected:
            return

        wps = self.missing_wps_to_request()
        print("re-requesting %s %s" % (self.itemstype(), str(wps)))
        self.send_wp_requests(wps)

    def idle_check_gui_menus(self):
        # probe for presence of GUI widgets and add our stuff in:
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

    def idle_task(self):
        '''handle missing waypoints'''
        self.idle_request_waypoints()
        self.idle_check_gui_menus()

    def process_waypoint_request(self, m, master):
        '''process a waypoint request from the master'''
        if m.target_system != self.settings.source_system:
            # self.console.error("Mission request is not for me")
            return
        if m.target_component != self.settings.source_component:
            # self.console.error("Mission request is not for me")
            return
        if not self.loading_waypoints:
            return
        if time.time() > self.loading_waypoint_lasttime + 10.0:
            self.loading_waypoints = False
            # self.console.error("not loading waypoints")
            return
        if m.seq >= self.wploader.count():
            self.console.error("Request for bad waypoint %u (max %u)" %
                               (m.seq, self.wploader.count()))
            return
        wp = self.wploader.wp(m.seq)
        wp.target_system = self.target_system
        wp.target_component = self.target_component
        self.master.mav.send(self.wploader.wp(m.seq))
        self.loading_waypoint_lasttime = time.time()
        self.console.writeln("Sent %s %u : %s" %
                             (self.itemtype(), m.seq, self.wploader.wp(m.seq)))
        if m.seq == self.wploader.count() - 1:
            self.loading_waypoints = False
            self.console.writeln("Sent all %u %s" % (self.wploader.count(),
                                                     self.itemstype()))

    def send_all_waypoints(self):
        return self.send_all_items()

    def send_all_items(self):
        '''send all waypoints to vehicle'''
        if self.master.mavlink20():
            if (self.mav_mission_type() == mavutil.mavlink.MAV_MISSION_TYPE_MISSION or self.wploader.count() == 0):
                # clear mission for backwards compatability.  Rally
                # and fence should cope (and be atomic)
                self.master.mav.mission_clear_all_send(
                    target_system=self.target_system,
                    target_component=self.target_component,
                    mission_type=self.mav_mission_type()
                )
            pass
        else:
            self.master.waypoint_clear_all_send()
        if self.wploader.count() == 0:
            return
        self.loading_waypoints = True
        self.loading_waypoint_lasttime = time.time()
        if self.master.mavlink20():
            self.master.mav.mission_count_send(
                self.target_system,
                self.target_component,
                self.wploader.count(),
                mission_type=self.mav_mission_type())
        else:
            self.master.waypoint_count_send(self.wploader.count())

    def load_waypoints(self, filename):
        '''load waypoints from a file'''
        self.wploader.target_system = self.target_system
        self.wploader.target_component = self.target_component
        try:
            # need to remove the leading and trailing quotes in filename
            self.wploader.load(filename.strip('"'))
        except Exception as msg:
            print("Unable to load %s - %s" % (filename, msg))
            return
        print("Loaded %u %s from %s" % (self.wploader.count(),
                                        self.itemstype(),
                                        filename))
        self.send_all_waypoints()

    def update_waypoints(self, filename, wpnum):
        '''update waypoints from a file'''
        self.wploader.target_system = self.target_system
        self.wploader.target_component = self.target_component
        try:
            self.wploader.load(filename)
        except Exception as msg:
            print("Unable to load %s - %s" % (filename, msg))
            return
        if self.wploader.count() == 0:
            print("No %s found in %s" % (self.itemstype(), filename))
            return
        if wpnum == -1:
            print("Loaded %u updated %s from %s" % (self.wploader.count(),
                                                    self.itemstype(),
                                                    filename))
        elif wpnum >= self.wploader.count():
            print("Invalid %s number %u" % (self.itemtype(), wpnum))
            return
        else:
            print("Loaded updated %s %u from %s" % (self.itemtype(),
                                                    wpnum,
                                                    filename))

        self.loading_waypoints = True
        self.loading_waypoint_lasttime = time.time()
        if wpnum == -1:
            start = 0
            end = self.wploader.count()-1
        else:
            start = self.item_num_to_offset(wpnum)
            end = self.item_num_to_offset(wpnum)

        if self.master.mavlink20():
            self.master.mav.mission_write_partial_list_send(
                self.target_system,
                self.target_component,
                start, end,
                mission_type=self.mav_mission_type())
        else:
            self.master.mav.mission_write_partial_list_send(
                self.target_system,
                self.target_component,
                start, end)

    def save_waypoints(self, filename):
        '''save waypoints to a file'''
        try:
            # need to remove the leading and trailing quotes in filename
            self.wploader.save(filename.strip('"'))
        except Exception as msg:
            print("Failed to save %s - %s" % (filename, msg))
            return
        print("Saved %u %s to %s" % (self.wploader.count(),
                                     self.itemstype(),
                                     filename))

    def save_waypoints_csv(self, filename):
        '''save waypoints to a file in a human readable CSV file'''
        try:
            # need to remove the leading and trailing quotes in filename
            self.wploader.savecsv(filename.strip('"'))
        except Exception as msg:
            print("Failed to save %s - %s" % (filename, msg))
            return
        print("Saved %u %s to CSV %s" % (self.wploader.count(),
                                         self.itemstype(),
                                         filename))

    def good_item_num_to_manipulate(self, idx):
        if idx > self.wploader.count():
            return False
        if idx < 1:
            return False
        return True

    def item_num_to_offset(self, item_num):
        if self.index_from_0():
            return item_num
        return item_num - 1

    def cmd_clear(self, args):
        if self.master.mavlink20():
            self.master.mav.mission_clear_all_send(
                self.target_system,
                self.target_component,
                mission_type=self.mav_mission_type())
        else:
            self.master.waypoint_clear_all_send()
        self.wploader.clear()
        if getattr(self.wploader, 'expected_count', None) is not None:
            self.wploader.expected_count = 0

    def cmd_move(self, args):
        '''handle wp move'''
        if not self.check_have_list():
            return
        if len(args) != 1:
            print("usage: %s move WPNUM" % self.command_name())
            return
        idx = int(args[0])
        if not self.good_item_num_to_manipulate(idx):
            print("Invalid %s number %u" % (self.itemtype(), idx))
            return
        offset = self.item_num_to_offset(idx)
        latlon = self.mpstate.click_location
        if latlon is None:
            print("No click position available")
            return
        wp = self.wploader.wp(offset)
        if wp is None:
            print("Invalid rally point")
            return

        # setup for undo
        self.undo_wp = copy.copy(wp)
        self.undo_wp_idx = idx
        self.undo_type = "move"

        (lat, lon) = latlon
        if wp.frame == mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT:
            alt1 = self.elevation_map.GetElevation(lat, lon)
            alt2 = self.elevation_map.GetElevation(wp.x, wp.y)
            if alt1 is not None and alt2 is not None:
                wp.z += alt1 - alt2
        wp.x = lat
        wp.y = lon
        if wp.get_type() == "MISSION_ITEM_INT":
            wp.x = int(wp.x * 1e7)
            wp.y = int(wp.y * 1e7)

        self.wploader.set(wp, offset)
        self.send_single_waypoint(offset)
        print("Moved %s %u to %f, %f at %.1fm" %
              (self.itemtype(),idx, lat, lon, wp.z))

    def send_single_waypoint(self, offset):
        '''send a single waypoint to the autopilot'''
        wp = self.wploader.item(offset)
        wp.target_system = self.target_system
        wp.target_component = self.target_component
        self.loading_waypoints = True
        self.loading_waypoint_lasttime = time.time()
        self.master.mav.mission_write_partial_list_send(
            self.target_system,
            self.target_component,
            offset,
            offset,
            mission_type=self.mav_mission_type())

    def cmd_movemulti(self, args, latlon=None):
        '''handle wp move of multiple waypoints'''
        if not self.check_have_list():
            return
        if len(args) < 3:
            print("usage: %s movemulti WPNUM WPSTART WPEND <rotation>" %
                  (self.command_name(),))
            return
        idx = int(args[0])
        if not self.good_item_num_to_manipulate(idx):
            print("Invalid move %s number %u" % (self.itemtype(), idx))
            return
        wpstart = int(args[1])
        if not self.good_item_num_to_manipulate(wpstart):
            print("Invalid start %s number %u" % (self.itemtype(), wpstart))
            return
        wpend = int(args[2])
        if not self.good_item_num_to_manipulate(wpend):
            print("Invalid end %s number %u" % (self.itemtype(), wpend))
            return
        if idx < wpstart or idx > wpend:
            print("NUM must be between START and END")
            return

        # optional rotation about center point
        if len(args) > 3:
            rot = float(args[3])
        else:
            rot = 0

        if latlon is None:
            latlon = self.mpstate.click_location
        if latlon is None:
            print("No map click position available")
            return
        offset = self.item_num_to_offset(idx)
        wp = self.wploader.wp(offset)
        if not self.wploader.is_location_command(wp.command):
            print("%s must be a location command" % self.itemtype())
            return

        wp_x = wp.x
        wp_y = wp.y
        if wp.get_type() == "MISSION_ITEM_INT":
            wp_x *= 1e-7
            wp_y *= 1e-7

        (lat, lon) = latlon
        dist = mp_util.gps_distance(wp_x, wp_y, lat, lon)
        bearing = mp_util.gps_bearing(wp_x, wp_y, lat, lon)

        for wpnum in range(wpstart, wpend+1):
            wp = self.wploader.wp(self.item_num_to_offset(wpnum))
            if not self.wploader.is_location_command(wp.command):
                continue
            wp_x = wp.x
            wp_y = wp.y
            if wp.get_type() == "MISSION_ITEM_INT":
                wp_x *= 1e-7
                wp_y *= 1e-7
            (newlat, newlon) = mp_util.gps_newpos(wp_x, wp_y, bearing, dist)
            if wpnum != idx and rot != 0:
                # add in rotation
                d2 = mp_util.gps_distance(lat, lon, newlat, newlon)
                b2 = mp_util.gps_bearing(lat, lon, newlat, newlon)
                (newlat, newlon) = mp_util.gps_newpos(lat, lon, b2+rot, d2)

            if wp.frame != mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT:
                alt1 = self.elevation_map.GetElevation(newlat, newlon)
                alt2 = self.elevation_map.GetElevation(wp_x, wp_y)
                if alt1 is not None and alt2 is not None:
                    wp.z += alt1 - alt2
            wp.x = newlat
            wp.y = newlon
            if wp.get_type() == "MISSION_ITEM_INT":
                wp.x = int(wp.x * 1e7)
                wp.y = int(wp.y * 1e7)
            wp.target_system = self.target_system
            wp.target_component = self.target_component
            print("Sending (%s)" % str(wp))
            self.wploader.set(wp, self.item_num_to_offset(wpnum))

        self.loading_waypoints = True
        self.loading_waypoint_lasttime = time.time()
        self.master.mav.mission_write_partial_list_send(
            self.target_system,
            self.target_component,
            self.item_num_to_offset(wpstart),
            self.item_num_to_offset(wpend),
            mission_type=self.mav_mission_type())
        print("Moved %s %u:%u to %f, %f rotation=%.1f" %
              (self.itemstype(), wpstart, wpend, lat, lon, rot))

    def cmd_changealt(self, args):
        '''handle wp change target alt of multiple waypoints'''
        if not self.check_have_list():
            return
        if len(args) < 2:
            print("usage: %s changealt WPNUM NEWALT <NUMWP>" % self.command_name())
            return
        idx = int(args[0])
        if not self.good_item_num_to_manipulate(idx):
            print("Invalid %s number %u" % (self.itemtype(), idx))
            return
        newalt = float(args[1])
        if len(args) >= 3:
            count = int(args[2])
        else:
            count = 1
        if not self.good_item_num_to_manipulate(idx+count-1):
            print("Invalid %s number %u" % (self.itemtype(), idx+count-1))
            return

        for wpnum in range(idx, idx+count):
            offset = self.item_num_to_offset(wpnum)
            wp = self.wploader.wp(offset)
            if not self.wploader.is_location_command(wp.command):
                continue
            wp.z = newalt
            wp.target_system = self.target_system
            wp.target_component = self.target_component
            print("new wp: (%s)" % str(wp))
            self.wploader.set(wp, offset)

        self.loading_waypoints = True
        self.loading_waypoint_lasttime = time.time()
        self.master.mav.mission_write_partial_list_send(
            self.target_system,
            self.target_component,
            offset,
            offset,
            mission_type=self.mav_mission_type())
        print("Changed alt for WPs %u:%u to %f" % (idx, idx+(count-1), newalt))

    def cmd_remove(self, args):
        '''handle wp remove'''
        if not self.check_have_list():
            return
        if len(args) != 1:
            print("usage: %s remove WPNUM" % self.command_name())
            return
        idx = int(args[0])
        if not self.good_item_num_to_manipulate(idx):
            print("Invalid wp number %u" % idx)
            return
        offset = self.item_num_to_offset(idx)
        wp = self.wploader.wp(offset)
        if wp is None:
            print("Invalid rally point")
            return

        # setup for undo
        self.undo_wp = copy.copy(wp)
        self.undo_wp_idx = idx
        self.undo_type = "remove"

        self.remove(wp)
        self.send_all_waypoints()
        print("Removed WP %u" % idx)

    def remove(self, wp):
        if not self.check_have_list():
            return
        self.wploader.remove(wp)
        expected = getattr(self.wploader, 'expected_count', None)
        if expected is not None:
            self.wploader.expected_count -= 1

    def insert(self, undo_wp_idx, wp):
        self.wploader.insert(self.item_num_to_offset(undo_wp_idx), wp)

    def cmd_undo(self, args):
        '''handle wp undo'''
        if self.undo_wp_idx == -1 or self.undo_wp is None:
            print("No undo information")
            return
        wp = self.undo_wp
        if self.undo_type == 'move':
            wp.target_system = self.target_system
            wp.target_component = self.target_component
            self.loading_waypoints = True
            self.loading_waypoint_lasttime = time.time()
            self.master.mav.mission_write_partial_list_send(
                self.target_system,
                self.target_component,
                self.item_num_to_offset(self.undo_wp_idx),
                self.item_num_to_offset(self.undo_wp_idx),
                mission_type=self.mav_mission_type())
            self.wploader.set(wp, self.item_num_to_offset(self.undo_wp_idx))
            print("Undid %s move" % self.itemtype())
        elif self.undo_type == 'remove':
            self.insert(self.undo_wp_idx, wp)
            self.send_all_waypoints()
            print("Undid %s remove" % self.itemtype())
        else:
            print("bad undo type")
        self.undo_wp = None
        self.undo_wp_idx = -1

    def cmd_param(self, args):
        '''handle wp parameter change'''
        if not self.check_have_list():
            return
        if len(args) < 2:
            print("usage: %s param WPNUM PNUM <VALUE>" % self.command_name())
            return
        idx = int(args[0])
        if not self.good_item_num_to_manipulate(idx):
            print("Invalid %s number %u" % (self.itemtype(), idx))
            return
        wp = self.wploader.wp(self.item_num_to_offset(idx))
        param = [wp.param1, wp.param2, wp.param3, wp.param4]
        pnum = int(args[1])
        if pnum < 1 or pnum > 4:
            print("Invalid param number %u" % pnum)
            return

        if len(args) == 2:
            print("Param %u: %f" % (pnum, param[pnum-1]))
            return

        param[pnum-1] = float(args[2])
        wp.param1 = param[0]
        wp.param2 = param[1]
        wp.param3 = param[2]
        wp.param4 = param[3]

        wp.target_system = self.target_system
        wp.target_component = self.target_component
        self.loading_waypoints = True
        self.loading_waypoint_lasttime = time.time()
        self.master.mav.mission_write_partial_list_send(
            self.target_system,
            self.target_component,
            self.item_num_to_offset(idx),
            self.item_num_to_offset(idx),
            mission_type=self.mav_mission_type())
        self.wploader.set(wp, self.item_num_to_offset(idx))
        print("Set param %u for %u to %f" % (pnum, idx, param[pnum-1]))

    def append(self, item):
        '''append an item to the held item list'''
        if not self.check_have_list():
            return
        if type(item) == list:
            for i in item:
                self.wploader.add(i)
        else:
            self.wploader.add(item)
        self.wploader.reindex()


    def cmd_list(self, args):
        self.wp_op = "list"
        self.request_list_send()

    def cmd_load(self, args):
        print("args: %s" % str(args))
        if len(args) != 1:
            print("usage: %s load FILENAME" % self.command_name())
            return
        self.load_waypoints(args[0])

    def cmd_save(self, args):
        if len(args) != 1:
            print("usage: wp save <filename>")
            return
        self.wp_save_filename = args[0]
        self.wp_op = "save"
        self.request_list_send()

    def cmd_savecsv(self, args):
        if len(args) != 1:
            print("usage: wp savecsv <filename.csv>")
            return
        self.savecsv(args[0])

    def cmd_savelocal(self, args):
        if len(args) != 1:
            print("usage: wp savelocal <filename>")
            return
        self.wploader.save(args[0])

    def cmd_show(self, args):
        if len(args) != 1:
            print("usage: wp show <filename>")
            return
        self.wploader.load(args[0])

    def cmd_update(self, args):
        if not self.check_have_list():
            return
        if len(args) < 1:
            print("usage: %s update <filename> <wpnum>" % cmdname)
            return
        if len(args) == 2:
            wpnum = int(args[1])
        else:
            wpnum = -1
        self.update_waypoints(args[0], wpnum)

    def commands(self):
        return {
            "changealt": self.cmd_changealt,
            "clear": self.cmd_clear,
            "list": self.cmd_list,
            "load": (self.cmd_load, ["(FILENAME)"]),
            "move": self.cmd_move,
            "movemulti": self.cmd_movemulti,
            "param": self.cmd_param,
            "remove": self.cmd_remove,
            "save": (self.cmd_save, ["(FILENAME)"]),
            "savecsv": (self.cmd_savecsv, ["(FILENAME)"]),
            "savelocal": self.cmd_savelocal,
            "show": (self.cmd_show, ["(FILENAME)"]),
            "status": self.cmd_status,
            "undo": self.cmd_undo,
            "update": (self.cmd_update, ["(FILENAME)"]),
        }

    def usage(self):
        subcommands = "|".join(sorted(self.commands().keys()))
        return "usage: %s <%s>" % (self.command_name(), subcommands)

    def cmd_wp(self, args):
        '''waypoint commands'''
        if len(args) < 1:
            print(self.usage())
            return

        commands = self.commands()
        if args[0] not in commands:
            print(self.usage())
            return

        function = commands[args[0]]
        if type(function) == tuple:
            (function, function_arguments) = function
            # TODO: do some argument validation here, remove same from
            # cmd_*

        function(args[1:])

    def pretty_enum_value(self, enum_name, enum_value):
        if enum_name == "MAV_FRAME":
            if enum_value == 0:
                return "Abs"
            elif enum_value == 1:
                return "Local"
            elif enum_value == 2:
                return "Mission"
            elif enum_value == 3:
                return "Rel"
            elif enum_value == 4:
                return "Local ENU"
            elif enum_value == 5:
                return "Global (INT)"
            elif enum_value == 10:
                return "AGL"
        ret = mavutil.mavlink.enums[enum_name][enum_value].name
        ret = ret[len(enum_name)+1:]
        return ret

    def csv_line(self, line):
        '''turn a list of values into a CSV line'''
        self.csv_sep = ","
        return self.csv_sep.join(['"' + str(x) + '"' for x in line])

    def pretty_parameter_value(self, value):
        '''pretty parameter value'''
        return value

    def savecsv(self, filename):
        '''save waypoints to a file in human-readable CSV file'''
        f = open(filename, mode='w')
        headers = ["Seq", "Frame", "Cmd",
                   "P1", "P2", "P3", "P4",
                   "X", "Y", "Z"]
        print(self.csv_line(headers))
        f.write(self.csv_line(headers) + "\n")
        for w in self.wploader.wpoints:
            if getattr(w, 'comment', None):
                # f.write("# %s\n" % w.comment)
                pass
            out_list = [w.seq,
                        self.pretty_enum_value('MAV_FRAME', w.frame),
                        self.pretty_enum_value('MAV_CMD', w.command),
                        self.pretty_parameter_value(w.param1),
                        self.pretty_parameter_value(w.param2),
                        self.pretty_parameter_value(w.param3),
                        self.pretty_parameter_value(w.param4),
                        self.pretty_parameter_value(w.x),
                        self.pretty_parameter_value(w.y),
                        self.pretty_parameter_value(w.z),
                        ]
            print(self.csv_line(out_list))
            f.write(self.csv_line(out_list) + "\n")
        f.close()

    def fetch(self):
        """Download wpts from vehicle (this operation is public to support
        other modules)"""
        # If we were already doing a list or save, just restart the
        # fetch without changing the operation
        if self.wp_op is None:
            self.wp_op = "fetch"
        self.request_list_send()

    def request_list_send(self):
        if self.master.mavlink20():
            self.master.mav.mission_request_list_send(
                self.target_system,
                self.target_component,
                mission_type=self.mav_mission_type())
        else:
            self.master.waypoint_request_list_send()

#!/usr/bin/env python
'''waypoint command handling'''

import time, os, fnmatch, copy, platform
from pymavlink import mavutil, mavwp
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import *
from MAVProxy.modules.lib import mission_item_protocol

class WPModule(mission_item_protocol.MissionItemProtocolModule):
    def __init__(self, mpstate):
        super(WPModule, self).__init__(mpstate, "wp", "waypoint handling", public = True)

    def gui_menu_items(self):
        ret = super(WPModule, self).gui_menu_items()
        ret.extend([
            MPMenuItem('Editor', 'Editor', '# wp editor'),
            MPMenuItem('Draw', 'Draw', '# wp draw ',
                       handler=MPMenuCallTextDialog(title='Mission Altitude (m)',
                                                    default=100)),
            MPMenuItem('Loop', 'Loop', '# wp loop'),
            MPMenuItem('Add NoFly', 'Loop', '# wp noflyadd'),
        ])
        return ret

    def loader_class(self):
        return mavwp.MAVWPLoader

    def mav_mission_type(self):
        return mavutil.mavlink.MAV_MISSION_TYPE_MISSION

    def save_filename_base(self):
        return 'way'

    def itemstype(self):
        '''returns description of items in the plural'''
        return 'waypoints'

    def itemtype(self):
        '''returns description of item'''
        return 'waypoint'

    def index_from_0(self):
        # other similar user-visible interfaces start indexing
        # user-modifiable items from 1.  waypoints make index 0
        # visible to the user.
        return True

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        mtype = m.get_type()
        if mtype in ["WAYPOINT_CURRENT", "MISSION_CURRENT"]:
            if m.seq != self.last_waypoint:
                self.last_waypoint = m.seq
                if self.settings.wpupdates:
                    self.say("waypoint %u" % m.seq,priority='message')

        elif mtype == "MISSION_ITEM_REACHED":
            wp = self.wploader.wp(m.seq)
            if wp is None:
                # should we spit out a warning?!
                # self.say("No waypoints")
                pass
            else:
                if wp.command == mavutil.mavlink.MAV_CMD_DO_LAND_START:
                    alt_offset = self.get_mav_param('ALT_OFFSET', 0)
                    if alt_offset > 0.005:
                        self.say("ALT OFFSET IS NOT ZERO passing DO_LAND_START")
        super(WPModule, self).mavlink_packet(m)

    def get_default_frame(self):
        '''default frame for waypoints'''
        if self.settings.terrainalt == 'Auto':
            if self.get_mav_param('TERRAIN_FOLLOW',0) == 1:
                return mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT
            return mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        if self.settings.terrainalt == 'True':
            return mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT
        return mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT

    def get_home(self):
        '''get home location'''
        if 'HOME_POSITION' in self.master.messages:
            h = self.master.messages['HOME_POSITION']
            return mavutil.mavlink.MAVLink_mission_item_message(self.target_system,
                                                                self.target_component,
                                                                0,
                                                                0,
                                                                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                                                0, 0, 0, 0, 0, 0,
                                                                h.latitude*1.0e-7, h.longitude*1.0e-7, h.altitude*1.0e-3)
        if self.wploader.count() > 0:
            return self.wploader.wp(0)
        return None
        

    def wp_draw_callback(self, points):
        '''callback from drawing waypoints'''
        if len(points) < 3:
            return
        from MAVProxy.modules.lib import mp_util
        home = self.get_home()
        if home is None:
            print("Need home location for draw - please run gethome")
            return
        self.wploader.clear()
        self.wploader.target_system = self.target_system
        self.wploader.target_component = self.target_component
        self.wploader.add(home)
        if self.get_default_frame() == mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT:
            use_terrain = True
        else:
            use_terrain = False
        for p in points:
            self.wploader.add_latlonalt(p[0], p[1], self.settings.wpalt, terrain_alt=use_terrain)
        self.send_all_waypoints()

    def cmd_loop(self, args):
        '''close the loop on a mission'''
        loader = self.wploader
        if loader.count() < 2:
            print("Not enough waypoints (%u)" % loader.count())
            return
        wp = loader.wp(loader.count()-2)
        if wp.command == mavutil.mavlink.MAV_CMD_DO_JUMP:
            print("Mission is already looped")
            return
        wp = mavutil.mavlink.MAVLink_mission_item_message(0, 0, 0, 0, mavutil.mavlink.MAV_CMD_DO_JUMP,
                                                          0, 1, 1, -1, 0, 0, 0, 0, 0)
        loader.add(wp)
        self.loading_waypoints = True
        self.loading_waypoint_lasttime = time.time()
        self.master.waypoint_count_send(self.wploader.count())
        print("Closed loop on mission")

    def cmd_noflyadd(self):
        '''add a square flight exclusion zone'''
        latlon = self.mpstate.click_location
        if latlon is None:
            print("No click position available")
            return
        loader = self.wploader
        (center_lat, center_lon) = latlon
        points = []
        points.append(mp_util.gps_offset(center_lat, center_lon, -25,  25))
        points.append(mp_util.gps_offset(center_lat, center_lon,  25,  25))
        points.append(mp_util.gps_offset(center_lat, center_lon,  25, -25))
        points.append(mp_util.gps_offset(center_lat, center_lon, -25, -25))
        start_idx = loader.count()
        for p in points:
            wp = mavutil.mavlink.MAVLink_mission_item_message(0, 0, 0, 0, mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
                                                              0, 1, 4, 0, 0, 0, p[0], p[1], 0)
            loader.add(wp)
        self.loading_waypoints = True
        self.loading_waypoint_lasttime = time.time()
        self.master.mav.mission_write_partial_list_send(self.target_system,
                                                        self.target_component,
                                                        start_idx, start_idx+4)
        print("Added nofly zone")

    def insert(self, undo_wp_index, wp):
        super(WPModule, self).insert(undo_wp_index, wp)
        self.fix_jumps(self.undo_wp_idx, 1)

    def remove(self, wp):
        super(WPModule, self).remove(wp)
        self.fix_jumps(wp.seq, -1)

    def command_name(self):
        return "wp"

    def get_loc(self, m):
        '''return a mavutil.location for item m'''
        t = m.get_type()
        if t == "MISSION_ITEM":
            lat = m.x * 1e7
            lng = m.y * 1e7
            alt = m.z * 1e2
        elif t == "MISSION_ITEM_INT":
            lat = m.x
            lng = m.y
            alt = m.z
        else:
            return None
        return mavutil.location(lat, lng, alt)

    def cmd_split(self, args):
        '''splits the segment ended by the supplied waypoint into two'''
        try:
            num = int(args[0])
        except IOError as e:
            return "Bad wp num (%s)" % args[0]

        if not self.good_item_num_to_manipulate(num):
            print("Bad item %s" % str(num))
            return
        wp = self.wploader.wp(num)
        if wp is None:
            print("Could not get wp %u" % num)
            return
        loc = self.get_loc(wp)
        if loc is None:
            print("wp is not a location command")
            return

        prev = num - 1
        if not self.good_item_num_to_manipulate(prev):
            print("Bad item %u" % num)
            return
        prev_wp = self.wploader.wp(prev)
        if prev_wp is None:
            print("Could not get previous wp %u" % prev)
            return
        prev_loc = self.get_loc(prev_wp)
        if prev_loc is None:
            print("previous wp is not a location command")
            return

        if wp.frame != prev_wp.frame:
            print("waypoints differ in frame (%u vs %u)" %
                  (wp.frame, prev_wp.frame))
            return

        if wp.frame != prev_wp.frame:
            print("waypoints differ in frame")
            return

        lat_avg = (loc.lat + prev_loc.lat)/2
        lng_avg = (loc.lng + prev_loc.lng)/2
        alt_avg = (loc.alt + prev_loc.alt)/2
        new_wp = mavutil.mavlink.MAVLink_mission_item_message(
            self.target_system,
            self.target_component,
            wp.seq,    # seq
            wp.frame,    # frame
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,    # command
            0,    # current
            0,    # autocontinue
            0.0,  # param1,
            0.0,  # param2,
            0.0,  # param3
            0.0,  # param4
            lat_avg * 1e-7,  # x (latitude)
            lng_avg * 1e-7,  # y (longitude)
            alt_avg * 1e-2,  # z (altitude)
            self.mav_mission_type(),
        )
        self.wploader.insert(wp.seq, new_wp)
        self.fix_jumps(wp.seq, 1)
        self.send_all_waypoints()

    def cmd_sethome(self, args):
        '''set home location from last map click'''
        latlon = self.mpstate.click_location
        if latlon is None:
            print("No click position available")
            return

        lat = float(latlon[0])
        lon = float(latlon[1])
        if self.wploader.count() == 0:
            self.wploader.add_latlonalt(lat, lon, 0)
        w = self.wploader.wp(0)
        w.x = lat
        w.y = lon
        self.wploader.set(w, 0)
        self.loading_waypoints = True
        self.loading_waypoint_lasttime = time.time()
        self.master.mav.mission_write_partial_list_send(self.target_system,
                                                             self.target_component,
                                                             0, 0)

    def fix_jumps(self, idx, delta):
        '''fix up jumps when we add/remove rows'''
        numrows = self.wploader.count()
        for row in range(numrows):
            wp = self.wploader.wp(row)
            jump_cmds = [mavutil.mavlink.MAV_CMD_DO_JUMP]
            if hasattr(mavutil.mavlink, "MAV_CMD_DO_CONDITION_JUMP"):
                jump_cmds.append(mavutil.mavlink.MAV_CMD_DO_CONDITION_JUMP)
            if wp.command in jump_cmds:
                p1 = int(wp.param1)
                if p1 > idx and p1+delta>0:
                    wp.param1 = float(p1+delta)
                    self.wploader.set(wp, row)

    def cmd_slope(self, args):
        '''show slope of waypoints'''
        if len(args) == 2:
            # specific waypoints
            wp1 = int(args[0])
            wp2 = int(args[1])
            w1 = self.wploader.wp(wp1)
            w2 = self.wploader.wp(wp2)
            delta_alt = w1.z - w2.z
            if delta_alt == 0:
                slope = "Level"
            else:
                delta_xy = mp_util.gps_distance(w1.x, w1.y, w2.x, w2.y)
                slope = "%.1f" % (delta_xy / delta_alt)
            print("wp%u -> wp%u %s" % (wp1, wp2, slope))
            return
        if len(args) != 0:
            print("Usage: wp slope WP1 WP2")
            return
        last_w = None
        for i in range(1, self.wploader.count()):
            w = self.wploader.wp(i)
            if w.command not in [mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, mavutil.mavlink.MAV_CMD_NAV_LAND]:
                continue
            if last_w is not None:
                if last_w.frame != w.frame:
                    print("WARNING: frame change %u -> %u at %u" % (last_w.frame, w.frame, i))
                delta_alt = last_w.z - w.z
                if delta_alt == 0:
                    slope = "Level"
                else:
                    delta_xy = mp_util.gps_distance(w.x, w.y, last_w.x, last_w.y)
                    slope = "%.1f" % (delta_xy / delta_alt)
                print("WP%u: slope %s" % (i, slope))
            last_w = w

    def cmd_draw(self, args):
        if 'draw_lines' not in self.mpstate.map_functions:
            print("No map drawing available")
            return
        if self.get_home() is None:
            print("Need home location - please run gethome")
            return
        if len(args) > 1:
            self.settings.wpalt = int(args[1])
        self.mpstate.map_functions['draw_lines'](self.wp_draw_callback)
        print("Drawing %s on map at altitude %d" %
              (self.itemstype(), self.settings.wpalt))

    def cmd_editor(self, args):
        if self.module('misseditor'):
            self.mpstate.functions.process_stdin("module reload misseditor", immediate=True)
        else:
            self.mpstate.functions.process_stdin("module load misseditor", immediate=True)

    def cmd_set(self, args):
        if len(args) != 1:
            print("usage: wp set <wpindex>")
            return
        self.master.waypoint_set_current_send(int(args[0]))

    def commands(self):
        ret = super(WPModule, self).commands()
        ret.update({
            'draw': self.cmd_draw,
            'editor': self.cmd_editor,
            'loop': self.cmd_loop,
            'noflyadd': self.cmd_noflyadd,
            'set': self.cmd_set,
            'sethome': self.cmd_sethome,
            'slope': self.cmd_slope,
            'split': self.cmd_split,
        })
        return ret

    def mission_type_string(self):
        return 'Mission'

def init(mpstate):
    '''initialise module'''
    return WPModule(mpstate)

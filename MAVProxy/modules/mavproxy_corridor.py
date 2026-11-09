'''
flight corridor graph handling via MissionItemProtocol

A corridor graph (MAV_MISSION_TYPE_CORRIDOR,
https://github.com/mavlink/mavlink/issues/2646) is a set of nodes, each
a sphere around a point, connected by directed edges, each a cylinder
around the segment between two nodes.  All nodes are uploaded before
all edges, and edges are sorted by the node they start from; this module
keeps the items in that order whatever is done to them.

AP_FLAKE8_CLEAN
'''

import time

from pymavlink import mavutil
from pymavlink import mavwp

from MAVProxy.modules.lib import mission_item_protocol
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util

if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import MPMenuCallTextDialog
    from MAVProxy.modules.lib.mp_menu import MPMenuItem
    from MAVProxy.modules.lib.mp_menu import MPMenuSubMenu

# values of MAV_CORRIDOR_NODE_TYPE, which an old pymavlink may not have
NODE_TYPES = {
    'waypoint': 0,
    'nest': 1,
    'divert': 2,
    'ditch': 3,
}
NODE_TYPE_NAMES = {v: k for (k, v) in NODE_TYPES.items()}

# map colours (RGB) by node type
NODE_COLOURS = {
    0: (80, 160, 255),    # waypoint
    1: (0, 255, 0),       # nest
    2: (255, 220, 0),     # divert site
    3: (255, 60, 60),     # ditch site
}
EDGE_COLOUR = (0, 200, 255)

# nodes and edges with no radius are drawn this big on the map
MIN_DRAW_RADIUS = 2.0

MAP_LAYER = 'Corridor'


def corridor_supported():
    '''true if pymavlink knows about corridor graphs'''
    return getattr(mavutil.mavlink, 'MAV_MISSION_TYPE_CORRIDOR', None) is not None


class MissionItemProtocol_Corridor(mavwp.MissionItemProtocol):
    '''loader for corridor graph items'''

    def mav_mission_type(self):
        return mavutil.mavlink.MAV_MISSION_TYPE_CORRIDOR

    def is_location_command(self, cmd):
        return cmd == mavutil.mavlink.MAV_CMD_NAV_CORRIDOR_NODE


class Node(object):
    def __init__(self, lat, lon, amsl, radius, node_type):
        self.lat = lat
        self.lon = lon
        self.amsl = amsl
        self.radius = radius
        self.type = node_type


class Edge(object):
    def __init__(self, node_from, node_to, cost, radius):
        self.node_from = node_from
        self.node_to = node_to
        self.cost = cost
        self.radius = radius


class CorridorModule(mission_item_protocol.MissionItemProtocolModule):

    def __init__(self, mpstate):
        '''initialise module'''
        self.corridor_settings = mp_settings.MPSettings([
            # altitude above home for new nodes
            ('alt', float, 50.0),
            ('noderadius', float, 20.0),
            ('edgeradius', float, 15.0),
            # add and change edges in both directions at once
            ('bidirectional', bool, True),
            # drawn points this close to a node join it
            ('snap', float, 30.0),
        ])
        super().__init__(
            mpstate,
            "corridor",
            "flight corridor graph management",
            public=True)
        self.add_completion_function('(CORRIDORSETTING)',
                                     self.corridor_settings.completion)
        self.edge_start = None
        self.moving_node = None
        self.map_drawn_change = None

    def command_name(self):
        '''command-line command name'''
        return "corridor"

    @staticmethod
    def loader_class():
        return MissionItemProtocol_Corridor

    def mav_mission_type(self):
        return mavutil.mavlink.MAV_MISSION_TYPE_CORRIDOR

    def itemstype(self):
        '''returns description of items in the plural'''
        return 'corridor items'

    def itemtype(self):
        '''returns description of item'''
        return 'corridor item'

    def index_from_0(self):
        '''nodes are identified by their zero-based index'''
        return True

    def mission_ftp_name(self):
        return None

    # ------------------------------------------------------------------ graph
    def graph(self):
        '''return (nodes, edges) from the held items'''
        nodes = []
        edges = []
        for i in range(self.wploader.count()):
            w = self.wploader.item(i)
            if w.command == mavutil.mavlink.MAV_CMD_NAV_CORRIDOR_NODE:
                (lat, lon) = (w.x, w.y)
                if w.get_type() == 'MISSION_ITEM_INT':
                    (lat, lon) = (lat * 1.0e-7, lon * 1.0e-7)
                nodes.append(Node(lat, lon, w.z, w.param2, int(w.param1)))
            elif w.command == mavutil.mavlink.MAV_CMD_NAV_CORRIDOR_EDGE:
                edges.append(Edge(int(w.param1), int(w.param2), w.param3, w.param4))
        return (nodes, edges)

    def corridor_graph(self):
        '''the graph for other modules to draw, as ([(lat, lon, amsl,
        radius, type)], [(from, to, radius)])'''
        (nodes, edges) = self.graph()
        return ([(n.lat, n.lon, n.amsl, n.radius, n.type) for n in nodes],
                [(e.node_from, e.node_to, e.radius) for e in edges])

    def node_item(self, node):
        return mavutil.mavlink.MAVLink_mission_item_message(
            self.target_system,
            self.target_component,
            0,    # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_NAV_CORRIDOR_NODE,
            0,    # current
            0,    # autocontinue
            node.type,
            node.radius,
            0,
            0,
            node.lat,
            node.lon,
            node.amsl,
            self.mav_mission_type(),
        )

    def edge_item(self, edge):
        return mavutil.mavlink.MAVLink_mission_item_message(
            self.target_system,
            self.target_component,
            0,    # seq
            mavutil.mavlink.MAV_FRAME_MISSION,
            mavutil.mavlink.MAV_CMD_NAV_CORRIDOR_EDGE,
            0,    # current
            0,    # autocontinue
            edge.node_from,
            edge.node_to,
            edge.cost,
            edge.radius,
            0,
            0,
            0,
            self.mav_mission_type(),
        )

    def set_graph(self, nodes, edges, send=True):
        '''replace the held items with nodes and edges, in upload order,
        and send them to the vehicle'''
        # one edge per (from, to), sorted by from:
        unique = {}
        for e in edges:
            if e.node_from == e.node_to:
                continue
            unique[(e.node_from, e.node_to)] = e
        edges = [unique[k] for k in sorted(unique.keys())]

        self.wploader.target_system = self.target_system
        self.wploader.target_component = self.target_component
        self.wploader.clear()
        for n in nodes:
            self.wploader.add(self.node_item(n))
        for e in edges:
            self.wploader.add(self.edge_item(e))
        self.wploader.expected_count = self.wploader.count()
        self.wploader.last_change = time.time()
        if send:
            self.send_all_items()

    def find_edges(self, edges, node_from, node_to):
        '''edges between node_from and node_to (in both directions if
        bidirectional is set)'''
        ret = []
        for e in edges:
            if e.node_from == node_from and e.node_to == node_to:
                ret.append(e)
            elif (self.corridor_settings.bidirectional and
                  e.node_from == node_to and e.node_to == node_from):
                ret.append(e)
        return ret

    def nearest_node(self, nodes, latlon, max_distance=None):
        '''index of the node nearest latlon, or None'''
        best = None
        best_distance = None
        for (i, n) in enumerate(nodes):
            distance = mp_util.gps_distance(latlon[0], latlon[1], n.lat, n.lon)
            if max_distance is not None and distance > max(max_distance, n.radius):
                continue
            if best_distance is None or distance < best_distance:
                best = i
                best_distance = distance
        return best

    def home_amsl(self):
        '''altitude of home, AMSL, or None if not known'''
        if self.master is not None:
            home = self.master.messages.get('HOME_POSITION', None)
            if home is not None:
                return home.altitude * 1.0e-3
        try:
            home = self.module('wp').get_home()
        except Exception:
            home = None
        if home is not None and home.frame == mavutil.mavlink.MAV_FRAME_GLOBAL:
            return home.z
        return None

    def amsl_from_args(self, args, index):
        '''an altitude above home from args[index] or the alt setting,
        converted to AMSL'''
        alt = self.corridor_settings.alt
        if len(args) > index:
            alt = float(args[index])
        home = self.home_amsl()
        if home is None:
            print("corridor: home altitude not known")
            return None
        return home + alt

    def parse_node_type(self, name):
        try:
            return NODE_TYPES[name.lower()]
        except KeyError:
            pass
        try:
            value = int(name)
        except ValueError:
            value = None
        if value not in NODE_TYPE_NAMES:
            print("corridor: node type must be one of %s" % "|".join(NODE_TYPES.keys()))
            return None
        return value

    def parse_node_index(self, nodes, arg):
        try:
            idx = int(arg)
        except ValueError:
            idx = -1
        if idx < 0 or idx >= len(nodes):
            print("corridor: no node %s (have %u)" % (arg, len(nodes)))
            return None
        return idx

    def click_location(self):
        latlon = self.mpstate.click_location
        if latlon is None:
            print("corridor: no map click position available")
        return latlon

    # --------------------------------------------------------------- commands
    def cmd_add(self, args):
        '''add a node at the last map click position'''
        if len(args) < 1:
            print("usage: corridor add <%s> [ALT] [RADIUS]" % "|".join(NODE_TYPES.keys()))
            return
        node_type = self.parse_node_type(args[0])
        if node_type is None:
            return
        latlon = self.click_location()
        if latlon is None:
            return
        amsl = self.amsl_from_args(args, 1)
        if amsl is None:
            return
        radius = self.corridor_settings.noderadius
        if len(args) > 2:
            radius = float(args[2])
        (nodes, edges) = self.graph()
        nodes.append(Node(latlon[0], latlon[1], amsl, radius, node_type))
        print("corridor: adding %s node %u" % (NODE_TYPE_NAMES[node_type], len(nodes) - 1))
        self.set_graph(nodes, edges)

    def add_edges(self, edges, node_from, node_to, radius, cost=0):
        edges.append(Edge(node_from, node_to, cost, radius))
        if self.corridor_settings.bidirectional:
            edges.append(Edge(node_to, node_from, cost, radius))

    def cmd_addedge(self, args):
        '''add an edge between two nodes'''
        if len(args) < 2:
            print("usage: corridor addedge FROM TO [RADIUS] [COST]")
            return
        (nodes, edges) = self.graph()
        node_from = self.parse_node_index(nodes, args[0])
        node_to = self.parse_node_index(nodes, args[1])
        if node_from is None or node_to is None:
            return
        if node_from == node_to:
            print("corridor: an edge must join two different nodes")
            return
        radius = self.corridor_settings.edgeradius
        if len(args) > 2:
            radius = float(args[2])
        cost = 0
        if len(args) > 3:
            cost = float(args[3])
        self.add_edges(edges, node_from, node_to, radius, cost)
        self.set_graph(nodes, edges)

    def cmd_edgefrom(self, args):
        '''note the node an edge started on the map starts from'''
        (nodes, edges) = self.graph()
        if len(args) != 1:
            print("usage: corridor edgefrom NODE")
            return
        self.edge_start = self.parse_node_index(nodes, args[0])
        if self.edge_start is not None:
            print("corridor: edge from node %u; choose 'End edge here' on another node" % self.edge_start)

    def cmd_edgeto(self, args):
        '''add an edge from the node chosen with edgefrom'''
        if len(args) != 1:
            print("usage: corridor edgeto NODE")
            return
        if self.edge_start is None:
            print("corridor: choose 'Start edge here' on a node first")
            return
        self.cmd_addedge([str(self.edge_start), args[0]])
        self.edge_start = None

    def cmd_removeedge(self, args):
        '''remove the edge(s) between two nodes'''
        if len(args) != 2:
            print("usage: corridor removeedge FROM TO")
            return
        (nodes, edges) = self.graph()
        doomed = self.find_edges(edges, int(args[0]), int(args[1]))
        if len(doomed) == 0:
            print("corridor: no edge from %s to %s" % (args[0], args[1]))
            return
        self.set_graph(nodes, [e for e in edges if e not in doomed])

    def cmd_removenode(self, args):
        '''remove a node and the edges which use it'''
        if len(args) != 1:
            print("usage: corridor removenode NODE")
            return
        (nodes, edges) = self.graph()
        idx = self.parse_node_index(nodes, args[0])
        if idx is None:
            return
        del nodes[idx]
        new_edges = []
        for e in edges:
            if idx in (e.node_from, e.node_to):
                continue
            if e.node_from > idx:
                e.node_from -= 1
            if e.node_to > idx:
                e.node_to -= 1
            new_edges.append(e)
        self.edge_start = None
        self.set_graph(nodes, new_edges)

    def cmd_movenode(self, args):
        '''move a node to the last map click position'''
        if len(args) != 1:
            print("usage: corridor movenode NODE")
            return
        (nodes, edges) = self.graph()
        idx = self.parse_node_index(nodes, args[0])
        if idx is None:
            return
        latlon = self.click_location()
        if latlon is None:
            return
        (nodes[idx].lat, nodes[idx].lon) = latlon
        self.set_graph(nodes, edges)

    def cmd_startmove(self, args):
        '''move a node to the next point clicked on the map'''
        (nodes, edges) = self.graph()
        if len(args) != 1:
            print("usage: corridor startmove NODE")
            return
        idx = self.parse_node_index(nodes, args[0])
        if idx is None:
            return
        if 'draw_lines' not in self.mpstate.map_functions:
            print("corridor: no map loaded")
            return
        self.moving_node = idx
        print("corridor: left-click the new position of node %u, then right-click" % idx)
        self.mpstate.map_functions['draw_lines'](self.move_draw_callback)

    def move_draw_callback(self, points):
        idx = self.moving_node
        self.moving_node = None
        if idx is None or len(points) == 0:
            return
        self.mpstate.click(points[-1])
        self.cmd_movenode([str(idx)])

    def cmd_nodetype(self, args):
        if len(args) != 2:
            print("usage: corridor nodetype NODE <%s>" % "|".join(NODE_TYPES.keys()))
            return
        (nodes, edges) = self.graph()
        idx = self.parse_node_index(nodes, args[0])
        node_type = self.parse_node_type(args[1])
        if idx is None or node_type is None:
            return
        nodes[idx].type = node_type
        self.set_graph(nodes, edges)

    def cmd_noderadius(self, args):
        if len(args) != 2:
            print("usage: corridor noderadius NODE RADIUS")
            return
        (nodes, edges) = self.graph()
        idx = self.parse_node_index(nodes, args[0])
        if idx is None:
            return
        nodes[idx].radius = float(args[1])
        self.set_graph(nodes, edges)

    def cmd_nodealt(self, args):
        '''set a node's altitude above home'''
        if len(args) != 2:
            print("usage: corridor nodealt NODE ALT")
            return
        (nodes, edges) = self.graph()
        idx = self.parse_node_index(nodes, args[0])
        if idx is None:
            return
        amsl = self.amsl_from_args(args, 1)
        if amsl is None:
            return
        nodes[idx].amsl = amsl
        self.set_graph(nodes, edges)

    def cmd_edgeradius(self, args):
        if len(args) != 3:
            print("usage: corridor edgeradius FROM TO RADIUS")
            return
        (nodes, edges) = self.graph()
        changed = self.find_edges(edges, int(args[0]), int(args[1]))
        if len(changed) == 0:
            print("corridor: no edge from %s to %s" % (args[0], args[1]))
            return
        for e in changed:
            e.radius = float(args[2])
        self.set_graph(nodes, edges)

    def cmd_draw(self, args):
        '''draw a corridor on the map: each point clicked is a node,
        joined to the one before by an edge.  Points clicked near an
        existing node use it'''
        if 'draw_lines' not in self.mpstate.map_functions:
            print("corridor: no map loaded")
            return
        if self.home_amsl() is None:
            print("corridor: home altitude not known")
            return
        print("corridor: left-click the corridor's nodes, then right-click to finish")
        self.mpstate.map_functions['draw_lines'](self.draw_callback, colour=EDGE_COLOUR)

    def draw_callback(self, points):
        if len(points) == 0:
            return
        (nodes, edges) = self.graph()
        amsl = self.home_amsl() + self.corridor_settings.alt
        previous = None
        for latlon in points:
            idx = self.nearest_node(nodes, latlon, max_distance=self.corridor_settings.snap)
            if idx is None:
                nodes.append(Node(latlon[0], latlon[1], amsl,
                                  self.corridor_settings.noderadius,
                                  NODE_TYPES['waypoint']))
                idx = len(nodes) - 1
            if previous is not None and previous != idx:
                self.add_edges(edges, previous, idx, self.corridor_settings.edgeradius)
            previous = idx
        self.set_graph(nodes, edges)

    def cmd_example(self, args):
        '''create the example graph from the RFC, with radii, around the
        last map click position'''
        latlon = self.click_location()
        if latlon is None:
            return
        home = self.home_amsl()
        if home is None:
            print("corridor: home altitude not known")
            return
        (lat, lon) = latlon

        def node(north, east, alt, radius, node_type):
            (nlat, nlon) = mp_util.gps_offset(lat, lon, east, north)
            return Node(nlat, nlon, home + alt, radius, NODE_TYPES[node_type])

        nodes = [
            node(0, 0, 30, 15, 'nest'),
            node(400, 0, 50, 25, 'waypoint'),
            node(400, 500, 50, 25, 'waypoint'),
            node(0, 500, 30, 15, 'divert'),
        ]
        edges = []
        self.add_edges(edges, 0, 1, 15)
        self.add_edges(edges, 1, 2, 20)
        edges.append(Edge(2, 3, 0, 15))
        self.set_graph(nodes, edges)

    def cmd_show_graph(self, args):
        '''print the graph'''
        (nodes, edges) = self.graph()
        home = self.home_amsl()
        for (i, n) in enumerate(nodes):
            rel = ""
            if home is not None:
                rel = " (%.1fm above home)" % (n.amsl - home)
            print("node %u: %s %.7f %.7f %.1fm AMSL%s radius=%.1fm" % (
                i, NODE_TYPE_NAMES.get(n.type, str(n.type)), n.lat, n.lon, n.amsl, rel, n.radius))
        for e in edges:
            cost = "distance" if e.cost == 0 else "%.2f" % e.cost
            print("edge %u->%u: cost=%s radius=%.1fm" % (e.node_from, e.node_to, cost, e.radius))

    def cmd_set(self, args):
        self.corridor_settings.command(args)

    def cmd_remove(self, args):
        self.cmd_removenode(args)

    def commands(self):
        '''returns map from command name to handling function'''
        ret = super().commands()
        node_types = "<" + "|".join(NODE_TYPES.keys()) + ">"
        ret.update({
            'add': (self.cmd_add, [node_types]),
            'addedge': self.cmd_addedge,
            'draw': self.cmd_draw,
            'edgefrom': self.cmd_edgefrom,
            'edgeradius': self.cmd_edgeradius,
            'edgeto': self.cmd_edgeto,
            'example': self.cmd_example,
            'graph': self.cmd_show_graph,
            'movenode': self.cmd_movenode,
            'nodealt': self.cmd_nodealt,
            'noderadius': self.cmd_noderadius,
            'nodetype': (self.cmd_nodetype, ["(NODE)", node_types]),
            'remove': self.cmd_remove,
            'removeedge': self.cmd_removeedge,
            'removenode': self.cmd_removenode,
            'set': (self.cmd_set, ["(CORRIDORSETTING)"]),
            'startmove': self.cmd_startmove,
        })
        # these work on items by number, which does not suit a graph
        for name in ('ftp', 'ftpload'):
            ret.pop(name, None)
        return ret

    # ------------------------------------------------------------------- mavlink
    def mavlink_packet(self, m):
        if m.get_type() == 'MISSION_ACK':
            if getattr(m, 'mission_type', 0) == self.mav_mission_type():
                if (m.target_system == self.settings.source_system and
                        m.target_component == self.settings.source_component):
                    result = mavutil.mavlink.enums['MAV_MISSION_RESULT'].get(m.type, None)
                    name = result.name if result is not None else str(m.type)
                    print("corridor: %s" % name)
                    if m.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                        print("corridor: use 'corridor list' to fetch the vehicle's graph again")
        super().mavlink_packet(m)

    # ----------------------------------------------------------------------- map
    def gui_menu_items(self):
        ret = super().gui_menu_items()
        ret = [x for x in ret if x.name not in ('FTP', 'Undo')]
        add_items = [
            MPMenuItem(
                name.capitalize(), name.capitalize(), '# corridor add %s ' % name,
                handler=MPMenuCallTextDialog(
                    title='Node altitude above home (m)',
                    default=self.corridor_settings.alt))
            for name in NODE_TYPES.keys()
        ]
        ret.extend([
            MPMenuSubMenu('Add Node', items=add_items),
            MPMenuItem('Draw', 'Draw', '# corridor draw'),
            MPMenuItem('Example', 'Example', '# corridor example'),
            MPMenuItem('Graph', 'Graph', '# corridor graph'),
        ])
        return ret

    def node_popup(self, idx):
        '''popup menu for a node on the map'''
        type_items = [MPMenuItem(name.capitalize(), name.capitalize(),
                                 '# corridor nodetype %u %s' % (idx, name))
                      for name in NODE_TYPES.keys()]
        return MPMenuSubMenu('Popup', items=[
            MPMenuItem('Node %u' % idx, 'Node %u' % idx, '# corridor graph'),
            MPMenuSubMenu('Set Type', items=type_items),
            MPMenuItem('Set Radius', 'Set Radius', '# corridor noderadius %u ' % idx,
                       handler=MPMenuCallTextDialog(title='Node radius (m)',
                                                    default=self.corridor_settings.noderadius)),
            MPMenuItem('Set Altitude', 'Set Altitude', '# corridor nodealt %u ' % idx,
                       handler=MPMenuCallTextDialog(title='Node altitude above home (m)',
                                                    default=self.corridor_settings.alt)),
            MPMenuItem('Move Node', 'Move Node', '# corridor startmove %u' % idx),
            MPMenuItem('Start Edge Here', 'Start Edge Here', '# corridor edgefrom %u' % idx),
            MPMenuItem('End Edge Here', 'End Edge Here', '# corridor edgeto %u' % idx),
            MPMenuItem('Remove Node', 'Remove Node', '# corridor removenode %u' % idx),
        ])

    def edge_popup(self, edge):
        '''popup menu for an edge on the map'''
        desc = '%u-%u' % (edge.node_from, edge.node_to)
        return MPMenuSubMenu('Popup', items=[
            MPMenuItem('Edge %s' % desc, 'Edge %s' % desc, '# corridor graph'),
            MPMenuItem('Set Radius', 'Set Radius',
                       '# corridor edgeradius %u %u ' % (edge.node_from, edge.node_to),
                       handler=MPMenuCallTextDialog(title='Edge radius (m)',
                                                    default=self.corridor_settings.edgeradius)),
            MPMenuItem('Remove Edge', 'Remove Edge',
                       '# corridor removeedge %u %u' % (edge.node_from, edge.node_to)),
        ])

    @staticmethod
    def edge_outline(a, b, radius):
        '''the corridor around a->b on the ground: a stadium of the given
        radius, as a list of (lat, lon)'''
        bearing = mp_util.gps_bearing(a.lat, a.lon, b.lat, b.lon)
        points = []
        for (centre, start) in ((b, bearing - 90), (a, bearing + 90)):
            for i in range(13):
                points.append(mp_util.gps_newpos(centre.lat, centre.lon, start + i * 15, radius))
        points.append(points[0])
        return points

    def draw_on_map(self, map_module):
        '''draw the graph on the 2D map'''
        from MAVProxy.modules.mavproxy_map import mp_slipmap
        slipmap = map_module.map
        slipmap.add_object(mp_slipmap.SlipClearLayer(MAP_LAYER))
        (nodes, edges) = self.graph()

        # an edge and its reverse are drawn once, as one corridor; a
        # one-way edge has an arrow
        drawn = set()
        for e in edges:
            if e.node_from >= len(nodes) or e.node_to >= len(nodes):
                continue
            key = tuple(sorted((e.node_from, e.node_to)))
            if key in drawn:
                continue
            drawn.add(key)
            (a, b) = (nodes[e.node_from], nodes[e.node_to])
            one_way = not any(r.node_from == e.node_to and r.node_to == e.node_from for r in edges)
            popup = self.edge_popup(e)
            radius = max(e.radius, MIN_DRAW_RADIUS)
            slipmap.add_object(mp_slipmap.SlipPolygon(
                'corridor edge %u-%u' % key,
                self.edge_outline(a, b, radius),
                layer=MAP_LAYER,
                linewidth=2,
                colour=EDGE_COLOUR,
                popup_menu=popup,
                showcircles=False,
                fill_alpha=0.15,
            ))
            slipmap.add_object(mp_slipmap.SlipPolygon(
                'corridor centreline %u-%u' % key,
                [(a.lat, a.lon), (b.lat, b.lon)],
                layer=MAP_LAYER,
                linewidth=1,
                colour=EDGE_COLOUR,
                arrow=one_way,
                popup_menu=popup,
                showcircles=False,
            ))

        # nodes stacked above one another share one label, lowest first
        home = self.home_amsl()
        labels = {}
        for (i, n) in enumerate(nodes):
            colour = NODE_COLOURS.get(n.type, (255, 255, 255))
            slipmap.add_object(mp_slipmap.SlipCircle(
                'corridor node %u' % i,
                MAP_LAYER,
                (n.lat, n.lon),
                max(n.radius, MIN_DRAW_RADIUS),
                colour,
                linewidth=2,
                popup_menu=self.node_popup(i),
            ))
            label = '%u %s' % (i, NODE_TYPE_NAMES.get(n.type, '?'))
            if home is not None:
                label += ' %.0fm' % (n.amsl - home)
            labels.setdefault((n.lat, n.lon), []).append((n.amsl, label, colour, i))
        for ((lat, lon), stack) in labels.items():
            stack.sort()
            slipmap.add_object(mp_slipmap.SlipLabel(
                'corridor label %u' % stack[0][3],
                (lat, lon),
                ' / '.join(x[1] for x in stack),
                MAP_LAYER,
                stack[0][2],
            ))

    def idle_task(self):
        super().idle_task()
        map_module = self.module('map')
        if map_module is None or getattr(map_module, 'map', None) is None:
            self.map_drawn_change = None
            return
        if self.map_drawn_change != self.last_change():
            self.map_drawn_change = self.last_change()
            self.draw_on_map(map_module)

    def unload(self):
        map_module = self.module('map')
        if map_module is not None and getattr(map_module, 'map', None) is not None:
            from MAVProxy.modules.mavproxy_map import mp_slipmap
            map_module.map.add_object(mp_slipmap.SlipClearLayer(MAP_LAYER))
        super().unload()


def init(mpstate):
    '''initialise module'''
    if not corridor_supported():
        print("corridor: pymavlink does not support MAV_MISSION_TYPE_CORRIDOR")
        return None
    return CorridorModule(mpstate)

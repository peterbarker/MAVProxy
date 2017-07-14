#!/usr/bin/env python
'''
ZeroConf Video Module
Peter barker, July 2017

'''

import os
import os.path
import sys
from pymavlink import mavutil
import errno
import time
import Queue

import re
import socket
import string

from zeroconf import ServiceBrowser, Zeroconf

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings

from multiprocessing import Process

# GTK stuff:
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject, Gtk
# end GTK tuff

class video(mp_module.MPModule):
    class ZeroconfListener(object):
        def __init__(self, queue_add, queue_remove):
            self.queue_add = queue_add
            self.queue_remove = queue_remove

        def remove_service(self, zeroconf, type, name):
            self.queue_remove.put(name)

        def add_service(self, zeroconf, type, name):
            info = zeroconf.get_service_info(type, name)
            self.queue_add.put( (name, info) )

    def __init__(self, mpstate):
        """Initialise module"""
        super(video, self).__init__(mpstate, "video", "")

        self.queue_add = Queue.Queue()
        self.queue_remove = Queue.Queue()
        self.zeroconf = Zeroconf()
        self.listener = video.ZeroconfListener(self.queue_add, self.queue_remove)
        self.browser = ServiceBrowser(self.zeroconf, "_rtsp._udp.local.", self.listener)


        self.status_callcount = 0
        self.boredom_interval = 10 # seconds
        self.last_bored = time.time()

        self.packets_mytarget = 0
        self.packets_othertarget = 0
        self.verbose = False

        self.video_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),
          ])
        self.add_command('video', self.cmd_video, "video module", ['status','set (LOGSETTING)', 'list'])
        self.streams = []

        # GTK stuff:
        Gst.init(None)
        GObject.threads_init()
        # end GTK stuff

    def usage(self):
        '''show help on command line options'''
        return "Usage: video <status|set|list>"

    def url_for_info(self, info, props={}):
        address = socket.inet_ntoa(info.address)
        path = string.split(info.name, sep='.')[0]
        # FIXME: use urllib/urllib2/flavour of week
        url = "rtsp://%s:%u%s" % (address, info.port, path)
        joiner = "?"
        for (key,value) in props.iteritems():
            url += "%s%s=%s" % (joiner, key,value) # todo: escaping
            joiner = "&"
        return url

    def cmd_video_list(self, args):
        count = 0
        oldname = None
        for stream in self.streams:
            (name, info) = stream
            url = self.url_for_info(info)
            if name != oldname:
                oldname = name
                print("# %s" % (info.properties["name"]))
            print("%u: %s" % (count, url))
            for (key,value) in info.properties.iteritems():
                if key.find("frame_size") == -1:
                    continue
                frame_size_match = re.match("(\w+)\(([^)]*)\)", value)
                if frame_size_match is None:
                    print("Malformed frame_size? (%s)" % value)
                    continue
                print("    fmt: %s" % (frame_size_match.group(1)))
                res =  frame_size_match.group(2)
                res = res.replace(',', ' ')
                print("      res: %s" % (res,))
            count += 1

    def play_on_message(self, bus, message):
        print("Message: %s" % repr(message))
        t = message.type
        if t == Gst.MessageType.EOS:
            self.player.set_state(Gst.State.NULL)
#            self.button.set_label("Start")
        elif t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print "Error: %s" % err, debug
            self.player.set_state(Gst.State.NULL)
#            self.button.set_label("Start")
        self.player.set_state(Gst.State.NULL)

    def play_on_sync_message(self, bus, message):
#        print("Sync-Message: %s" % str(message))
        struct = message.get_structure()
        if not struct:
            return
        message_name = struct.get_name()
        if message_name == "prepare-xwindow-id":
            # Assign the viewport
            imagesink = message.src
            imagesink.set_property("force-aspect-ratio", True)
            imagesink.set_xwindow_id(self.movie_window.window.xid)

    def play_destroyed_window(self, *rest):
        print("Window was destroyed (%s)" % str(*rest))
        self.player.set_state(Gst.State.NULL)

    def play_on_delete_event(self):
        print("On delete event!")
        self.player.set_state(Gst.State.NULL)

    def play_sync_handler(self, bus, message):
        print("Sync handler! (%s)" % repr(message))
        return Gst.BusSyncReply.PASS
    def play_url(self, title, url):
        #        gst_pipeline = "rtspsrc location=%s ! application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 ! rtph264depay ! avdec_h264 ! autovideosink" % (url,)
#        imagesink = "autovideosink"        # autovideosink crashes mavproxy hard when the window is closed:
        imagesink = "xvimagesink sync=0"

        gst_pipeline = "rtspsrc location={url} ! application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 ! rtph264depay ! avdec_h264 ! {imagesink}".format(url=url, imagesink=imagesink)
        player = Gst.parse_launch(gst_pipeline)
        bus = player.get_bus()
        bus.add_signal_watch()
        bus.enable_sync_message_emission()
        bus.connect("message", self.play_on_message)
        bus.connect("sync-message::element", self.play_on_sync_message)
        bus.set_sync_handler(self.play_sync_handler)
        player.set_state(Gst.State.PLAYING)


    def cmd_video_play(self, args):
        try:
            num = int(args[0])
        except ValueError:
            print("Stream should be a number")
            return
        props = {}
        for arg in args[1:]:
            try:
                (name,value) = arg.split("=")
            except ValueError:
                print("Malfrormed argument (%s)" % str(arg))
                return
            if name == "res":
                try:
                    (x,y) = value.split("x")
                except ValueError:
                    print("Malformed res value (%s)" % str(value))
                    return
                props["width"] = x
                props["height"] = y
            elif name == "fmt":
                props["fmt"] = value
            else:
                print("Unknown arg (%s)" % str(arg))
                return
        stream = self.streams[num]
        (name, info) = stream
        url = self.url_for_info(info, props)
        print("URL: %s" % str(url))
        title = info.properties["name"]
        self.play_url(title, url)

    def cmd_video(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print self.usage()
        elif args[0] == "status":
            print self.status()
        elif args[0] == "set":
            self.video_settings.command(args[1:])
        elif args[0] == "list":
            self.cmd_video_list(args[1:])
        elif args[0] == "play":
            self.cmd_video_play(args[1:])
        else:
            print self.usage()

    def status(self):
        '''returns information about module'''
        self.status_callcount += 1
        self.last_bored = time.time() # status entertains us
        return("status called %(status_callcount)d times.  My target positions=%(packets_mytarget)u  Other target positions=%(packets_mytarget)u" %
               {"status_callcount": self.status_callcount,
                "packets_mytarget": self.packets_mytarget,
                "packets_othertarget": self.packets_othertarget,
               })

    def boredom_message(self):
        if self.video_settings.verbose:
            return ("I'm very bored")
        return ("I'm bored")

    def idle_task(self):
        '''called rapidly by mavproxy'''
        now = time.time()
        if now-self.last_bored > self.boredom_interval:
            self.last_bored = now
            message = self.boredom_message()
            self.say("%s: %s" % (self.name,message))
            # See if whatever we're connected to would like to play:
            self.master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,
                                            message)

        try:
            thing_to_add = self.queue_add.get(block=False)
            if thing_to_add:
                print("Got thing to add %s" % (str(thing_to_add)))
                (name, info) = thing_to_add
                self.streams.append( (name, info) )
        except Queue.Empty:
            pass

        try:
            thing_to_remove = self.queue_remove.get(block=False)
            if thing_to_remove:
                print("Got %s" % (str(thing_to_remove)))
                name = thing_to_remove
                self.streams = [ x for x in self.streams if x[0] != name ]
        except Queue.Empty:
            pass

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        if m.get_type() == 'GLOBAL_POSITION_INT':
            if self.settings.target_system == 0 or self.settings.target_system == m.get_srcSystem():
                self.packets_mytarget += 1
            else:
                self.packets_othertarget += 1

def init(mpstate):
    '''initialise module'''
    return video(mpstate)

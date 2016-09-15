#!/usr/bin/env python

"""Encapsulate RF survey data into a DATA64 packet which is then sent
Peter Barker   September 2016

This module handles both sending and receiving of the data64 packet.  Use "rfsurvey send" on the drone's companion computer, "rfsurvey receive" on the GCS.

An instance of rfsurvey_sample contains all of the data about a sample.

Two threads are created, one to create samples and enqueue them (sample_create_thread_target), one to consume samples from a receive queue (sample_handle_thread_target).

These threads enqueue/dequeue from send and receive queues which are filled and rained by handle_packet and idle callbacks.

Sanity checks are done to ensure the data received is from the same shape of rfsurvey sample as the GCS is expecting.

"""
from __future__ import print_function

import atexit
import io
import math
import Queue
import random
import signal
import struct
import threading
import time
import traceback

from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings

class rfsurvey_sample():
    """Encapsulates data about an RF measurement"""

    @classmethod
    def format_ver(cls):
        """specifies the version of the format of the data packed inside the mavlink DATA packet.  Increment this if you change the fields in below."""
        return 7

    @classmethod
    def fields(cls):
        """field definitions for the data packed inside the DATA64 packets.
        Format is [name, struct.pack format-char, getter-function]
        """
        return [
            [ 'TimeStamp', 'Q', lambda self : self._timestamp ],
            [ 'Lat', 'i', lambda self : self._lat ],
            [ 'Lon', 'i', lambda self : self._lon ],
            [ 'Hdg', 'i', lambda self : self._hdg ],
            [ 'Pitch', 'f', lambda self : self._pitch ],
            [ 'Roll', 'f', lambda self : self._roll ],
            [ 'BaroHT', 'f', lambda self : self._alt ],
            [ 'HDOP', 'f', lambda self : self._hdop ],
            [ 'VDOP', 'f', lambda self : self._vdop ],
            [ 'NSats', 'B', lambda self : self._nsats ],
            [ '578.5', 'f', lambda self : self._578_5 ],
            [ '585.5', 'f', lambda self : self._585_5 ],
            [ '592.5', 'f', lambda self : self._592_5 ],
            [ '599.5', 'f', lambda self : self._599_5 ],
            [ '606.5', 'f', lambda self : self._606_5 ],
            ]

    @classmethod
    def generate_format(cls):
        """generates a struct.struct format string based on fields()"""
        format = '<B'
        for f in cls.fields():
            format += f[1]
        format_len = struct.calcsize(format)
        if format_len > 64:
            raise ValueError("Too much data (%d > 64)" % (format_len))
        format += str(64-format_len) + "x" # add padding
        return format

    def __init__(self, timestamp, lat, lon, hdg, pitch, roll, alt, hdop, vdop, nsats, _578_5, _585_5, _592_5, _599_5, _606_5):
        self._timestamp = timestamp
        self._lat = lat
        self._lon = lon
        self._hdg = hdg
        self._pitch = pitch
        self._roll = roll
        self._alt = alt
        self._hdop = hdop
        self._vdop = vdop
        self._nsats = nsats
        self._578_5 = _578_5
        self._585_5 = _585_5
        self._592_5 = _592_5
        self._599_5 = _599_5
        self._606_5 = _606_5

        self.format = self.generate_format()

    def lat(self):
        """return float latitude"""
        return self._lat/1.0e7
    def lon(self):
        """return float longitude"""
        return self._lon/1.0e7

    def pack(self):
        """pack this sample into a sequence of bytes"""
        values = [self.format_ver()]
        for f in self.fields():
            values.append(f[2](self))
        data = bytes(struct.pack(self.format, *values))
        if len(data) > 64:
            raise ValueError("Too much data for data64")
        orddata = [ord(x) for x in data]
        return orddata

    @classmethod
    def unpack(cls,orddata):
        """unpack sample from a sequence of bytes, return sample object"""
        data = ''.join([chr(x) for x in orddata])
        values =  list(struct.unpack(cls.generate_format(), data))
        format_ver = values[0]
        if format_ver != cls.format_ver():
            raise ValueError("rfsurvey format_ver incorrect (%d) vs (%d)" % (format_ver, cls.format_ver()))
            return
        values = values[1:]
        sample = rfsurvey_sample(*values)
        return sample

    def to_csv(self):
        # Time,Lat,Long,NoseHDG,Pitch,Roll,BaroHt,GPSHt,HDOP,VDOP,Satellites,Freq(MHz),578.5,585.5,592.5,599.5,606.5,
        # 10:16:07.606,-33.342409,148.983291,10,-4.05008,0.182496,-0.88,1383.53,0.66,655.35,19,CH_Power(dBm),-55.071,-54.958,-54.513,-55.682,-56.184,
        ts = time.strftime("%H:%M:%S", time.localtime(self._timestamp))
        micros = self._timestamp - int(self._timestamp)
        ts = ts + "." + str(int(micros*1000)) # only want milliseconds
        return ",".join([ts,
                         "%f" % (self._lat/1.0e07),
                         "%f" % (self._lon/1.0e07),
                         "%f" % (self._hdg/100.0),
                         "%f" % self._pitch,
                         "%f" % self._roll,
                         "%f" % (self._alt/100.0),
                         "%f" % self._hdop,
                         "%f" % self._vdop,
                         "%u" % self._nsats,
                         "CH_Power(dBm)",
                         "%f" % self._578_5,
                         "%f" % self._585_5,
                         "%f" % self._592_5,
                         "%f" % self._599_5,
                         "%f" % self._606_5,
       ])

class rfsurvey(mp_module.MPModule):
    """script to transfer sample data to a GCS via DATA packets"""

    def __init__(self, mpstate):
        """Initialise module"""
        super(rfsurvey, self).__init__(mpstate, "rfsurvey", "encapsulation of RF survey messages")

        self.mpstate = mpstate
        self.rfsurvey_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),
              ('csv', bool, True),
        ])
        self.add_command('rfsurvey', self.cmd_rfsurvey, "RF Survey telemetry", ['status','start','stop','set (RFSURVEYSETTING)'])
        self.add_completion_function('(RFSURVEYSETTING)', self.rfsurvey_settings.completion)

        self.csv_fh = None
        self.data64_type = 57
        self.send_queue = Queue.Queue()
        self.receive_queue = Queue.Queue()
        atexit.register(self.terminate_threads)
        self.sample_create_thread_should_quit = False
        self.sample_create_thread = None
        self.sample_handle_thread_should_quit = False
        self.sample_handle_thread = None

        self.last_GLOBAL_POSITION_INT_time = None
        self.GLOBAL_POSITION_INT_maxage = 1 # seconds
        self.position_notify_interval = 10 # seconds
        self.position_notify_timestamp = 0

        self.last_GPS_RAW_INT_time = None
        self.GPS_RAW_INT_maxage = 1 # seconds

        self.last_ATTITUDE_time = None
        self.ATTITUDE_maxage = 1 # seconds

    def terminate_create_thread(self):
        """stop the thread responsible for creating samples"""
        self.close_csv_fh()
        self.sample_create_thread_should_quit = True
        if self.sample_create_thread is not None:
            self.sample_create_thread.join()
            self.sample_create_thread = None

    def terminate_handle_thread(self):
        """stop the thread responsible for consuming samples on the GCS"""
        self.close_csv_fh()
        self.sample_handle_thread_should_quit = True
        if self.sample_handle_thread is not None:
            self.sample_handle_thread.join()
            self.sample_handle_thread = None

    def terminate_threads(self):
        self.debug("Terminating threads")
        self.terminate_create_thread()
        self.terminate_handle_thread()

    def usage(self):
        """show help on a command line options"""
        return "Usage: rfsurvey <status|send|receive|stop|set>"

    def position_is_good(self):
        """returns true if we reasonably know our position"""
        if self.last_GLOBAL_POSITION_INT_time is None:
            return False;
        if time.time()-self.last_GLOBAL_POSITION_INT_time > self.GLOBAL_POSITION_INT_maxage:
            return False;
        if self.last_GPS_RAW_INT_time is None:
            return False;
        if time.time()-self.last_GPS_RAW_INT_time > self.GPS_RAW_INT_maxage:
            return False;
        if self.last_ATTITUDE_time is None:
            return False;
        if time.time()-self.last_ATTITUDE_time > self.ATTITUDE_maxage:
            return False;
        return True

    def status(self):
        """returns string representing module status"""
        if self.sample_handle_thread:
            return "Receiving";
        elif self.sample_create_thread:
            if not self.position_is_good():
                return "Sending (position bad)";
            return "Sending"
        else:
            return "Inactive"

    def start_send(self):
        """put module into send mode, where samples are created and sent via mavlink"""
        self.terminate_handle_thread()

        if self.rfsurvey_settings.csv:
            self.open_new_csv_fh()

        def sample_create_thread_target():
            while not self.sample_create_thread_should_quit:
                now = time.time()
                if not self.position_is_good():
                    # no position!  Can't create samples right now
                    if now - self.position_notify_timestamp > self.position_notify_interval:
                        self.position_notify_timestamp = now
                        self.say("No position; can't create samples")
                        time.sleep(1)
                    continue

                sample = rfsurvey_sample(now,
                                         self.last_GLOBAL_POSITION_INT.lat,
                                         self.last_GLOBAL_POSITION_INT.lon,
                                         self.last_GLOBAL_POSITION_INT.hdg,
                                         math.degrees(self.last_ATTITUDE.pitch),
                                         math.degrees(self.last_ATTITUDE.roll),
                                         self.last_GLOBAL_POSITION_INT.alt,
                                         self.last_GPS_RAW_INT.eph,
                                         self.last_GPS_RAW_INT.epv,
                                         self.last_GPS_RAW_INT.satellites_visible,
                                         random.random(),
                                         random.random(),
                                         random.random(),
                                         random.random(),
                                         random.random())
                self.debug("Sample going in with value of %f" % (sample._578_5))
                self.send_queue.put(sample)
                if self.csv_fh is not None:
                    self.csv_fh.write(sample.to_csv() + "\n")
                time.sleep(10)

        self.sample_create_thread_should_quit = False
        self.sample_create_thread = threading.Thread(target=sample_create_thread_target)
        self.sample_create_thread.start()

    def start_receive(self):
        """put module into receive mode, where samples are received via mavlink and decoded"""
        if self.rfsurvey_settings.csv:
            self.open_new_csv_fh()

        self.terminate_create_thread()
        self.sample_handle_thread_should_quit = False
        def sample_handle_thread_target():
            while not self.sample_handle_thread_should_quit:
                try:
                    sample = self.receive_queue.get(timeout=0.1)
                except Queue.Empty:
                    continue
                self.debug("Received sample (lat=%f) (lon=%f) with value %f" % (sample.lat(), sample.lon(), sample._578_5))

        self.sample_handle_thread = threading.Thread(target=sample_handle_thread_target)
        self.sample_handle_thread.start()


    def stop(self):
        """stop module sending or receiving"""
        self.terminate_threads()

    def cmd_rfsurvey(self, args):
        """control behaviour of the module"""
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "status":
            print(self.status())
        elif args[0] == "send":
            self.start_send()
        elif args[0] == "receive":
            self.start_receive()
        elif args[0] == "set":
            self.rfsurvey_settings.command(args[1:])
        elif args[0] == "stop":
            self.stop()
        else:
            print(self.usage())

    def idle_handle_new_samples(self):
        """take samples from queue, encode and send them"""
        try:
            sample = self.send_queue.get(block=0)
        except Queue.Empty:
            return

        orddata = sample.pack()

        try:
            self.master.mav.data64_send(self.data64_type, len(orddata), orddata)
            for m in self.mpstate.mav_outputs:
                m.mav.data64_send(self.data64_type, len(orddata), orddata)
        except Exception as e:
            print("Got exception: %s" % str(e))
            print(traceback.format_exc(e))
            return

    def close_csv_fh(self):
        self.csv_fh = None

    def open_new_csv_fh(self):
        filename = "rfsurvey-%s.csv" % (time.strftime("%Y%m%d%H%M%S", time.localtime()),)
        self.debug("Opening CSV (%s)" % (filename,))
        bufsize = 0
        self.csv_fh = io.open(filename, "wb", bufsize)
        self.csv_fh.write("Time,Lat,Long,NoseHDG,Pitch,Roll,Alt,GPSHt,HDOP,VDOP,Satellites,Freq(MHz),578.5,585.5,592.5,599.5,606.5\n")

    def idle_handle_csv(self):
        if (self.rfsurvey_settings.csv and
            (self.sample_create_thread is not None or
             self.sample_handle_thread is not None)):
            if self.csv_fh is None:
                self.open_new_csv_fh()
        else:
            self.csv_fh = None

    def idle_task(self):
        """called rapidly by mavproxy"""
        self.idle_handle_csv()
        self.idle_handle_new_samples()

    def debug(self,msg):
        """print a message tagged with our name"""
        print("%s: %s" % (self.name, msg))

    def handle_received_data64(self, m):
        """decode a data64 message and add sample to receive queue"""
        if self.sample_handle_thread is None:
            return
        if m.type != self.data64_type:
            if self.rfsurvey_settings.verbose:
                self.debug("data64 received with wrong data64_type")
            return
        data = m.data
        sample = rfsurvey_sample.unpack(data)
        self.receive_queue.put(sample)
        if self.csv_fh is not None:
            self.csv_fh.write(sample.to_csv() + "\n")

    def mavlink_packet(self, m):
        """handle mavlink packets"""
        if m.get_type() == 'GLOBAL_POSITION_INT':
                self.last_GLOBAL_POSITION_INT_time = time.time()
                self.last_GLOBAL_POSITION_INT = m
        elif m.get_type() == 'GPS_RAW_INT':
                self.last_GPS_RAW_INT_time = time.time()
                self.last_GPS_RAW_INT = m
        elif m.get_type() == 'ATTITUDE':
                self.last_ATTITUDE_time = time.time()
                self.last_ATTITUDE = m
        elif m.get_type() == 'DATA64':
            self.handle_received_data64(m)

def unload(self):
    """unload module"""
    self.terminate_threads()

def init(mpstate):
    """initialise module"""
    return rfsurvey(mpstate)
#!/usr/bin/env python3
'''
silvus radio support
To use add this in mavinit.scr for vehicle:

 module load silvus
 silvus set gnd_ip 172.20.167.5
 silvus set air_ip 172.20.168.66
 silvus set nmea_ip 172.20.167.5
 silvus set nmea_port 11234 # UDP port for NMEA input to radio

thanks to Felix from Amber Technologies for the code this came from

AP_FLAKE8_CLEAN
'''

import time
import socket
import requests
import threading
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings


class SilvusModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SilvusModule, self).__init__(mpstate, "Silvus", "Silvus output")
        # filter_dist is distance in metres
        self.silvus_settings = mp_settings.MPSettings([("gnd_ip", str, ""),
                                                       ("gnd_port", int, None),
                                                       ("air_ip", str, ""),
                                                       ("air_port", int, None),
                                                       ("air_node", int, 0),
                                                       ("gnd_node", int, 0),
                                                       ("nmea_ip", str, ""),
                                                       ("nmea_port", int, -1),
                                                       ('log_dt', float, 1.0),
                                                       ('debug', int, 0),
                                                       ])
        self.add_completion_function('(SILVUSSETTING)',
                                     self.silvus_settings.completion)
        self.add_command('silvus', self.cmd_silvus, "silvus control",
                         ["status", "set (SILVUSSETTING)"])
        self.last_nmea_send = time.time()
        self.last_log_time = time.time()

        self.thread = threading.Thread(target=self.thread_loop)
        self.thread.start()
        self.values = {}

    def cmd_silvus(self, args):
        '''silvus commands'''
        if len(args) == 0:
            print("silvus [set|status]")
            return
        if args[0] == "set":
            self.silvus_settings.command(args[1:])
        elif args[0] == "status":
            self.cmd_status()

    def nmea_checkstr(self, msg):
        d = msg[1:]
        cs = 0
        for i in d:
            cs ^= ord(i)
        return "*%02X\r\n" % cs

    def send_nmea(self):
        '''send a NMEA packet to a radio, so the radio knows its position for logging and display'''

        if not self.silvus_settings.nmea_ip or self.silvus_settings.nmea_port <= 0:
            return

        now_time = time.time()
        if now_time - self.last_nmea_send < 1.0:
            return
        self.last_nmea_send = now_time

        gps = self.master.messages.get('GPS_RAW_INT', None)
        if gps is None or gps.fix_type < 3:
            return

        lat = gps.lat * 1.0e-7
        lon = gps.lon * 1.0e-7
        alt = gps.alt * 1.0e-3
        nsat = gps.satellites_visible
        hdop = gps.eph/100.0
        fix = gps.fix_type
        speed = ((gps.vel/100.0)/1852.0)*3600 # knots
        course = gps.cog/100.0

        output = ""
        utc_sec = now_time
        tm_t = time.gmtime(utc_sec)
        dstr = "%02d%02d%02d" % (tm_t.tm_mday, tm_t.tm_mon, tm_t.tm_year % 100)
        subsecs = utc_sec - int(utc_sec)
        tstr = "%02d%02d%05.3f" % (tm_t.tm_hour, tm_t.tm_min, tm_t.tm_sec + subsecs)
        deg = abs(lat)
        minutes = (deg - int(deg))*60
        latstr = "%02d%08.5f,%c" % (int(deg), minutes, 'S' if lat < 0 else 'N')
        deg = abs(lon)
        minutes = (deg - int(deg))*60
        lonstr = "%03d%08.5f,%c" % (int(deg), minutes, 'W' if lon < 0 else 'E')

        gga = "$GPGGA,%s,%s,%s,%01d,%02d,%04.1f,%07.2f,M,0.0,M,," % (tstr, latstr, lonstr, fix, nsat, hdop, alt)
        output = output + gga + self.nmea_checkstr(gga)

        rmc = "$GPRMC,%s,%s,%s,%s,%.2f,%.2f,%s,," % (tstr, fix, latstr, lonstr, speed, course, dstr)
        output = output + rmc + self.nmea_checkstr(rmc)

        if self.silvus_settings.debug > 1:
            print("NMEA: %s" % output)

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            sock.sendto(output.encode("UTF-8"), (self.silvus_settings.nmea_ip, self.silvus_settings.nmea_port))
        except Exception as ex:
            print("Silvus NMEA send fail: %s" % ex)

    def url(self, nodeip, api, port=None):
        host = nodeip
        if port is not None:
            host = ":".join([host, str(port)])
        return "http://%s/%s" % (host, api)

    # Get Frequency == freq (nodeip)
    def get_freq(self, radio):
        data = '{"jsonrpc":"2.0","method":"freq","id":"sbkb5u0c"}'
        response = self.make_request(radio, post_data=data)
        freql = (response.json()["result"])
        freq = int(freql[0])
        return freq

    # Get NoiseLevel == noise(nodeip)
    def get_noise(self, radio):
        data = '{"jsonrpc":"2.0","method":"noise_level","id":"sbkb5u0c"}'
        response = self.make_request(radio, post_data=data)
        noisel = (response.json()["result"])
        return int(noisel[0])

    # Get Neighbor RSSI == nbr_rssi(nodeip, localnode)
    def get_rssi(self, local, remote):
        data = '{"jsonrpc":"2.0","method":"nbr_rssi","params":["' + remote.node + '"],"id":"sbkb5u0c"}'
        response = self.make_request(local, post_data=data)
        nbr_rssi = (response.json()["result"])
        return nbr_rssi

    # Get network Status, output as an array
    def network_status(self, radio):
        data = '{"jsonrpc":"2.0","method":"network_status","id":"sbkb5u0c"}'
        response = self.make_request(radio, post_data=data)
        netstat = (response.json()["result"])
        return netstat

    # Get GPS status
    def get_gps_state(self, radio):
        data = '{"jsonrpc":"2.0","method":"gps_mode","id":"sbkb5u0c"}'
        response = self.make_request(radio, post_data=data)
        gpsstat = (response.json()["result"])
        return gpsstat

    # Get max throughput between nodes
    def get_throughput(self, local, remote):
        data = '{"jsonrpc":"2.0","method":"link_throughput","params":["' + remote.node + '", "1"],"id":"sbkb5u0c"}'
        response = self.make_request(local, post_data=data)
        nbr_tp = (response.json()["result"])
        nbr_tp = (nbr_tp)[0]
        return nbr_tp

    # Returns the TX MCS
    def get_neighbor_mcs(self, local, remote):
        data = '{"jsonrpc":"2.0","method":"nbr_mcs","params":["' + remote.node + '"],"id":"sbkb5u0c"}'
        response = self.make_request(local, post_data=data)
        nbr_mcs = (response.json()["result"])
        nbr_mcs = (nbr_mcs)[0]
        # print(data)
        return nbr_mcs

    def make_request(self, radio, post_data):
        print(f"Making request to {radio} {post_data} {radio.port}")
        uri = self.url(radio.ip, 'streamscape_api', port=radio.port)
        try:
            result = requests.post(uri, data=post_data)
        except Exception:  # FIXME: narrow this exception
            return None
        return result

    # Returns the RX MCS
    def get_neighbor_mcs_rx(self, local, remote):
        data = '{"jsonrpc":"2.0","method":"nbr_mcs_rx","params":["' + remote.node + '"],"id":"sbkb5u0c"}'
        response = self.make_request(local, data=data)
        nbr_mcs_rx = (response.json()["result"])
        nbr_mcs_rx = (nbr_mcs_rx)[0]
        # print(data)
        return nbr_mcs_rx

    # Get GPS coords, output as an array
    def get_gps_coords(self, radio):
        data = '{"jsonrpc":"2.0","method":"gps_coordinates","id":"sbkb5u0c"}'
        response = self.make_request(radio, post_data=data)
        coords = (response.json()["result"])
        return coords

    def get_radio_data(self):
        now = time.time()
        if now - self.last_log_time < self.silvus_settings.log_dt:
            return
        self.last_log_time = now

        localip = self.silvus_settings.gnd_ip
        localport = self.silvus_settings.gnd_port
        remoteip = self.silvus_settings.air_ip
        remoteport = self.silvus_settings.air_port

        if len(localip.split('.')) != 4:
            return
        if len(remoteip.split('.')) != 4:
            return
        if localport <= 0:
            return
        if remoteport <= 0:
            return

        class Radio():
            def __init__(self, remote_ip, remote_port, remote_node):
                self.ip = remote_ip
                self.port = remote_port
                self.node = remote_node

        localnode = str(self.silvus_settings.gnd_node)
        remotenode = str(self.silvus_settings.air_node)

        remote = Radio(remoteip, remoteport, remotenode)
        local = Radio(localip, localport, localnode)

        try:
            self.values['TXMCS'] = float(self.get_neighbor_mcs(local, remote))
        except Exception:
            pass
        try:
            self.values['RXMCS'] = float(self.get_neighbor_mcs_rx(local, remote))
        except Exception:
            pass
        try:
            rssi = self.get_rssi(local, remote)
            if len(rssi) >= 4:
                self.values['TXRSSI1'] = float(rssi[0])
                self.values['TXRSSI2'] = float(rssi[1])
                self.values['TXRSSI3'] = float(rssi[2])
                self.values['TXRSSI4'] = float(rssi[3])
        except Exception:
            pass

        try:
            rssi = self.get_rssi(remote, local)
            if len(rssi) >= 4:
                self.values['RXRSSI1'] = float(rssi[0])
                self.values['RXRSSI2'] = float(rssi[1])
                self.values['RXRSSI3'] = float(rssi[2])
                self.values['RXRSSI4'] = float(rssi[3])
        except Exception:
            pass

        try:
            self.values['LOCNSE'] = float(self.get_noise(local))
        except Exception:
            pass
        try:
            self.values['REMNSE'] = float(self.get_noise(remote))
        except Exception:
            pass
        try:
            self.values['LINKSNR'] = float(self.network_status(local)[2])
        except Exception:
            pass

        try:
            self.values['LOCTPUT'] = float(self.get_throughput(local, remote))
        except Exception:
            pass
        try:
            self.values['REMTPUT'] = float(self.get_throughput(remote, local))
        except Exception:
            pass

        for f in self.values:
            self.send_named_float('SR_' + f, self.values[f])

    def cmd_status(self):
        for f in sorted(self.values.keys()):
            print("%20s %.1f" % (f, self.values[f]))

    def thread_loop(self):
        while True:
            time.sleep(self.silvus_settings.log_dt)
            try:
                self.send_nmea()
                self.get_radio_data()
            except Exception as ex:
                if self.silvus_settings.debug > 0:
                    print(ex)
                if self.silvus_settings.debug > 1:
                    print(self.get_exception_stacktrace(ex))


def init(mpstate):
    '''initialise module'''
    return SilvusModule(mpstate)

#!/usr/bin/env python

from __future__ import print_function

'''firmware handling'''

'''Currently handles firmware downloading but not flashing'''

import time, os, fnmatch
import json
import threading

from pymavlink import mavutil, mavparm
from MAVProxy.modules.lib import mp_util

from MAVProxy.modules.lib import mp_module

class FirmwareModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(FirmwareModule, self).__init__(mpstate, "firmware", "firmware handling", public = True)
        self.add_command('fw', self.cmd_fw, "firmware handling",
                         ["<manifest> (OPT)",
                          "list <filterterm...>"])
        self.downloaders_lock = threading.Lock()
        self.downloaders = {}
        self.manifests_parse()

    def usage(self):
        '''show help on a command line options'''
        return "Usage: fw <manifest|list>"

    def cmd_fw_help(self):
        '''show help on fw command'''
        print(self.usage())

    def cmd_fw(self, args):
        '''execute command defined in args'''
        if len(args) == 0:
            print(self.usage())
            return
        rest = args[1:]
        if args[0] == "manifest":
            self.cmd_fw_manifest(rest)
        elif args[0] == "list":
            self.cmd_fw_list(rest)
        elif args[0] == "download":
            self.cmd_fw_download(rest)
        elif args[0] in ["help","usage"]:
            self.cmd_fw_help(rest)
        else:
            print(self.usage())

    def frame_from_firmware(self, firmware):
        '''extract information from firmware, return pretty string to user'''
        # see Tools/scripts/generate-manifest for this map:
        frame_to_mavlink_dict = {
            "quad": "QUADROTOR",
            "hexa": "HEXAROTOR",
            "y6": "ARDUPILOT_Y6",
            "tri": "TRICOPTER",
            "octa": "OCTOROTOR",
            "octa-quad": "ARDUPILOT_OCTAQUAD",
            "heli": "HELICOPTER",
            "Plane": "FIXED_WING",
            "Tracker": "ANTENNA_TRACKER",
            "Rover": "GROUND_ROVER",
            "PX4IO": "ARDUPILOT_PX4IO",
        }
        mavlink_to_frame_dict = { v : k  for k,v in frame_to_mavlink_dict.items() }
        x = firmware["mav-type"]
        if firmware["mav-autopilot"] != "ARDUPILOTMEGA":
            return x
        if x in mavlink_to_frame_dict:
            return mavlink_to_frame_dict[x]

        return x

    def row_is_filtered(self, row_subs, filters):
        '''returns True if row should NOT be included according to filters'''
        for filtername in filters:
            filtervalue = filters[filtername]
            if filtername in row_subs:
                row_subs_value = row_subs[filtername]
                if str(row_subs_value) != str(filtervalue):
                    return True
            else:
                print("Unknown filter keyword (%s)" % (filtername,))
        return False

    def filters_from_args(self, args):
        '''take any argument of the form name=value anmd put it into a dict; return that and the remaining arguments'''
        filters = dict()
        remainder = []
        for arg in args:
            equals = arg.index('=')
            if equals != 1:
                # anything ofthe form key-value is taken as a filter
                filters[arg[0:equals]] = arg[equals+1:];
            else:
                remainder.append(arg)
        return (filters,remainder)

    def all_firmwares(self):
        ''' return firmware entries from all manifests'''
        all = []
        for manifest in self.manifests:
            for firmware in manifest["firmware"]:
                all.append(firmware)
        return all

    def rows_for_firmwares(self, firmwares):
        '''provide user-readable text for a firmware entry'''
        rows = []
        i = 0
        for firmware in firmwares:
            frame = self.frame_from_firmware(firmware)
            row = {
                "seq": i,
                "platform": firmware["platform"],
                "frame": frame,
#                "type": firmware["mav-type"],
                "releasetype": firmware["mav-firmware-version-type"],
                "latest": firmware["latest"],
                "git-sha": firmware["git-sha"][0:7],
                "format": firmware["format"],
                "_firmware": firmware,
            }
            i += 1
            rows.append(row)
            
        return rows

    def filter_rows(self, filters, rows):
        '''returns rows as filtered by filters'''
        ret = []
        for row in rows:
            if not self.row_is_filtered(row, filters):
                ret.append(row)
        return ret

    def filtered_rows_from_args(self, args):
        '''extracts filters from args, rows from manifests, returns filtered rows'''
        (filters,remainder) = self.filters_from_args(args)

        if len(self.manifests) == 0:
            print("No manifests downloaded.  Try 'manifest download'")
            return None
        all = self.all_firmwares()
        rows = self.rows_for_firmwares(all)
        filtered = self.filter_rows(filters, rows)
        return (filtered, remainder)

    def cmd_fw_list(self, args):
        '''cmd handler for list'''
        stuff = self.filtered_rows_from_args(args)
        if stuff is None:
            return
        (filtered, remainder) = stuff
        print("")
        print(" seq platform frame    releasetype latest git-sha format")
        for row in filtered:
            print("{seq:>4} {platform:<8} {frame:<10} {releasetype:<9} {latest:<6} {git-sha} {format}".format(**row))

    def cmd_fw_download(self, args):
        '''cmd handler for downloading firmware'''
        import multiprocessing
        stuff = self.filtered_rows_from_args(args)
        if stuff is None:
            return
        (filtered, remainder) = stuff
        if len(filtered) == 0:
            print("No firmware specified")
            return
        if len(filtered) > 1:
            print("No single firmware specified")
            return

        firmware = filtered[0]["_firmware"]
        url = firmware["url"]

        try:
            print("URL: %s"  % (url,))
            filename=os.path.basename(url)
            files = []
            files.append((url,filename))
            child = multiprocessing.Process(target=mp_util.download_files, args=(files,))
            child.start()
        except Exception as e:
            print("Firmware download failed")
            print(e)

    def fw_manifest_usage(self):
        '''return help on manifest subcommand'''
        return("Usage: fw manifest <list|download>")

    def cmd_fw_manifest_help(self):
        '''show help on manifest subcommand'''
        print(self.fw_manifest_usage())

    def find_manifests(self):
        '''locate manifests and return filepaths thereof'''
        return ['/home/pbarker/manifest.json']

        manifest_dir = mp_util.dot_mavproxy()

        ret = []
        for file in listdir(manifest_dir):
            if file.index("manifest") == -1:
                continue
            ret.append(file)
        return ret

    def cmd_fw_manifest_list(self):
        for filepath in self.find_manifests():
            print(filepath)

    def cmd_fw_manifest_purge(self):
        for filepath in self.find_manifests():
            os.path.unlink(filepath)

    def cmd_fw_manifest(self, args):
        '''cmd handler for manipulating manifests'''
        if len(args) == 0:
            print(self.fw_manifest_usage())
            return
        rest = args[1:]
        if args[0] == "download":
            return self.manifest_download()
        if args[0] == "list":
            return self.cmd_fw_manifest_list()
        if args[0] == "purge":
            return self.cmd_fw_manifest_purge()
        if args[0] == "help":
            return self.cmd_fw_manifest_help()
        else:
            print("Unknown manifest option (%s)" % args[0])
            print(fw_manifest_usage())

    def manifest_parse(self, path):
        '''parse manifest at path, return JSON object'''
        content = open(path).read()
        return json.loads(content)

    def semver_major(self,semver):
        '''return major part of semver version number. Avoids "import semver"'''
        return int(semver[0:semver.index(".")])

    def manifest_path_is_old(self, path):
        mtime = os.path.getmtime(path)
        return (time.time() - mtime) > 24*60*60

    def manifests_parse(self):
        '''parse manifests present on system'''
        self.manifests = []
        for manifest_path in self.find_manifests():
            if self.manifest_path_is_old(manifest_path):
                print("Manifest (%s) is old; consider 'manifest download'" % (manifest_path))
            manifest = self.manifest_parse(manifest_path)
            if self.semver_major(manifest["format-version"]) != 1:
                print("Manifest (%s) has major version %d; MAVProxy only understands version 1" % (manifest_path,manifest["format-version"]))
                continue
            self.manifests.append(manifest)

    def download_url(self, url, path):
        mp_util.download_files([(url,path)])

    def idle_task(self):
        '''called rapidly by mavproxy'''
        if self.downloaders_lock.acquire(False):
            for url in self.downloaders.keys():
                if not self.downloaders[url].is_alive():
                    print("Download thread for (%s) done" % url)
                    del self.downloaders[url]
            self.downloaders_lock.release()

    def manifest_download(self):
        '''download manifest files'''
        import multiprocessing
        if self.downloaders_lock.acquire(False):
            if len(self.downloaders):
                # there already exist downloader threads
                self.downloaders_lock.release()
                return

            for url in ['http://firmware.diydrones.com/manifest.json']:
                path = mp_util.dot_mavproxy("manifest-%s" % url)
                self.downloaders[url] = threading.Thread(target=self.download_url, args=(url, path))
                self.downloaders[url].start()
            self.downloaders_lock.release()
        else:
            print("Failed to acquire download lock")

def init(mpstate):
    '''initialise module'''
    return FirmwareModule(mpstate)

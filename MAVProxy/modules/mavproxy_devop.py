#!/usr/bin/env python
'''remote low level device operations'''

# read first 16 bytes from device@0x55 on bus 0 (Toshiba LED on PixHawk):
# devop i2c read 0 0x55 0 16

# read first 16 bytes from device@0x1e on bus 1:
# devop i2c read 1 0x1e 0x0 16

# see ToshibaLED_I2C.cpp....
# disable Toshiba LED on PixHawk 1:
# devop i2c write 0 0x55 0x04 1 0
# enable Toshiba LED on PixHawk 1 (write 1 byte (0x03) @0x04 to device 0x55 on bus 0):
# devop i2c write 0 0x55 0x04 1 0x3

# FIXME: this really should be "devop spi read mpu6000 0x75 1"
# read register at 0x75 on mpu6000 bus (should be ox68 on a PixHawk):
# devop spi read mpu6000 0xf5 1


import time, os, sys
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module

class Ops(object):
    def __init__(self, devop):
        self.devop = devop

    def debug(self,message):
        self.devop.debug("DEVOP: " + message)

class OpsSPI(Ops):
    def __init__(self, devop):
        super(OpsSPI, self).__init__(devop)

    def cmd(self, args):
        usage = "Usage: devop spi <read|write> SPINAME ADDRESS REG COUNT"
        if len(args) < 1:
            print(usage)
            return
        if args[0] == 'read':
            self.cmd_read(args[1:])
        elif args[0] == 'write':
            self.cmd_write(args[1:])
        else:
            print(usage)

    def cmd_read(self, args):
        usage = "Usage: devop spi read SPINAME REG COUNT"
        if len(args) < 3:
            print(usage)
            return
        bustype = mavutil.mavlink.DEVICE_OP_BUSTYPE_SPI
        spiname = args[0]
        i2cbus = 0 # unused
        i2caddr = 0 # unused
        self.devop.devop_read_send(bustype, spiname, i2cbus,i2caddr, usage, args[1:])

    def cmd_write(self, args):
        usage = "Usage: devop spi write SPINAME REG COUNT [BYTE ...]"
        if len(args) < 4:
            print(usage)
            return
        bustype = mavutil.mavlink.DEVICE_OP_BUSTYPE_SPI
        name = args[0]
        i2cbus = 0 # unused
        i2caddr = 0 # unused
        self.devop.devop_write_send(bustype, spiname, i2cbus,i2caddr, usage, args[1:])

class OpsI2c(Ops):
    def __init__(self, devop):
        super(OpsI2c, self).__init__(devop)
        self.scan_probe_timeout = 1 # seconds before resending probe
        self.scan_request_id = None
        self.scan_address = None
        self.scan_last_probe_sent = time.time()

    def idle_task(self):
        self.scan_idle_task()

    def cmd(self, args):
        usage = "Usage: devop i2c <read|write|scan> I2CBUS I2CADDR REG COUNT [BYTE ...]"
        if len(args) < 1:
            print(usage)
            return;
        if args[0] == 'read':
            self.cmd_read(args[1:])
        elif args[0] == 'write':
            self.cmd_write(args[1:])
        elif args[0] == 'scan':
            self.cmd_scan(args[1:])
        else:
            print(usage)

    def cmd_read(self, args):
        usage = "Usage: devop i2c read BUS ADDRESS REG COUNT"
        if len(args) < 4:
            print(usage)
            return
        bustype = mavutil.mavlink.DEVICE_OP_BUSTYPE_I2C
        spiname = "BOB" # unused
        i2cbus = int(args[0],base=0)
        i2caddr = int(args[1],base=0)
        self.devop.devop_read_send(bustype, spiname, i2cbus,i2caddr, usage, args[2:])

    def cmd_write(self, args):
        usage = "Usage: devop i2c write I2CBUS I2CADDR REG COUNT [BYTE ...]"
        print("args: " + str (args))
        if len(args) < 4:
            print(usage)
            return
        bustype = mavutil.mavlink.DEVICE_OP_BUSTYPE_I2C
        spiname = "BOB" # unused
        i2cbus = int(args[0],base=0)
        i2caddr = int(args[1],base=0)
        self.devop.devop_write_send(bustype, spiname, i2cbus,i2caddr, usage, args[2:])

    def cmd_scan(self, args):
        '''scan for devices'''
        usage = "Usage: devop i2c scan BUS [FIRST LAST]"
        if len(args) < 1:
            print(usage)
            return
        self.scan_bus = int(args[0],base=0)
        # See https://www.i2c-bus.org/addressing/ for valid addresses
        first = 16
        last = 119
        if (len(args) == 3):
            start = int(args[1],base=0)
            stop = int(args[2],base=0)
        self.scan_pending = range(first,last+2,2)
        self.scan_bustype = mavutil.mavlink.DEVICE_OP_BUSTYPE_I2C
        self.scan_name = "bob" # unused
        self.scan_results = []
        self.scan_next_address()
        self.scan_count = 0
        self.scan_attempts = 2 # max times to probe

    def scan_do_probe(self, address):
        self.debug("Sending probe bus=0x%02x addr=%02x" % (self.scan_bus, address))
        self.scan_address = address
        self.devop.master.mav.device_op_read_send(self.devop.target_system,
                                                  self.devop.target_component,
                                                  self.devop.request_id,
                                                  self.scan_bustype,
                                                  self.scan_bus,
                                                  address,
                                                  self.scan_name,
                                                  0x0,
                                                  1)
        self.scan_request_id = self.devop.request_id
        self.devop.request_id += 1
        self.scan_last_probe_sent = time.time()

    def scan_idle_task(self):
        '''check to see if we should re-send the probe'''
        if self.scan_address is None:
            return
        now = time.time()
        if now - self.scan_last_probe_sent > self.scan_probe_timeout:
            if self.scan_count < self.scan_attempts:
                self.debug("Giving up on %02x" % self.scan_address)
                self.scan_next_address()
            else:
                self.debug("Rescanning %02x" % self.scan_address)
                self.scan_do_probe(self.scan_address)
                self.scan_count+=1

    def scan_print_summary(self):
        print("DEVOP: scan summary:")
        for result in self.scan_results:
            (addr,result) = result
            print("DEVOP: 0x%02x: %d" % (addr, result))

    def scan_next_address(self):
        self.scan_count = 0
        if not len(self.scan_pending):
            self.scan_request_id = None
            self.scan_address = None
            self.scan_print_summary()
            return
        self.scan_do_probe(self.scan_pending[0])
        self.scan_pending = self.scan_pending[1:]

    def scan_handle_reply(self, m):
        if m.request_id != self.scan_request_id:
            return False

        self.debug("Scan (0x%02x) reply: %d" % (self.scan_address, m.result))
        if m.result != 4:
            self.scan_results.append( (self.scan_address, m.result) )
        self.scan_next_address()
        return True


class DeviceOpModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(DeviceOpModule, self).__init__(mpstate, "DeviceOp")
        self.add_command('devop', self.cmd_devop, "device operations",
                         ["<spi|i2c> <read|write|scan>"])
        self.i2c = OpsI2c(self)
        self.spi = OpsSPI(self)
        self.request_id = 1

    def cmd_devop(self, args):
        '''device operations'''
        usage = "Usage: devop <spi|i2c> <read|write|scan> <NAME|BUS> [ADDRESS]"
        if len(args) < 1:
            print(usage)
            return

        if args[0] == 'spi':
            self.spi.cmd(args[1:])
        elif args[0] == 'i2c':
            self.i2c.cmd(args[1:])
        else:
            print(usage)

    # shared functions
    def devop_read_send(self, bustype, spiname, i2cbus,i2caddr, usage, args):
        '''read from device'''
        if len(args) != 2:
            print(usage)
            return
        reg = int(args[0],base=0)
        count = int(args[1],base=0)
        self.master.mav.device_op_read_send(self.target_system,
                                            self.target_component,
                                            self.request_id,
                                            bustype,
                                            i2cbus,
                                            i2caddr,
                                            spiname,
                                            reg,
                                            count)
        self.request_id += 1

    def devop_write_send(self, bustype, spiname, i2cbus,i2caddr, usage, args):
        '''write to a device'''
        if len(args) < 2:
            print(usage)
            return
        reg = int(args[0],base=0)
        count = int(args[1],base=0)
        args = args[2:]
        if len(args) < count:
            print("Error: number of bytes supplied < count")
            print(usage)
            return
        bytes = [0]*128
        for i in range(count):
            bytes[i] = int(args[i],base=0)
        self.master.mav.device_op_write_send(self.target_system,
                                             self.target_component,
                                             self.request_id,
                                             bustype,
                                             i2cbus,
                                             i2caddr,
                                             spiname,
                                             reg,
                                             count,
                                             bytes)
        self.request_id += 1


    def debug(self,message):
        print("DEVOP: " + message)

    def idle_task(self):
        self.i2c.idle_task()

    def read_show_reply(self, m):
        # header
        sys.stdout.write("     ")
        for i in range(16):
            sys.stdout.write("%3x" % i)
        print("")
        sys.stdout.write("%4x: " % (m.regstart-m.regstart%16,))
        # leading no-data-read:
        for i in range(m.regstart%16):
            sys.stdout.write("-- ")

        for i in range(m.count):
            reg = i + m.regstart
            if m.data[i]:
                sys.stdout.write("%02x " % (m.data[i]))
            else:
                sys.stdout.write("   ")
            if (reg+1) % 16 == 0 and i != m.count-1:
                print("")
                sys.stdout.write("%4x: " % ((reg+1)-(reg+1)%16,))

        # trailing no-data-read
        if (m.regstart+m.count) % 16 != 0:
            if m.count < 16 and m.regstart % 16 != 0 and (m.regstart%16+m.count) <16:
                # front of line is padded
                count = 16 - m.regstart%16 - m.count
            else:
                count = 16 - (m.regstart+m.count)%16

            for i in range(0,count):
                sys.stdout.write("-- ")
        print("")

    def mavlink_packet(self, m):
        '''handle a mavlink packet'''
        mtype = m.get_type()
        if mtype == "DEVICE_OP_READ_REPLY":
            if self.i2c.scan_handle_reply(m):
                return
            if m.result != 0:
                print("Operation %u failed: %u" % (m.request_id, m.result))
            else:
                print("Operation %u OK: %u bytes" % (m.request_id, m.count))
                self.read_show_reply(m)

        if mtype == "DEVICE_OP_WRITE_REPLY":
            if m.result != 0:
                print("Operation %u failed: %u" % (m.request_id, m.result))
            else:
                print("Operation %u OK" % m.request_id)

def init(mpstate):
    '''initialise module'''
    return DeviceOpModule(mpstate)

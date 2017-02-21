#!/usr/bin/env python
'''remote low level device operations'''

# read first 16 bytes from device@0x1e on bus 1
# devop i2c read 1 0x1e 0x0 16

# read register at 0x7f on mpu6000 bus:
# devop spi read mpu6000 0xf5 1


import time, os, sys
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module

class Ops(object):
    def __init__(self, devop):
        self.devop = devop

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
        usage = "Usage: devop spi read SPINAME REGISTER COUNT"
        if len(args) < 3:
            print(usage)
            return
        bustype = mavutil.mavlink.DEVICE_OP_BUSTYPE_SPI
        spiname = args[0]
        i2cbus = 0 # unused
        i2caddr = 0 # unused
        self.devop.devop_read_send(bustype, spiname, i2cbus,i2caddr, usage, args[1:])

    def cmd_write(self, args):
        usage = "Usage: devop spi write SPINAME REGISTER COUNT [BYTE ...]"
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

    def cmd(self, args):
        usage = "Usage: devop i2c <read|write> BUS ADDRESS REG COUNT [BYTE ...]"
        if len(args) < 1:
            print(usage)
            return;
        if args[0] == 'read':
            self.cmd_read(args[1:])
        elif args[0] == 'write':
            self.cmd_write(args[1:])
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
        usage = "Usage: devop i2c write BUS ADDRESS REG COUNT [BYTE ...]"
        if len(args) < 4:
            print(usage)
            return
        bustype = mavutil.mavlink.DEVICE_OP_BUSTYPE_I2C
        spiname = "BOB" # unused
        i2cbus = int(args[0],base=0)
        i2caddr = int(args[1],base=0)
        self.devop.devop_write_send(bustype, spiname, i2cbus,i2caddr, usage, args[1:])



class DeviceOpModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(DeviceOpModule, self).__init__(mpstate, "DeviceOp")
        self.add_command('devop', self.cmd_devop, "device operations",
                         ["<read|write> <spi|i2c>"])
        self.i2c = OpsI2c(self)
        self.spi = OpsSPI(self)
        self.request_id = 1

    def cmd_devop(self, args):
        '''device operations'''
        usage = "Usage: devop <spi|i2c> <read|write> <name|bus> address"
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

    def mavlink_packet(self, m):
        '''handle a mavlink packet'''
        mtype = m.get_type()
        if mtype == "DEVICE_OP_READ_REPLY":
            if m.result != 0:
                print("Operation %u failed: %u" % (m.request_id, m.result))
            else:
                print("Operation %u OK: %u bytes" % (m.request_id, m.count))
                for i in range(m.count):
                    reg = i + m.regstart
                    sys.stdout.write("%02x:%02x " % (reg, m.data[i]))
                    if (i+1) % 16 == 0:
                        print("")
                if m.count % 16 != 0:
                    print("")

        if mtype == "DEVICE_OP_WRITE_REPLY":
            if m.result != 0:
                print("Operation %u failed: %u" % (m.request_id, m.result))
            else:
                print("Operation %u OK" % m.request_id)

def init(mpstate):
    '''initialise module'''
    return DeviceOpModule(mpstate)

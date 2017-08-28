#!/usr/bin/env python

"""
    A Python driver for the Arduino microcontroller running the
    ROSArduinoBridge firmware.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html

"""

import thread
from math import pi as PI, degrees, radians
import ctypes
import os
import time
import sys, traceback
from smbus import SMBus
from arduino_driver import Arduino

class ArduinoSMBus(Arduino):
    def __init__(self, port = 1, device = 0x42):
        self.retry_count = 3
        self.port = port
        self.device = device
        self.bus = None
        self.base_init()

    def connect(self):
        if self.bus != None:
            self.bus.close()

        self.bus = SMBus(self.port)

    def calculate_fletcher16(self, buff):
        s1 = 0
        s2 = 0

        for b in buff:
            s1 += b
            s2 += s1

        ret = (s2 & 0xff) << 8 | (s1 & 0xff)
        # print "calculate_fletcher16 retuns %d" % ret

        return ret


    def handle_exeception(self, e):
        if e.__class__.__name__ == "IOError":
            # print "handle_exeception?!"
            try:
                self.bus.close()
            except Exception as e1:
                print "handle_exeception - close execption " + e1.__class__.__name__
                pass
            try:
                self.bus = SMBus(self.port)
            except Exception as e2:
                print "handle_exeception - open execption " + e2.__class__.__name__
                pass

    def update_pid(self, Kp, Kd, Ki, Ko):
        ''' Set the PID parameters on the Arduino
        '''
        # print "Updating PID parameters"
        self.mutex.acquire()
        retry = self.retry_count
        while retry > 0:
            try:
                Kp_value = ctypes.c_short(Kp).value
                chk = ctypes.c_ushort(self.calculate_fletcher16([ self.device, 0x50, 4, Kp_value & 0xff, (Kp_value >> 8) & 0xff ])).value
                # print "update_pid Kp chk= %x" % chk
                self.bus.write_i2c_block_data(self.device, 0x50, [ 4, Kp_value & 0xff, (Kp_value >> 8) & 0xff, chk & 0xff, (chk >> 8) & 0xff ])

                Kd_value = ctypes.c_short(Kd).value
                chk = ctypes.c_ushort(self.calculate_fletcher16([ self.device, 0x54, 4, Kd_value & 0xff, (Kd_value >> 8) & 0xff ])).value
                # print "update_pid Kd chk= %x" % chk
                self.bus.write_i2c_block_data(self.device, 0x54, [ 4, Kd_value & 0xff, (Kd_value >> 8) & 0xff, chk & 0xff, (chk >> 8) & 0xff ])

                Ki_value = ctypes.c_short(Ki).value
                chk = ctypes.c_ushort(self.calculate_fletcher16([ self.device, 0x52, 4, Ki_value & 0xff, (Ki_value >> 8) & 0xff ])).value
                # print "update_pid Ki chk= %x" % chk
                self.bus.write_i2c_block_data(self.device, 0x52, [ 4, Ki_value & 0xff, (Ki_value >> 8) & 0xff, chk & 0xff, (chk >> 8) & 0xff ])

                Ko_value = ctypes.c_short(Ko).value
                chk = ctypes.c_ushort(self.calculate_fletcher16([ self.device, 0x56, 4, Ko_value & 0xff, (Ko_value >> 8) & 0xff ])).value
                # print "update_pid Ko chk= %x" % chk
                self.bus.write_i2c_block_data(self.device, 0x56, [ 4, Ko_value & 0xff, (Ko_value >> 8) & 0xff, chk & 0xff, (chk >> 8) & 0xff ])

                chk = ctypes.c_ushort(self.calculate_fletcher16([ self.device, 0x40, 3, 0x75 ])).value
                # print "update_pid cmd chk= %x" % chk
                self.bus.write_i2c_block_data(self.device, 0x40, [ 3, 0x75, chk & 0xff, (chk >> 8) & 0xff ])
                retry = 0
            except Exception as e:
                print "update_pid execption " + e.__class__.__name__
                self.handle_exeception(e)
                retry -= 1
                pass
        self.mutex.release()
        return True

    def get_baud(self):
        ''' Get the current baud rate on the serial port.
        '''
        return 0;

    def get_encoder_counts(self):
        self.mutex.acquire()
        retry = self.retry_count * 2
        while retry > 0:
            try:
                l_value_array = self.bus.read_i2c_block_data(self.device, 0x44, 6)
                chk = self.calculate_fletcher16([ self.device, 0x44, 6, l_value_array[0], l_value_array[1], l_value_array[2], l_value_array[3]])
                chk_r = ctypes.c_ushort((l_value_array[5] << 8 | l_value_array[4])).value

                if chk != chk_r:
                    # print "get_encoder_counts chk= %x  chk_r= %x" % (chk, chk_r)
                    raise ValueError('checksum error')
                retry = -1
            except Exception as e:
                # print "test execption " + e.__class__.__name__
                self.handle_exeception(e)
                retry -= 1
                pass
        self.mutex.release()

        if retry == 0:
            raise ValueError('Could not get l_value_array')

        self.mutex.acquire()
        retry = self.retry_count * 2
        while retry > 0:
            try:
                r_value_array = self.bus.read_i2c_block_data(self.device, 0x48, 6)
                chk = self.calculate_fletcher16([ self.device, 0x48, 6, r_value_array[0], r_value_array[1], r_value_array[2], r_value_array[3]])
                chk_r = ctypes.c_ushort((r_value_array[5] << 8 | r_value_array[4])).value
                if chk != chk_r:
                    # print "get_encoder_counts chk= %x  chk_r= %x" % (chk, chk_r)
                    raise ValueError('checksum error')
                retry = -1
            except Exception as e:
                # print "test execption " + e.__class__.__name__
                self.handle_exeception(e)
                retry -= 1
                pass
        self.mutex.release()

        if retry == 0:
            raise ValueError('Could not get r_value_array')

        l_value = ctypes.c_long(l_value_array[3] << 24 | l_value_array[2] << 16 | l_value_array[1] << 8 | l_value_array[0]).value
        r_value = ctypes.c_long(r_value_array[3] << 24 | r_value_array[2] << 16 | r_value_array[1] << 8 | r_value_array[0]).value

        # if l_value != 0 or r_value != 0:
        #     print "get_encoder_counts --> %d:%d" % (l_value, r_value)

        return [ l_value, r_value ]

    def reset_encoders(self):
        ''' Reset the encoder counts to 0
        '''
        self.mutex.acquire()
        retry = self.retry_count
        while retry > 0:
            try:
                chk = ctypes.c_ushort(self.calculate_fletcher16([ self.device, 0x40, 3, 0x72 ])).value
                self.bus.write_i2c_block_data(self.device, 0x40, [ 3, 0x72, chk & 0xff, (chk >> 8) & 0xff ])
                retry = 0
            except Exception as e:
                # print "test execption " + e.__class__.__name__
                self.handle_exeception(e)
                retry -= 1
                pass
        self.mutex.release()
        return True

    def drive(self, left, right):
        ''' Speeds are given in encoder ticks per PID interval
        '''
        # print "drive %d:%d" % (left, right)
        self.mutex.acquire()
        retry = self.retry_count
        while retry > 0:
            if right < 0:
                right = - (abs(right) % 255)
            else:
                right = right % 255

            if left < 0:
                left = - (abs(left) % 255)
            else:
                left = left % 255

            left = int(left)
            right = int(right)

            r_value = ctypes.c_short(right).value
            l_value = ctypes.c_short(left).value
            # print "drive converted %d:%d" % (l_value, r_value)
            try:
                chk = ctypes.c_ushort(self.calculate_fletcher16([ self.device, 0x4c, 4, l_value & 0xff, (l_value >> 8) & 0xff ])).value
                self.bus.write_i2c_block_data(self.device, 0x4c, [ 4, l_value & 0xff, (l_value >> 8) & 0xff, chk & 0xff, (chk >> 8) & 0xff ])

                chk = ctypes.c_ushort(self.calculate_fletcher16([ self.device, 0x4e, 4, r_value & 0xff, (r_value >> 8) & 0xff ])).value
                self.bus.write_i2c_block_data(self.device, 0x4e, [ 4, r_value & 0xff, (r_value >> 8) & 0xff, chk & 0xff, (chk >> 8) & 0xff ])

                chk = ctypes.c_ushort(self.calculate_fletcher16([ self.device, 0x40, 3, 0x6d ])).value
                self.bus.write_i2c_block_data(self.device, 0x40, [ 3, 0x6d, chk & 0xff, (chk >> 8) & 0xff ])
                retry = 0
            except Exception as e:
                print "test execption " + e.__class__.__name__
                self.handle_exeception(e)
                retry -= 1
                pass
        self.mutex.release()
        return True

    def analog_read(self, pin):
        self.mutex.acquire()
        addr = 0x58 + (pin * 2)
        retry = self.retry_count
        while retry > 0:
            try:
                value_array = self.bus.read_i2c_block_data(self.device, addr, 4)
                chk = self.calculate_fletcher16([ self.device, addr, 4, value_array[0], value_array[1]])
                chk_r = ctypes.c_ushort((value_array[3] << 8 | value_array[2])).value

                if chk != chk_r:
                    # print "analog_read(%d) -> %x  chk= %x  chk_r= %x" % (pin, addr, chk, chk_r)
                    raise ValueError('checksum error')
                retry = -1
            except Exception as e:
                # print "test execption " + e.__class__.__name__
                self.handle_exeception(e)
                retry -= 1
                pass
        self.mutex.release()

        if retry == 0:
            raise ValueError('Could not get values')

        value = ctypes.c_long(value_array[1] << 8 | value_array[0]).value
        # print "analog_read(%d) -> %d" % (pin, value)

        return value

    def  analog_write(self, pin, value):
        return True

    def digital_read(self, pin):
        return 0

    def digital_write(self, pin, value):
        return True

    def pin_mode(self, pin, mode):
        return True

    def servo_write(self, id, pos):
        ''' Usage: servo_write(id, pos)
            Position is given in radians and converted to degrees before sending
        '''
        return True

    def servo_read(self, id):
        ''' Usage: servo_read(id)
            The returned position is converted from degrees to radians
        '''
        return 0

    def ping(self, pin):
        ''' The srf05/Ping command queries an SRF05/Ping sonar sensor
            connected to the General Purpose I/O line pinId for a distance,
            and returns the range in cm.  Sonar distance resolution is integer based.
        '''
        return 0

    def get_maxez1(self, triggerPin, outputPin):
        ''' The maxez1 command queries a Maxbotix MaxSonar-EZ1 sonar
            sensor connected to the General Purpose I/O lines, triggerPin, and
            outputPin, for a distance, and returns it in Centimeters. NOTE: MAKE
            SURE there's nothing directly in front of the MaxSonar-EZ1 upon
            power up, otherwise it wont range correctly for object less than 6
            inches away! The sensor reading defaults to use English units
            (inches). The sensor distance resolution is integer based. Also, the
            maxsonar trigger pin is RX, and the echo pin is PW.
        '''
        return 0


""" Basic test for connectivity """
if __name__ == "__main__":
    myArduino = ArduinoSMBus(port = 1)
    myArduino.connect()

    print "Sleeping for 1 second..."
    time.sleep(1)

    print "Reading on analog port 0", myArduino.analog_read(0)
    print "Reading on digital port 0", myArduino.digital_read(0)
    print "Blinking the LED 3 times"
    for i in range(3):
        myArduino.digital_write(13, 1)
        time.sleep(1.0)
    #print "Current encoder counts", myArduino.encoders()

    print "Connection test successful.",

    print "Shutting down Arduino."


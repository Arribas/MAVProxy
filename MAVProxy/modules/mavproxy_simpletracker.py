#!/usr/bin/env python
'''
simpletracker Module
Javier Arribas 2019
This module computes the Azimuth and Elevation of the UAV, seen from a GCS with specific location
and sends the corresponding PWM signals to an antenna tracker equipped with a simple arduino interface connected by ttyUSB port
'''

import os
import os.path
import sys
from pymavlink import mavutil
import errno
import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings
from cuav.lib import cuav_util
import pyproj
import math
import threading
import serial
from MAVProxy.modules.lib import maestro


class simpletracker(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(simpletracker, self).__init__(mpstate, "simpletracker", "")

        self.simpletracker_settings = mp_settings.MPSettings(
            [('verbose', bool, False),
             ('lat', float, 0.0),
             ('long', float, 0.0),
             ('alt', float, 0.0),
             ('azimuth_offset_deg', float, 0.0),
             ('elevation_offset_deg', float, 0.0),
             ('tracker_tty', str, "/dev/ttyACM0"),
             ('baudrate', int, 115200)
             ])
        self.add_command('simpletracker', self.cmd_simpletracker, "simpletracker module",
                         ['status', 'set', 'gcs_location'])

        self.debug = True
        self.tracker_type = 2  # 1: text-based cmd 2: Pololu maestro cmd
        self.azimuth_deg = 0.0
        self.azimuth_angle_span_deg = 180.0
        self.azimuth_min_pwm = 496
        self.azimuth_max_pwm = 2496

        self.elevation_deg = 0.0
        self.elevation_angle_span_deg = 180.0
        self.elevation_min_pwm = 1296
        self.elevation_max_pwm = 1664
        self.last_idle_call = time.time()
        self.tracker_angle_tx_interval = 0.5
        self.lock = threading.Lock()
        if self.tracker_type == 1:
            try:
                self.ser = serial.Serial(self.simpletracker_settings.tracker_tty, self.simpletracker_settings.baudrate,
                                         timeout=1, exclusive=False)
            except Exception, e:
                print("Error opening TTY port " + self.simpletracker_settings.tracker_tty)
        elif self.tracker_type == 2:
            try:
                self.servo = maestro.Controller()
            except Exception, e:
                print("Error opening maestro controller port")
        else:
            print("Unknown tracker type ")

    # Deleting (Calling destructor)
    def __del__(self):
        if self.tracker_type == 1:
            try:
                self.servo.close()
            except Exception, e:
                print("Error closing maestro controller port")

    def usage(self):
        '''show help on command line options'''
        return "Usage: simpletracker <status|set|gcs_location lat_deg long_deg alt_m>"

    def cmd_simpletracker(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "status":
            print(self.status())
        elif args[0] == "set":
            if len(args) > 1:
                if args[1] == "tracker_tty":
                    self.simpletracker_settings.command(args[1:])

                    if self.tracker_type == 1:
                        try:
                            self.ser.close()
                        except Exception, e:
                            self.say("error closing serial port: " + str(e))
                        try:
                            self.say("Trying to open port...")
                            self.ser = serial.Serial(self.simpletracker_settings.tracker_tty,
                                                     self.simpletracker_settings.baudrate, timeout=1, exclusive=False)
                        except Exception, e:
                            self.say("error opening serial port: " + str(e))
                else:
                    self.simpletracker_settings.command(args[1:])
            else:
                self.simpletracker_settings.command(args[1:])
        elif args[0] == "gcs_location":
            self.simpletracker_settings.command(['lat', args[1]])
            self.simpletracker_settings.command(['long', args[2]])
            self.simpletracker_settings.command(['alt', args[3]])
        elif args[0] == "close_tty":
            try:
                self.ser.close()
            except Exception, e:
                self.say("error closing serial port: " + str(e))
        else:
            print(self.usage())

    def status(self):
        '''returns information about module'''
        return ("Simple antenna tracker status: \n Antenna position: Lat: %f , Long: %f , Alt: %f \n verbose: %d" % (
            self.simpletracker_settings.lat, self.simpletracker_settings.long, self.simpletracker_settings.alt,
            self.simpletracker_settings.verbose))

    def idle_task(self):
        '''called rapidly by mavproxy'''
        now = time.time()
        if now - self.last_idle_call > self.tracker_angle_tx_interval:

            self.last_idle_call = now
            self.lock.acquire()
            self.say("CGS Az: %f , El: %f" % (self.azimuth_deg, self.elevation_deg))

            if self.tracker_type == 2:
                az_pwm = self.az_deg_to_pwm(360.0-self.azimuth_deg, self.azimuth_angle_span_deg)
                el_pwm = self.el_deg_to_pwm(180.0-self.elevation_deg, self.elevation_angle_span_deg)
                try:
                    self.servo.setTarget(0, int(az_pwm * 4))  # set Azimuth servo target position (ch 0)
                    self.servo.setTarget(1, int(el_pwm * 4))  # set Elevation servo target position (ch 1)
                    if self.debug:
                        self.say("Az_pwm_x4:%4.0f,El_pwm_x4:%4.0f" % (int(az_pwm * 4), int(el_pwm * 4)))
                        curr_az = self.servo.getPosition(0)  # get the current position of servo Az
                        curr_el = self.servo.getPosition(1)  # get the current position of servo El
                        self.say("curr_Az_x4:%d,curr_El_x4:%d" % (curr_az, curr_el))
                except Exception, e:
                    self.say("error sending maestro cmd " + str(e))
            elif self.tracker_type == 1:
                try:
                    if self.ser.is_open:
                        # self.ser.write("!!!PAN:%4.0f,TLT:%4.0f\n" % (az_pwm,el_pwm))
                        str_cmd = "AzEl:%3.2f,%3.2f\n" % (self.azimuth_deg, self.elevation_deg)
                        self.ser.write(str_cmd.encode())
                        self.say(str_cmd.encode())
                        line = self.ser.readline()
                        if line.rstrip() != 'OK':
                            self.say("CMD ERROR: " + str(line))
                    else:
                        self.say("TTY port not open")
                except Exception, e:
                    self.say("error writing serial port: " + str(e))
            self.lock.release()

    def wrap_to_360(self, input_angle):
        angle = math.fmod(input_angle, 360.0)
        if angle < 0:
            angle = angle + 360
        return angle

    def az_deg_to_pwm(self, angle_deg, angle_span_deg):
        # type: (object, object) -> object
        pwm = self.azimuth_min_pwm + (self.azimuth_max_pwm - self.azimuth_min_pwm) * self.wrap_to_360(
            angle_deg - self.simpletracker_settings.azimuth_offset_deg) / angle_span_deg
        if pwm > self.azimuth_max_pwm:
            pwm = self.azimuth_max_pwm
        elif pwm < self.azimuth_min_pwm:
            pwm = self.azimuth_min_pwm

        return pwm

    def el_deg_to_pwm(self, angle_deg, angle_span_deg):
        pwm = self.elevation_min_pwm + (self.elevation_max_pwm - self.elevation_min_pwm) * (
                angle_deg - self.simpletracker_settings.elevation_offset_deg) / angle_span_deg
        if pwm > self.elevation_max_pwm:
            pwm = self.elevation_max_pwm
        elif pwm < self.elevation_min_pwm:
            pwm = self.elevation_min_pwm

        return pwm

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        if m.get_type() == 'GLOBAL_POSITION_INT':
            bearing = cuav_util.gps_bearing(self.simpletracker_settings.lat, self.simpletracker_settings.long,
                                            m.lat * 1E-7, m.lon * 1E-7)

            ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
            lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')

            long_deg = m.lon * 1E-7
            lat_deg = m.lat * 1E-7
            alt_m = m.alt * 1E-3
            uav_x, uav_y, uav_z = pyproj.transform(lla, ecef, long_deg, lat_deg, alt_m, radians=False)
            gcs_x, gcs_y, gcs_z = pyproj.transform(lla, ecef, self.simpletracker_settings.long,
                                                   self.simpletracker_settings.lat, self.simpletracker_settings.alt,
                                                   radians=False)
            # print uav_x, uav_y, uav_z
            dx = uav_x - gcs_x
            dy = uav_y - gcs_y
            dz = uav_z - gcs_z
            cos_el = (gcs_x * dx + gcs_y * dy + gcs_z * dz) / math.sqrt(
                (gcs_x ** 2 + gcs_y ** 2 + gcs_z ** 2) * (dx ** 2 + dy ** 2 + dz ** 2))
            elevation_deg = 90 - math.degrees(math.acos(cos_el))
            # self.say("UAV at Lat: %f, Long: %f , Alt: %f, CGS Az: %f , El: %f" % (lat_deg, long_deg,alt_m, bearing, elevation_deg))
            self.lock.acquire()
            # update angles
            self.azimuth_deg = bearing
            self.elevation_deg = elevation_deg
            self.lock.release()


def init(mpstate):
    '''initialise module'''
    return simpletracker(mpstate)

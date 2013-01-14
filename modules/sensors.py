#!/usr/bin/env python

'''
classes for sensor models
'''

import struct, time, numpy
from math import sin, cos

import noise
from constants import *

# TODO
class Pressure(object):

    def __init__(self, time, press_abs, press_diff1, press_diff2, temperature, mean=0, var=0):
        self.time = time
        self.press_abs = press_abs
        self.press_diff1 = press_diff1
        self.press_diff2 = press_diff2
        self.temperature = temperature
        self.sensor_noise = noise.GaussianNoise(mean, var)

    def send_to_mav(self, mav):
        bar2mbar = 1.0e3
        try:
            mav.raw_pressure_send(self.time*sec2msec,
                             self.press_abs*bar2mbar, self.press_diff1*bar2mbar,
                             self.press_diff2*bar2mbar, self.temperature*100)
        except struct.error as e:
            print 'mav raw pressure packet data exceeds int bounds'

    @classmethod
    def default(cls):
        return cls(time.time(),0,0,0,0, mean=0, var=.01)

    def from_state(self, state, attack=None):
        ground_press = 1.01325 #bar
        ground_tempC = 21.0
        tempC = 21.0  # TODO temp variation
        tempAvgK = T0 + (tempC + ground_tempC)/2
        pressBar = ground_press/math.exp(state.alt*(g/R)/tempAvgK)

        self.press_abs = pressBar
        self.press_diff1 = 0 # TODO, for velocity
        self.press_diff2 = 0 # TODO, ?
        self.temperature = tempC

        self.time = time.time()

        # Add noise to measurement
        self.press_abs += self.sensor_noise


class Imu(object):

    def __init__(self, time, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag,
            acc_mean=0, acc_var=0, gyro_mean=0, gyro_var=0, mag_mean=0, mag_var=0):

        self.time = time
        self.xacc = xacc
        self.yacc = yacc
        self.zacc = zacc
        self.xgyro = xgyro
        self.ygyro = ygyro
        self.zgyro = zgyro
        self.xmag = xmag
        self.ymag = ymag
        self.zmag = zmag

        self.acc_noise = noise.GaussianNoise(acc_mean, acc_var)
        self.gyro_noise = noise.GaussianNoise(gyro_mean, gyro_var)
        self.mag_noise = noise.GaussianNoise(mag_mean, mag_var)

    def send_to_mav(self, mav):
        try:
            mav.raw_imu_send(self.time*sec2msec,
                             self.xacc*mpss2mg, self.yacc*mpss2mg, self.zacc*mpss2mg,
                             self.xgyro*rad2mrad, self.ygyro*rad2mrad, self.zgyro*rad2mrad,
                             self.xmag*ga2mga, self.ymag*ga2mga, self.zmag*ga2mga)
        except struct.error as e:
            print 'mav raw imu packet data exceeds int bounds'

    @classmethod
    def default(cls):
        return cls(time.time(),0,0,0,0,0,0,0,0,0,
            acc_mean = 0,
            acc_var = .01,
            gyro_mean = 0,
            gyro_var = .01,
            mag_mean = 0,
            mag_var = .01)

    def from_state(self, state, attack=None):

        # accelerometer
        self.xacc = state.xacc
        self.yacc = state.yacc
        self.zacc = state.zacc
    
        # gyroscope
        self.xgyro = state.p
        self.ygyro = state.q
        self.zgyro = state.r

        # mag field properties
        # setting to constants, should
        # depend on position
        magFieldStrength = 0.5
        dip = 0.0*deg2rad
        dec = 0.0*deg2rad

        magVectN = magFieldStrength*numpy.matrix([
            [cos(dip)*cos(dec)],
            [cos(dip)*sin(dec)],
            [sin(dip)]])
        magVectB = numpy.transpose(state.C_nb)*magVectN

        # magnetometer
        self.xmag = magVectB[0,0]
        self.ymag = magVectB[1,0]
        self.zmag = magVectB[2,0]

        self.time = time.time()

        # Add noise to measurement
        self.xacc += self.acc_noise
        self.yacc += self.acc_noise
        self.zacc += self.acc_noise

        self.xgyro += self.gyro_noise
        self.ygyro += self.gyro_noise
        self.zgyro += self.gyro_noise

        self.xmag += self.mag_noise
        self.ymag += self.mag_noise
        self.zmag += self.mag_noise

class Gps(object):

    def __init__(self, time, fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible,
            latlon_mean=0, latlon_var=0, alt_mean=0, alt_var=0, vel_mean=0, vel_var=0):

        self.time = time
        self.fix_type = fix_type
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.eph = eph
        self.epv = epv
        self.vel = vel
        self.cog = cog
        self.satellites_visible = satellites_visible

        self.latlon_noise = noise.GaussianNoise(latlon_mean, latlon_var)
        self.alt_noise = noise.GaussianNoise(alt_mean, alt_var)
        self.vel_noise = noise.GaussianNoise(vel_mean, vel_var)

    def send_to_mav(self, mav):
        try:
            mav.gps_raw_int_send(self.time*sec2msec,
                             self.fix_type,
                             self.lat*rad2degE7, self.lon*rad2degE7, self.alt*m2mm,
                             self.eph*m2cm, self.epv*m2cm, self.vel*m2cm, self.cog*100*rad2deg,
                             self.satellites_visible)
        except struct.error as e:
            print 'mav gps raw int packet data exceeds int bounds'

    def from_state(self, state, attack=None):
        vel = math.sqrt(state.vN*state.vN + state.vE*state.vE)
        cog = math.atan2(state.vE, state.vN)
        if cog < 0: cog += 2*math.pi

        self.fix_type = 3
        self.lat = state.lat
        self.lon = state.lon
        self.alt = state.alt
        self.eph = 1.0
        self.epv = 5.0
        self.vel = vel
        self.cog = cog
        self.satellites_visible = 10

        self.time = time.time()

        # Add noise to measurement
        self.lat += self.latlon_noise
        self.lon += self.latlon_noise
        self.alt += self.alt_noise
        self.vel += self.vel_noise

    @classmethod
    def default(cls):
        return cls(time.time(),0,0,0,0,0,0,0,0,0,
                latlon_mean = 0,
                latlon_var = 0.0001,
                alt_mean = 0,
                alt_var = 0.1,
                vel_mean = 0,
                vel_var = 1
                )

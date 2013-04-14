#!/usr/bin/env python

'''
classes for sensor models
'''

import struct, time, numpy
from math import sin, cos

import noise
from constants import *

class Imu(object):

    def __init__(self, time, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag,
            abs_pressure, diff_pressure, pressure_alt, temperature,
            acc_mean=0, acc_var=0,
            gyro_mean=0, gyro_var=0,
            mag_mean=0, mag_var=0,
            baro_mean=0, baro_var=0):

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
        self.abs_pressure = abs_pressure
        self.diff_pressure = diff_pressure
        self.pressure_alt = pressure_alt
        self.temperature = temperature

        self.acc_noise = noise.GaussianNoise(acc_mean, acc_var)
        self.gyro_noise = noise.GaussianNoise(gyro_mean, gyro_var)
        self.mag_noise = noise.GaussianNoise(mag_mean, mag_var)
        self.baro_noise = noise.GaussianNoise(baro_mean, baro_var)

    def send_to_mav(self, mav):
        try:
            bar2mbar = 1000.0
            mav.highres_imu_send(self.time*sec2usec,
                             self.xacc, self.yacc, self.zacc,
                             self.xgyro, self.ygyro, self.zgyro,
                             self.xmag, self.ymag, self.zmag,
                             self.abs_pressure*bar2mbar, self.diff_pressure*bar2mbar,
                             self.pressure_alt, self.temperature, 65535)
        except struct.error as e:
            print 'mav raw imu packet data exceeds int bounds'

    @classmethod
    def default(cls):
        return cls(time.time(),
            xacc=0, yacc=0, zacc=0,
            xgyro=0, ygyro=0, zgyro=0,
            xmag=0, ymag=0, zmag=0,
            abs_pressure=0, diff_pressure=0,
            pressure_alt=0, temperature=0,
            acc_mean = 0, acc_var = .01,
            gyro_mean = 0, gyro_var = .01,
            mag_mean = 0, mag_var = .01,
            baro_mean = 0, baro_var = 0.0000001)

    def from_state(self, state, attack=None):

        # accelerometer
        self.xacc = state.xacc + self.acc_noise
        self.yacc = state.yacc + self.acc_noise
        self.zacc = state.zacc + self.acc_noise
    
        # gyroscope
        self.xgyro = state.p + self.gyro_noise
        self.ygyro = state.q + self.gyro_noise
        self.zgyro = state.r + self.gyro_noise

        # mag field properties
        # setting to constants, should
        # depend on position
        magFieldStrength = 0.5
        dip = 60.0*deg2rad
        dec = 0.0*deg2rad

        magVectN = magFieldStrength*numpy.matrix([
            [cos(dip)*cos(dec)],
            [cos(dip)*sin(dec)],
            [sin(dip)]])
        magVectB = numpy.transpose(state.C_nb)*magVectN

        # magnetometer
        self.xmag = magVectB[0,0] + self.mag_noise
        self.ymag = magVectB[1,0] + self.mag_noise
        self.zmag = magVectB[2,0] + self.mag_noise

        # baro
        ground_press = 1.01325 #bar
        ground_tempC = 21.0
        tempC = 21.0  # TODO temp variation
        tempAvgK = T0 + (tempC + ground_tempC)/2

        self.abs_pressure = ground_press/math.exp(state.alt*(g/R)/tempAvgK) + self.baro_noise
        self.diff_pressure = 0 + self.baro_noise # TODO, for velocity
        self.temperature = tempC
        self.pressure_alt = state.alt # TODO compute from pressure

        self.time = time.time()

class Gps(object):

    def __init__(self, time, fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible,
            pos_mean=0, pos_var=0, alt_mean=0, alt_var=0, vel_mean=0, vel_var=0):

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

        self.pos_noise = noise.GaussianNoise(pos_mean, pos_var)
        self.alt_noise = noise.GaussianNoise(alt_mean, alt_var)
        self.vel_noise = noise.GaussianNoise(vel_mean, vel_var)

    def send_to_mav(self, mav):
        try:
            mav.gps_raw_int_send(self.time*sec2usec,
                             self.fix_type,
                             self.lat*rad2degE7, self.lon*rad2degE7, self.alt*m2mm,
                             self.eph*m2cm, self.epv*m2cm, self.vel*m2cm, self.cog*100*rad2deg,
                             self.satellites_visible)
        except struct.error as e:
            print 'mav gps raw int packet data exceeds int bounds'

    def from_state(self, state, attack=None):

        sog = math.sqrt(state.vN*state.vN + state.vE*state.vE)
        cog = math.atan2(state.vE, state.vN)

        if cog < 0: cog += 2*math.pi

	r_earth = 6378100
	pos_north_error = self.pos_noise + 0	
	pos_east_error = self.pos_noise + 0	

	self.time = time.time()
        self.fix_type = 3
        self.lat = state.lat + pos_north_error/r_earth
        self.lon = state.lon + pos_east_error*cos(state.lat)/r_earth
        self.alt = state.alt + self.alt_noise
        self.eph = 1.0
        self.epv = 5.0
        self.vel = sog + self.vel_noise
        self.cog = cog
        self.satellites_visible = 10

    @classmethod
    def default(cls):
        return cls(time.time(),0,0,0,0,0,0,0,0,0,
                pos_mean = 0,
                pos_var = 1,
                alt_mean = 0,
                alt_var = 5,
                vel_mean = 0,
                vel_var = 1
                )

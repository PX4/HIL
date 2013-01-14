#!/usr/bin/env python

'''
classes for inserting attacks in sensor models
'''

import time
import sensors

class AttackPressure(object):

    def __init__(self, time, press_abs, press_diff1, press_diff2, temperature, mean=0, var=0):

        self.pressure = sensors.Pressure(time, press_abs, press_diff1, press_diff2, temperature,
                mean=mean, var=var)

    @classmethod
    def default(cls):
        return cls(time.time(),0,0,0,0)

    def send_to_mav(self, mav):
        self.pressure.send_to_mav(mav)

    def from_state(self, state, attack):
        self.pressure.from_state(state, attack)
        attack.attackValues.pressure_modifications(self.pressure)

class AttackImu(object):

    def __init__(self, time, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag,
            acc_mean=0, acc_var=0, gyro_mean=0, gyro_var=0, mag_mean=0, mag_var=0):

        self.imu = sensors.Imu(time, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag,
                acc_mean=acc_mean, acc_var=acc_var, gyro_mean=gyro_mean, gyro_var=gyro_var,
                mag_mean=mag_mean, mag_var=mag_var)

    def send_to_mav(self, mav):
        self.imu.send_to_mav(mav)

    @classmethod
    def default(cls):
        return cls(time.time(),0,0,0,0,0,0,0,0,0)

    def from_state(self, state, attack):

        self.imu.from_state(state, attack)
        attack.attackValues.imu_modifications(self.imu)

class AttackGps(object):

    def __init__(self, time, fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible,
            latlon_mean=0, latlon_var=0, alt_mean=0, alt_var=0, vel_mean=0, vel_var=0):

        self.gps = sensors.Gps(time, fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible,
                latlon_mean=latlon_mean, latlon_var=latlon_var, alt_mean=alt_mean, alt_var=alt_var,
                vel_mean=vel_mean, vel_var=vel_var)

    def send_to_mav(self, mav):
        self.gps.send_to_mav(mav)

    def from_state(self, state, attack):

        self.gps.from_state(state, attack)
        attack.attackValues.gps_modifications(self.gps)

    @classmethod
    def default(cls):
        return cls(time.time(),0,0,0,0,0,0,0,0,0)

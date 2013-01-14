#!/usr/bin/env python

'''
classes for inserting attacks in sensor models
'''

import time
import sensors

class AttackPressure(object):

    def __init__(self, time, press_abs, press_diff1, press_diff2, temperature):
        self.pressure = sensors.Pressure(time, press_abs, press_diff1, press_diff2, temperature)

    @classmethod
    def default(cls):
        return cls(time.time(),0,0,0,0)

    def send_to_mav(self, mav):
        self.pressure.send_to_mav(mav)

    def from_state(self, state, attack):
        self.pressure.from_state(state, attack)
        attack.attackValues.pressure_modifications(self.pressure)

class AttackImu(object):

    def __init__(self, time, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag):
        self.imu = sensors.Imu(time, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag)

    def send_to_mav(self, mav):
        self.imu.send_to_mav(mav)

    @classmethod
    def default(cls):
        return cls(time.time(),0,0,0,0,0,0,0,0,0)

    def from_state(self, state, attack):

        self.imu.from_state(state, attack)
        attack.attackValues.imu_modifications(self.imu)

class AttackGps(object):

    def __init__(self, time, fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible):
        self.gps = sensors.Gps(time, fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible)

    def send_to_mav(self, mav):
        self.gps.send_to_mav(mav)

    def from_state(self, state, attack):

        self.gps.from_state(state, attack)
        attack.attackValues.gps_modifications(self.gps)

    @classmethod
    def default(cls):
        return cls(time.time(),0,0,0,0,0,0,0,0,0)

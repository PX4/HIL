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
    def from_sensor(cls, pressure):
        return cls(pressure.time, pressure.press_abs, pressure.press_diff1, pressure.press_diff2, pressure.temperature)

    @classmethod
    def default(cls):
        return cls(time.time(),0,0,0,0)

    def send_to_mav(self, mav):
        self.pressure.send_to_mav(mav)

    @classmethod
    def from_state(cls, state, attack):
        meas = cls.from_sensor(sensors.Pressure.from_state(state))

        meas = attack.attackValues.pressure_modifications(meas)

        return meas

class AttackImu(object):

    def __init__(self, time, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag):
        self.imu = sensors.Imu(time, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag)

    def send_to_mav(self, mav):
        self.imu.send_to_mav(mav)

    @classmethod
    def from_sensor(cls, imu):
        return cls(imu.time, imu.xacc, imu.yacc, imu.zacc, imu.xgyro, imu.ygyro, imu.zgyro, imu.xmag, imu.ymag, imu.zmag)

    @classmethod
    def default(cls):
        return cls(time.time(),0,0,0,0,0,0,0,0,0)

    @classmethod
    def from_state(cls, state, attack):

        meas = cls.from_sensor(sensors.Imu.from_state(state))

        meas = attack.attackValues.imu_modifications(meas)

        return meas

class AttackGps(object):

    def __init__(self, time, fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible):
        self.gps = sensors.Gps(time, fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible)

    def send_to_mav(self, mav):
        self.gps.send_to_mav(mav)

    @classmethod
    def from_sensor(cls, gps):
        return AttackGps(gps.time, gps.fix_type, gps.lat, gps.lon, gps.alt, gps.eph, gps.epv, gps.vel, gps.cog, gps.satellites_visible)

    @classmethod
    def from_state(cls, state, attack):

        meas = cls.from_sensor(sensors.Gps.from_state(state))

        meas = attack.attackValues.gps_modifications(meas)

        return meas


    @classmethod
    def default(cls):
        return cls(time.time(),0,0,0,0,0,0,0,0,0)

#!/usr/bin/env python

from __future__ import print_function
import time

import aircraft
import sensors
from math import cos, sin, pi
from collections import deque

class BasicAircraft(object):

    def __init__(self, attack=None):
        t_now = time.time()
        self.x = aircraft.State.default()
        self.x_delay = deque([self.x],maxlen=10000)
        self.u = aircraft.Controls.default()

        if attack == None:
            self.imu = sensors.Imu.default()
            self.gps = sensors.Gps.default()
        else:
            import attack_sensors
            self.imu = attack_sensors.AttackImu.default()
            self.gps = attack_sensors.AttackGps.default()

        self.imu_period = 1.0/200;
        self.t_imu = t_now
        self.imu_count = 0

        self.gps_period = 1.0/10;
        self.t_gps = t_now
        self.gps_count = 0

        self.t_out = t_now

        self.attack = attack

        # test variables
        self.t0 = time.time()

    def update_state(self, fdm):
        self.x = aircraft.State.from_fdm(fdm)
        self.x_delay.append(self.x)

    def update_state_test(self, sog, cog):
        t = time.time()
        dt = t - self.t0
        r_earth = 6378100
        vN = sog*cos(cog)
        vE = sog*sin(cog)
        vD = 0
        phi = 0
        theta = 0
        psi = cog
        latDot = vN/r_earth
        lat = latDot*dt
        lonDot = vE/r_earth/cos(lat)
        lon = lonDot*dt
        alt = 1000 -vD*dt
        self.x = aircraft.State(time = t,
            phi=phi, theta=theta, psi=psi,
            p=0, q=0, r=0,
            lat=lat, lon=lon, alt=alt,
            vN=vN, vE=vE, vD=vD, xacc=0, yacc=0, zacc=-9.806)
        self.x_delay.append(self.x)
        #print 'latDot:', latDot, 'lonDot:', lonDot
        #print 'vN:', vN, 'vE:', vE, 'vD:', vD
        #print 'lat:', lat, 'lon:', lon, 'alt:', alt,
        #print 'phi:', phi, 'theta:', theta, 'psi:', psi

    def get_delayed_state(self, dt=1.0):
        tLast = self.x_delay[-1].time
        i = len(self.x_delay)
        while True:
            i = i - 1
            if i < 0:
                state = self.x_delay[0]
                break
            if tLast - self.x_delay[i].time > dt:
                state = self.x_delay[i]
                break
        #print 'delayed by:', tLast - state.time
        return state

    def update_controls(self, m):
        self.u = aircraft.Controls.from_mavlink(m)

    def send_controls(self, jsb_console):
        self.u.send_to_jsbsim(jsb_console)

    def send_state(self, mav):
        self.x.send_to_mav(mav)

    def send_imu(self, mav):
        self.imu.from_state(self.x, self.attack)
        self.imu.send_to_mav(mav)

    def send_gps(self, mav):
        self.gps.from_state(
            self.get_delayed_state(1.0),
            self.attack)
        self.gps.send_to_mav(mav)

    def send_sensors(self, mav):
        t_now = time.time()
        if t_now - self.t_gps > self.gps_period:
            self.t_gps = t_now
            self.send_gps(mav)
            self.gps_count += 1

        t_now = time.time()
        if t_now - self.t_imu > self.imu_period:
            self.t_imu = t_now
            self.send_imu(mav)
            self.imu_count += 1

        t_now = time.time()
        if t_now - self.t_out > 1:
            self.t_out = t_now
            print('imu {0:4d} Hz, gps {1:4d} Hz\n'.format(
                self.imu_count, self.gps_count))
            self.gps_count = 0
            self.imu_count = 0

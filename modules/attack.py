#!/usr/bin/env python

'''
classes for cyberattack insertion
'''

import math, time, os
from collections import defaultdict
import plotting as plot
import pickle
import numpy

class AttackDefintion(object):
	def __init__(self,nominal,name,units,label,scripts,file_name,variable,attack_values,attack_comment):
		self.nominal = nominal
		self.name = name
		self.units = units
		self.label = label
		self.scripts = scripts
		self.file_name = file_name
		self.variable = variable
		self.attack_values = attack_values
		self.attack_comment = attack_comment

class AttackValues(object):
    def __init__(self):
        pass
        #set variables

    def pressure_modifications(self, meas):

        # TODO ATTACK MODIFICATIONS

        #print 'pressure modifications'

        return meas

    def imu_modifications(self, meas):

        # TODO ATTACK MODIFICATIONS

        #print 'IMU modifications'

        return meas

    def gps_modifications(self, meas):

        # TODO ATTACK MODIFICATIONS

        #print 'GPS modifications'

        return meas
    

class Attack(object):
    def __init__(self, attack1, attack2):
        self.tFinal = 10
        self.enableUpdate = False

        self.attackList = self.generate_attacks()
        self.attack1 = self.attackList[attack1]
        self.attack2 = self.attackList[attack2]
        
        self.attackValues = AttackValues()

        self.iterator1 = -1
        self.iterator2 = -1

        self.resultKeys = ['flightFail', 'missionFail']
        self.innerSize = len(self.attack1.attack_values)
        self.outerSize = len(self.attack2.attack_values)

        self.results = dict.fromkeys(self.resultKeys)
        self.iterResults = None
        self.missionFailed = False
        for key in self.results:
            self.results[key] = numpy.empty(shape=(self.innerSize, self.outerSize))

        self.startTime = 0

        self.increment_iteration()

    def increment_iteration(self):
        completed = False

        # initial iteration
        if self.iterator1 == -1 or self.iterator2 == -1:
            self.iterator1 = 0
            self.iterator2 = 0

            self.iterResults = defaultdict(list)
        else:
            # End of inner loop
            if self.iterator2 == self.outerSize - 1:
                for key in self.resultKeys:
                    self.results[key][self.iterator1] = self.iterResults[key]
                self.iterator2 = 0

                # End of outer loop too (simulation finished)
                if self.iterator1 == self.innerSize - 1:
                    completed = True
                else:
                    self.iterator1 += 1
                    self.iterResults = defaultdict(list)
            else:
                self.iterator2 += 1
                
        # disable updates until the sim time is reset
        self.enableUpdate = False
        if not completed:
            self.set_attack_variables()

        return completed

    def set_attack_variables(self):
        attack1_var = self.attack1.attack_values[self.iterator1]
        attack2_var = self.attack2.attack_values[self.iterator2]
    
    def set_sim_start(self):
        self.startTime = time.time()
        self.enableUpdate = True

    def completed(self):
        plotStructs = plot.generatePlotStructs(self.results, [self.attack1, self.attack2], os.getcwd()+'/')
        write = open('plotStructs.pkl', 'wb')
        pickle.dump(plotStructs, write)
        write.close()
        print plotStructs
        plot.generatePlots(plotStructs)


    def update(self, fdm):
        if not self.enableUpdate:
            return False, False
            
        tsim = time.time() - self.startTime
        iterationFinished = False
        simulationFinished = False

#        if not self.missionFailed:
#            if not check_mission_env(fdm, 0):
#                self.missionFailed = True
#                self.iterResults['missionFail'].append(tsim)
#        if not check_flight_env(fdm):
#            iterationFinished = True
#            self.iterResults['flightFail'].append(tsim)
#            if not self.missionFailed:
#                # if the mission has not already been failed, failing the flight env
#                # does so
#                self.iterResults['missionFail'].append(tsim)

        if tsim > self.tFinal:
            iterationFinished = True
            if not self.missionFailed:
                self.iterResults['missionFail'].append(tsim)
            self.iterResults['flightFail'].append(tsim)

        if iterationFinished:
            simulationFinished = self.increment_iteration()
        if simulationFinished:
            self.completed()

        return iterationFinished, simulationFinished
        
    def great_circle_dist(lat1, lon1, lat2, lon2):
        d2r = math.pi/180
        R = 6371000
        return math.acos(sin(lat1*d2r)*sin(lat2*d2r) + cos(lat1*d2r)*cos(lat2*d2r)*cos((lon2-lon1)*d2r)) * R;

    def check_mission_env(fdm, tsim):

        alt_min = 2500
        alt_max = 3500
        
        lat0 = 37.616611
        lon0 = -122.416105

        destLat = 37.6082701666
        destLon = -122.400064179

        target_start_time = 70
        target_end_time = 80

        lat = fdm.get('latitude', units='degrees')
        lon = fdm.get('longitude', units='degrees')

        theater_size = 2000 
        target_window = 150

        theaterDist = great_circle_dist(lat0, lon0, lat, lon)
        targetDist = great_circle_dist(destLat, destLon, lat, lon)

        #print 'Theater distance = %f' % theaterDist
        #print 'Target distance = %f' % targetDist

        # Check that the vehicle is within the target window for the specified interval
        if tsim > target_start_time and tsim < target_end_time:
            #print 'Target distance = %f' % targetDist
            if targetDist > target_window:
                print 'Target window Violated'
                return False

        if theaterDist > theater_size:
            print 'Theater Window Violated'
            return False

        # Check that it is within the altitude range
        alt = fdm.get('altitude', units='meters')
        #if(alt < alt_min or alt > alt_max):
        #    return False;

        return True;

    def check_flight_env(fdm):
        alt = fdm.get('altitude', units='meters')

        phi = fdm.get('phi', units='radians')
        theta = fdm.get('theta', units='radians')
        psi = fdm.get('psi', units='radians')

        phidot = fdm.get('phidot', units='rps')
        thetadot = fdm.get('thetadot', units='rps')
        psidot = fdm.get('psidot', units='rps')

        p = phidot - psidot*sin(theta)
        q = cos(phi)*thetadot + sin(phi)*cos(theta)*psidot
        r = -sin(phi)*thetadot + cos(phi)*cos(theta)*psidot

        if math.fabs(p) > math.pi or math.fabs(q) > math.pi or math.fabs(r) > math.pi:
            return False
        if alt <= 0:
            return False

        return True



    
    # Generated in attack_cases.ods, don't modify here
    def generate_attacks(self):
        digitalUpdateRate = AttackDefintion(1,
            'Digital Update Rate',
            'Hz',
            'Digital Update Rate (Hz)',
            '',
            'digitalUpdateRate',
            'attack.digitalUpdateRate',
            [(float(x)-0)*1/4 for x in range(0,5)],
            "0")
        throttle = AttackDefintion(1,
            'Throttle Actuator Effectiveness',
            '%',
            'Throttle Actuator Effectiveness (%)',
            '',
            'throttle',
            'attack.actuator.throttle',
            [(float(x)-0)*1/4 for x in range(0,5)],
            "Doesn't fail it alone")
        aileron = AttackDefintion(1,
            'Aileron Actuator Effectiveness',
            '%',
            'Aileron Actuator Effectiveness (%)',
            '',
            'aileron',
            'attack.actuator.aileron',
            [(float(x)-0)*1/4 for x in range(0,5)],
            "Not connected")
        elevator = AttackDefintion(1,
            'Elevator Actuator Effectiveness',
            '%',
            'Elevator Actuator Effectiveness (%)',
            '',
            'elevator',
            'attack.actuator.elevator',
            [(float(x)-0)*1/4 for x in range(0,5)],
            "Doesn't fail it alone")
        rudder = AttackDefintion(1,
            'Rudder Actuator Effectiveness',
            '%',
            'Rudder Actuator Effectiveness (%)',
            '',
            'rudder',
            'attack.actuator.rudder',
            [(float(x)-0)*1/4 for x in range(0,5)],
            "Close to failure at 0")
        adsbFreq = AttackDefintion(0,
            'ADS-B Injection Frequency',
            'Hz',
            'ADS-B Injection Frequency (Hz)',
            '',
            'adsbFreq',
            'attack.adsb.freq',
            [(float(x)-0)*1/4 for x in range(0,5)],
            "Doesn't fail alone, Ineffective at high frequencies ")
        gpsLatFreq = AttackDefintion(0,
            'GPS Latitude Injection Frequency',
            'Hz',
            'GPS Latitude Injection Frequency (Hz)',
            '',
            'gpsLatFreq',
            'attack.gps.latFreq',
            [(float(x)-0)*30/4 for x in range(0,5)],
            "Fails at low frequencies")
        gpsLatAmplitude = AttackDefintion(0.001,
            'GPS Latitude Injection Amplitude',
            'rad',
            'GPS Latitude Injection Amplitude (rad)',
            '',
            'gpsLatAmplitude',
            'attack.gps.latAmplitude',
            [(float(x)-0)*100/4 for x in range(0,5)],
            "0")
        gpsLonFreq = AttackDefintion(0,
            'GPS Longitude Injection Frequency',
            'Hz',
            'GPS Longitude Injection Frequency (Hz)',
            '',
            'gpsLonFreq',
            'attack.gps.lonFreq',
            [(float(x)-0)*30/4 for x in range(0,5)],
            "Fails at low frequencies")
        gpsLonAmplitude = AttackDefintion(0.001,
            'GPS Longitude Injection Amplitude',
            'rad',
            'GPS Longitude Injection Amplitude (rad)',
            '',
            'gpsLonAmplitude',
            'attack.gps.lonAmplitude',
            [(float(x)-0)*100/4 for x in range(0,5)],
            "0")
        gpsAltFreq = AttackDefintion(0,
            'GPS Altitude Injection Frequency',
            'Hz',
            'GPS Altitude Injection Frequency (Hz)',
            '',
            'gpsAltFreq',
            'attack.gps.altFreq',
            [(float(x)-0)*20/4 for x in range(0,5)],
            "Fails at low frequencies")
        gpsAltAmplitude = AttackDefintion(0,
            'GPS Altitude Injection Amplitude',
            'ft',
            'GPS Altitude Injection Amplitude (ft)',
            '',
            'gpsAltAmplitude',
            'attack.gps.altAmplitude',
            [(float(x)-0)*100/4 for x in range(0,5)],
            "0")
        gpsLatLonNoise = AttackDefintion(0,
            'GPS Lat-Lon Noise Variance',
            'rad',
            'GPS Lat-Lon Noise Variance (rad)',
            '',
            'gpsLatLonNoise',
            'attack.gps.latLonNoise',
            [(float(x)-0)*1/4 for x in range(0,5)],
            "Doesn't fail by 10,000")
        gpsAltNoise = AttackDefintion(0,
            'GPS Altitude Noise Variance',
            'ft',
            'GPS Altitude Noise Variance (ft)',
            '',
            'gpsAltNoise',
            'attack.gps.altNoise',
            [(float(x)-0)*100/4 for x in range(0,5)],
            "0")
        gpsVelNoise = AttackDefintion(0,
            'GPS Velocity Noise Deviation',
            'ft/s',
            'GPS Velocity Noise Deviation (ft/s)',
            '',
            'gpsVelNoise',
            'attack.gps.velNoise',
            [(float(x)-0)*200/4 for x in range(0,5)],
            "0")
        imuGyroNoise = AttackDefintion(0,
            'IMU Gyro Noise Deviation',
            'deg/s',
            'IMU Gyro Noise Deviation (deg/s)',
            '',
            'imuGyroNoise',
            'attack.imu.gyroNoise',
            [(float(x)-0)*5/4 for x in range(0,5)],
            "0")
        imuAccelNoise = AttackDefintion(0,
            'IMU Accelerometer Noise Variance',
            'ft/s^2',
            'IMU Accelerometer Noise Variance (ft/s^2)',
            '',
            'imuAccelNoise',
            'attack.imu.accelNoise',
            [(float(x)-0)*100/4 for x in range(0,5)],
            "0")
        magDecNoise = AttackDefintion(0,
            'Magnetometer Dec Noise Variance',
            'rad',
            'Magnetometer Dec Noise Variance (rad)',
            '',
            'magDecNoise',
            'attack.mag.decNoise',
            [(float(x)-0)*2/4 for x in range(0,5)],
            "0")
        magDipNoise = AttackDefintion(0,
            'Magnetometer Dip Noise Variance',
            'rad',
            'Magnetometer Dip Noise Variance (rad)',
            '',
            'magDipNoise',
            'attack.mag.dipNoise',
            [(float(x)-0)*0.3/4 for x in range(0,5)],
            "0")
        gainVd = AttackDefintion(1,
            'Down Velocity Gain',
            '%',
            'Down Velocity Gain (%)',
            '',
            'gainVd',
            'attack.gain.Vd',
            [(float(x)-0.5)*20/4 for x in range(0,5)],
            "0")
        gainVe = AttackDefintion(1,
            'East Velocity Gain',
            '%',
            'East Velocity Gain (%)',
            '',
            'gainVe',
            'attack.gain.Ve',
            [(float(x)-0.5)*2000/4 for x in range(0,5)],
            "Won't fail by 500")
        gainVn = AttackDefintion(1,
            'North Velocity Gain',
            '%',
            'North Velocity Gain (%)',
            '',
            'gainVn',
            'attack.gain.Vn',
            [(float(x)-0.5)*2000/4 for x in range(0,5)],
            "Won't fail by 500")
        gainAccelX = AttackDefintion(1,
            'X Accelerometer Gain',
            '%',
            'X Accelerometer Gain (%)',
            '',
            'gainAccelX',
            'attack.gain.AccelX',
            [(float(x)-(-0.5))*1/4 for x in range(0,5)],
            "0")
        gainP = AttackDefintion(1,
            'P Gain',
            '%',
            'P Gain (%)',
            '',
            'gainP',
            'attack.gain.P',
            [(float(x)-0.5)*1/4 for x in range(0,5)],
            "0")
        gainQ = AttackDefintion(1,
            'Q Gain',
            '%',
            'Q Gain (%)',
            '',
            'gainQ',
            'attack.gain.Q',
            [(float(x)-0.5)*1/4 for x in range(0,5)],
            "0")
        gainQ = AttackDefintion(1,
            'R Gain',
            '%',
            'R Gain (%)',
            '',
            'gainQ',
            'attack.gain.R',
            [(float(x)-0.5)*1/4 for x in range(0,5)],
            "0")
        gainVt = AttackDefintion(1,
            'True Velocity Gain',
            '%',
            'True Velocity Gain (%)',
            '',
            'gainVt',
            'attack.gain.Vt',
            [(float(x)-0.5)*1/4 for x in range(0,5)],
            "0")
        gainAlpha = AttackDefintion(1,
            'Alpha Gain',
            '%',
            'Alpha Gain (%)',
            '',
            'gainAlpha',
            'attack.gain.Alpha',
            [(float(x)-0.5)*1/4 for x in range(0,5)],
            "0")
        gainAlt = AttackDefintion(1,
            'Altimeter Gain',
            '%',
            'Altimeter Gain (%)',
            '',
            'gainAlt',
            'attack.gain.Alt',
            [(float(x)-0.5)*1/4 for x in range(0,5)],
            "0")
        navA = AttackDefintion(0,
            'A Quaternion Offset',
            '0',
            'A Quaternion Offset',
            '',
            'navA',
            'attack.aNav(xNav.a)',
            [(float(x)-0.5)*10/4 for x in range(0,5)],
            "0")
        navB = AttackDefintion(0,
            'B Quaternion Offset',
            '0',
            'B Quaternion Offset',
            '',
            'navB',
            'attack.aNav(xNav.b)',
            [(float(x)-0.5)*10/4 for x in range(0,5)],
            "0")
        navC = AttackDefintion(0,
            'C Quaternion Offset',
            '0',
            'C Quaternion Offset',
            '',
            'navC',
            'attack.aNav(xNav.c)',
            [(float(x)-0.5)*10/4 for x in range(0,5)],
            "0")
        navD = AttackDefintion(0,
            'D Quaternion Offset',
            '0',
            'D Quaternion Offset',
            '',
            'navD',
            'attack.aNav(xNav.d)',
            [(float(x)-0.5)*10/4 for x in range(0,5)],
            "0")
        navVn = AttackDefintion(0,
            'North Velocity Offset',
            'ft/s',
            'North Velocity Offset (ft/s)',
            '',
            'navVn',
            'attack.aNav(xNav.vn)',
            [(float(x)-0.5)*10/4 for x in range(0,5)],
            "0")
        navVe = AttackDefintion(0,
            'East Velocity Offset',
            'ft/s',
            'East Velocity Offset (ft/s)',
            '',
            'navVe',
            'attack.aNav(xNav.ve)',
            [(float(x)-0.5)*10/4 for x in range(0,5)],
            "0")
        navVd = AttackDefintion(0,
            'Down Velocity Offset',
            'ft/s',
            'Down Velocity Offset (ft/s)',
            '',
            'navVd',
            'attack.aNav(xNav.vd)',
            [(float(x)-0.5)*10/4 for x in range(0,5)],
            "0")
        navLat = AttackDefintion(0,
            'Latitude Offset',
            'rad',
            'Latitude Offset (rad)',
            '',
            'navLat',
            'attack.aNav(xNav.lat)',
            [(float(x)-0.5)*10/4 for x in range(0,5)],
            "0")
        navLon = AttackDefintion(0,
            'Longitude Offset',
            'rad',
            'Longitude Offset (rad)',
            '',
            'navLon',
            'attack.aNav(xNav.lon)',
            [(float(x)-0.5)*10/4 for x in range(0,5)],
            "0")
        navAlt = AttackDefintion(0,
            'Altitude Offset',
            'ft',
            'Altitude Offset (ft)',
            '',
            'navAlt',
            'attack.aNav(xNav.alt)',
            [(float(x)-0.5)*10/4 for x in range(0,5)],
            "0")

        # Add all attacks to list
        L={'digitalUpdateRate' : digitalUpdateRate, 'throttle' : throttle, 'aileron' : aileron, 'elevator' : elevator, 'rudder' : rudder, 'adsbFreq' : adsbFreq, 'gpsLatFreq' : gpsLatFreq, 'gpsLatAmplitude' : gpsLatAmplitude, 'gpsLonFreq' : gpsLonFreq, 'gpsLonAmplitude' : gpsLonAmplitude, 'gpsAltFreq' : gpsAltFreq, 'gpsAltAmplitude' : gpsAltAmplitude, 'gpsLatLonNoise' : gpsLatLonNoise, 'gpsAltNoise' : gpsAltNoise, 'gpsVelNoise' : gpsVelNoise, 'imuGyroNoise' : imuGyroNoise, 'imuAccelNoise' : imuAccelNoise, 'magDecNoise' : magDecNoise, 'magDipNoise' : magDipNoise, 'gainVd' : gainVd, 'gainVe' : gainVe, 'gainVn' : gainVn, 'gainAccelX' : gainAccelX, 'gainP' : gainP, 'gainQ' : gainQ, 'gainQ' : gainQ, 'gainVt' : gainVt, 'gainAlpha' : gainAlpha, 'gainAlt' : gainAlt, 'navA' : navA, 'navB' : navB, 'navC' : navC, 'navD' : navD, 'navVn' : navVn, 'navVe' : navVe, 'navVd' : navVd, 'navLat' : navLat, 'navLon' : navLon, 'navAlt' : navAlt}
        return L

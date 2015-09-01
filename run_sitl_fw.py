#!/usr/bin/env python
import pymavlink.mavutil as mavutil
import pexpect, sys, os, socket, fdpexpect, select, argparse

import pymavlink.fgFDM as fgFDM

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'modules'))

import sensors, util, aircraft

class Simulator():
    @classmethod
    def command_line(cls):
        ''' command line parser '''
        parser = argparse.ArgumentParser()
        parser.add_argument('--sitl_address', help='address used for UDP communication with flight stack', default='127.0.0.1:14560')
        parser.add_argument('--script', help='jsbsim script', default='data/easystar_test.xml')
        parser.add_argument('--options', help='jsbsim options', default=None)
        parser.add_argument('--fgout', help="flight gear output", default=None)
        args = parser.parse_args()
        
        inst = cls(sitl_address=args.sitl_address, fgout=args.fgout, script=args.script, options=args.options)
        inst.run()

    def __init__(self, sitl_address, fgout, script, options):
        self.sitl_address = sitl_address
        self.fg_address = fgout

        self.Imu = sensors.Imu.default()
        self.Gps = sensors.Gps.default()
        self.Controls = aircraft.Controls.default()

        self.script = script
        self.options = options
        
        self.jsb = None
        self.jsb_console = None
        self.jsb_in = None
        self.jsb_out = None
        self.fg_out = None

    def run(self):
        # send something to simulator to get it running
        self.sitl = mavutil.mavudp(self.sitl_address, input=False)
        self.sitl.write("hello")

        #setup output to flightgear

        if self.fg_address is not None:
            self.fg_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.fg_out.connect(fg_address)

        self.init_JSBSim()
        util.pexpect_drain(self.jsb_console)
        self.jsb_console.send('resume\n')
        self.jsb_set('simulation/reset',1)
        self.update()
        self.jsb.expect("\(Trim\) executed")
        
        while self.update(): pass
    
    
    def update(self):
        # watch files
        rin = [self.jsb_in.fileno(), self.jsb_console.fileno(), self.jsb.fileno(), self.sitl.port.fileno()]
        
        try:
            (rin, win, xin) = select.select(rin, [], [], 1.0)
        except select.error:
            util.check_parent()
            return

        if self.jsb_in.fileno() in rin:
            self.process_sensor_data()
            
        if self.sitl.port.fileno() in rin:
            msg = self.sitl.recv_msg()
            self.Controls = aircraft.Controls.from_mavlink(msg)
            self.Controls.send_to_jsbsim(self.jsb_console)
        
        if self.jsb_console.fileno() in rin:
            util.pexpect_drain(self.jsb_console)

        if self.jsb.fileno() in rin:
            util.pexpect_drain(self.jsb)
            
        return True

    def process_sensor_data(self):
        buf = self.jsb_in.recv(self.fdm.packet_size())
        if len(buf) == 408:
            self.fdm.parse(buf)
            self.Imu.from_state(aircraft.State.from_fdm(self.fdm))
            self.Imu.send_to_mav(self.sitl.mav)
            self.Gps.from_state(aircraft.State.from_fdm(self.fdm))
            self.Gps.send_to_mav(self.sitl.mav)
            if self.fg_address is not None:
                self.fg_out.send(self.fdm.pack())

    def init_JSBSim(self):
        cmd = "JSBSim --realtime  --suspend --nice --simulation-rate=400  --script=%s --logdirectivefile=data/fgout.xml" % self.script
        jsb = pexpect.spawn(cmd, logfile=sys.stdout, timeout=10)
        jsb.delaybeforesend = 0
        util.pexpect_autoclose(jsb)
        i = jsb.expect(["Successfully bound to socket for input on port (\d+)",
                    "Could not bind to socket for input"])
        if i == 1:
            print("Failed to start JSBSim - is another copy running?")
            sys.exit(1)
        jsb.expect("Creating UDP socket on port (\d+)")
        jsb.expect("Successfully connected to socket for output")
        jsb.expect("JSBSim Execution beginning")
        
        # setup output to jsbsim
        jsb_out_address = ('127.0.0.1', 5124)
        print("JSBSim console on %s" % str(jsb_out_address))
        jsb_out = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        jsb_out.connect(jsb_out_address)
        jsb_console = fdpexpect.fdspawn(jsb_out.fileno(), logfile=sys.stdout)
        jsb_console.delaybeforesend = 0
        jsb_console.logfile = None
        
        # setup input from jsbsim
        jsb_in_address = ('127.0.0.1', 5123)
        print("JSBSim FG FDM input on %s" % str(jsb_in_address))
        jsb_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        jsb_in.bind(jsb_in_address)
        jsb_in.setblocking(0)
        
        # set class data
        self.jsb = jsb
        self.jsb_in = jsb_in
        self.jsb_out = jsb_out
        self.jsb_console = jsb_console
        self.fdm = fgFDM.fgFDM()

    def jsb_set(self, variable, value):
        '''set a JSBSim variable'''
        self.jsb_console.send('set %s %s\r\n' % (variable, value))

if __name__ == '__main__':
    Simulator.command_line()
    
# -*- coding: utf-8 -*-
"""
This module implements the RedPitaya Laser diode controller.

Qudi is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Qudi is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Qudi. If not, see <http://www.gnu.org/licenses/>.

Copyright (c) the Qudi Developers. See the COPYRIGHT.txt file at the
top-level directory of this distribution and at <https://github.com/Ulm-IQO/qudi/>
"""

from core.module import Base
from interface.simple_laser_interface import SimpleLaserInterface
from interface.simple_laser_interface import LaserState
from interface.simple_laser_interface import ShutterState
from interface.simple_laser_interface import ControlMode
import math
import pickle
import time
import os
import visa

class RedPitayaLaser(Base, SimpleLaserInterface):
    """
    Lazor dummy
    """
    _modclass = 'RedPitayaLaser'
    _modtype = 'hardware'

    MAX_DIODE_VOLTAGE = 1.8

    def __init__(self, config, **kwargs):
        """ """
        super().__init__(config=config, **kwargs)

        self.log.info('The following configuration was found.')

        # checking for the right configuration
        for key in config.keys():
            self.log.info('{0}: {1}'.format(key, config[key]))

        
        self.lstate = LaserState.OFF
        self.shutter = ShutterState.CLOSED
        self.mode = ControlMode.CURRENT
        self.power_setpoint = 0
        self.current_setpoint = 0
        self.voltage_offset = 0.65
        
        try:
            self.conversion=pickle.load(open(
                os.path.join(os.path.dirname(__file__),
                             "red_pitaya_laser_calibration.pickle"),"rb"))
        except IOError:
            self.conversion=[[1,0],0,[1,0],0.05,[1,0],[1,0]]

    def on_activate(self):
        """ Activate module.
        """
        config = self.getConfiguration()
        
        if 'ip_address' in config.keys():
            self._ip_address = config['ip_address']
        else:
            self.log.error('No paramerter "ip_address" configured in '
                    'Red Pitaya Counter. This parameter is mandatory.')
        if 'ip_timeout' in config.keys():
            self._ip_timeout = int(config['ip_timeout'])
        else:
            self.log.info('No timeout set for RedPitaya communication. Using default value of 10s')
            self._ip_timeout = 10

        self._rm = visa.ResourceManager('@py')
        try:
            self._ip_connection = self._rm.open_resource('TCPIP::{0}::5000::SOCKET'.format(self._ip_address),
                                                        read_termination='\r\n', write_termination='\r\n')
            self._ip_connection.timeout = self._ip_timeout * 1000
        except:
            self.log.error('This is RedPitayaCounter: could not connect to RedPitaya on IP address {0}'
                           ''.format(self._ip_address))
            raise
        self.log.info('RedPitayaCounter initialized and connected to hardware.')
        return

    def on_deactivate(self):
        """ Deactivate module.
        """
        self.off()
        #self._ip_connection.close()
        #self._rm.close()
        return

    def get_power_range(self):
        """ Return optical power range

            @return (float, float): power range
        """
        _,c = self.get_current_range()
        p = self._current_to_power(c)
        return (0, p)

    def get_power(self):
        """ Return laser power

            @return float: Laser power in watts
        """
        return self._current_to_power(self.current_setpoint)

    def get_power_setpoint(self):
        """ Return optical power setpoint.

            @return float: power setpoint in watts
        """
        return self.power_setpoint

    def set_power(self, power):
        """ Set power setpoint.

            @param float power: power setpoint

            @return float: actual new power setpoint
        """
        self.power_setpoint = power
        return self.power_setpoint

    def get_current_unit(self):
        """ Get unit for laser current.

            @return str: unit
        """
        return 'a. U.'

    def get_current_range(self):
        """ Get laser current range.

            @return (float, float): laser current range
        """
        return (0, 2496)

    def get_current(self):
        """ Get current laser current

            @return float: laser current in current curent units
        """
        return self.current_setpoint

    def get_current_setpoint(self):
        """ Get laser curent setpoint

            @return float: laser current setpoint
        """
        return self.current_setpoint

    def set_current(self, current):
        """ Set laser current setpoint

            @prarm float current: desired laser current setpoint

            @return float: actual laser current setpoint
        """
        print("!!! DEBUG: Laser set_current ", current)
        self.current_setpoint = current
        self._write_command("ANALOG:PIN AOUT1,%s" %(self.voltage_offset))
        self._write_command("ANALOG:PIN AOUT0,%s" %(float(current/100.)*self.MAX_DIODE_VOLTAGE))
        return self.current_setpoint

    def allowed_control_modes(self):
        """ Get supported control modes

            @return list(): list of supported ControlMode
        """
        return [ControlMode.CURRENT]

    def get_control_mode(self):
        """ Get the currently active control mode

            @return ControlMode: active control mode
        """
        return ControlMode.CURRENT

    def set_control_mode(self, control_mode):
        """ Set the active control mode

            @param ControlMode control_mode: desired control mode

            @return ControlMode: actual active ControlMode
        """
        return ControlMode.CURRENT

    def on(self):
        """ Turn on laser.

            @return LaserState: actual laser state
        """
        print("!!! DEBUG: Laser on")
        self._write_command("ANALOG:PIN AOUT1,%s" %(self.voltage_offset))
        self._write_command("ANALOG:PIN AOUT0,%s" %(float(self.current_setpoint/100.)*self.MAX_DIODE_VOLTAGE))
        self.lstate = LaserState.ON
        return self.lstate

    def off(self):
        """ Turn off laser.

            @return LaserState: actual laser state
        """
        print("!!! DEBUG: Laser on")
        self._write_command("ANALOG:PIN AOUT1,0")
        self._write_command("ANALOG:PIN AOUT0,0")
        self.lstate = LaserState.OFF
        return self.lstate

    def get_laser_state(self):
        """ Get laser state

            @return LaserState: actual laser state
        """
        return self.lstate

    def set_laser_state(self, state):
        """ Set laser state.

            @param LaserState state: desired laser state

            @return LaserState: actual laser state
        """
        if state == LaserState.OFF:
            self.off()
        else:
            self.on()
        return self.lstate

    def get_shutter_state(self):
        """ Get laser shutter state

            @return ShutterState: actual laser shutter state
        """
        return self.shutter

    def set_shutter_state(self, state):
        """ Set laser shutter state.

            @param ShutterState state: desired laser shutter state

            @return ShutterState: actual laser shutter state
        """
        print("!!! DEBUG: Shutter state change: ", state)
        if state == ShutterState.OPEN:
            self.voltage_offset = 0.94
        elif state == ShutterState.CLOSED:
            self.voltage_offset = 0.65
        self.shutter = state
        return self.shutter

    def get_temperatures(self):
        """ Get all available temperatures.

            @return dict: dict of temperature namce and value in degrees Celsius
        """
        return {}

    def set_temperatures(self, temps):
        """ Set temperatures for lasers with tunable temperatures.

            @return {}: empty dict, dummy not a tunable laser
        """
        return {}

    def get_temperature_setpoints(self):
        """ Get temperature setpoints.

            @return dict: temperature setpoints for temperature tunable lasers
        """
        return {}

    def get_extra_info(self):
        """ Multiple lines of dignostic information

            @return str: much laser, very useful
        """
        return "Red Pitaya Laser controller"


    def _write_command(self, cmd):
        self._ip_connection.write(cmd)

    def _query_command(self, cmd):
        return self._ip_connection.query(cmd)


    def _polyfunc(self, polyfit, x): 
        func = 0
        for i in range(len(polyfit)):
            func += x**i * polyfit[len(polyfit) - i - 1]
        return func
    
    def _current_to_power(self, value):
        return self._polyfunc(self.conversion[2], self._polyfunc(self.conversion[0], value*self.MAX_DIODE_VOLTAGE/2496.))

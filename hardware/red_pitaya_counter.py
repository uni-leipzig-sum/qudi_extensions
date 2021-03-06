# -*- coding: utf-8 -*-
"""
This file contains the Qudi hardware implementation for the Red Pitaya counter.

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

import numpy as np

import socket
import random
import time
import visa

from core.module import Base
from interface.slow_counter_interface import SlowCounterInterface
from interface.slow_counter_interface import SlowCounterConstraints
from interface.slow_counter_interface import CountingMode
from interface.odmr_counter_interface import ODMRCounterInterface


class RedPitayaCounter(Base, SlowCounterInterface, ODMRCounterInterface):

    """This is the Interface class to define the controls for the Red Pitaya counter.
    """
    _modclass = 'RedPitayaCounter'
    _modtype = 'hardware'

    def __init__(self, config, **kwargs):
        super().__init__(config=config, **kwargs)

        self.log.info('The following configuration was found.')

        # checking for the right configuration
        for key in config.keys():
            self.log.info('{0}: {1}'.format(key, config[key]))

    def on_activate(self):
        """ Initialisation performed during activation of the module.
        """

        config = self.getConfiguration()

        if 'ip_address' in config.keys():
            self._ip_address = config['ip_address']
        else:
            self.log.error('No paramerter "ip_address" configured in '
                    'Red Pitaya Counter. This parameter is mandatory.')
        
        if 'clock_frequency' in config.keys():
            self._clock_frequency=config['clock_frequency']
        else:
            self._clock_frequency = 100
            self.log.warning('No parameter "clock_frequency" configured in '
                    'Red Pitaya Counter, taking the default value of {0} Hz '
                    'instead.'.format(self._clock_frequency))

        if 'samples_number' in config.keys():
            self._samples_number = config['samples_number']
        else:
            self._samples_number = 1
            self.log.warning('No parameter "samples_number" configured in '
                    'Red Pitaya Counter, taking the default value of {0} '
                    'instead.'.format(self._samples_number))

        self.source_channels = 2
        self.odmr_length = 100

        self._connect()

        # Initialize Counter
        # NOTE: Do not do that here! This messes up the counts (why?)
        #self._send_command('COUNT:StopOutput')
        #self._send_command('COUNT:time {:f}'.format(1./self._clock_frequency))
        #self.log.info("Red Pitaya Counter: COUNT:time is "+str(1./self._clock_frequency))
        #self._send_command('COUNT:CountOutput 1000000,0')
        
        self.log.info('RedPitayaCounter initialized and connected to hardware.')
        return

    def _connect(self):
        try:
            self._ip_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._ip_connection.connect((self._ip_address, 5000))
        except:
            self.log.error('This is RedPitayaCounter: could not connect to RedPitaya on IP address {0}'
                           ''.format(self._ip_address))
            raise

    def _disconnect(self):
        self._ip_connection.shutdown(socket.SHUT_RDWR)
        self._ip_connection.close()



    def on_deactivate(self):
        """ Deinitialisation performed during deactivation of the module.
        """
        self._disconnect()
        return

    def get_constraints(self):
        """ Return a constraints class for the slow counter."""
        constraints = SlowCounterConstraints()
        constraints.min_count_frequency = 5e-5
        constraints.max_count_frequency = 5e5
        constraints.counting_mode = [
            CountingMode.CONTINUOUS,
            CountingMode.GATED,
            CountingMode.FINITE_GATED]

        return constraints

    def get_odmr_channels(self):
        """ Return a list of channel names.

        @return list(str): channels recorded during ODMR measurement
        """
        return ['Counts']

    def set_up_clock(self, clock_frequency=None, clock_channel=None):
        """ Configures the hardware clock of the NiDAQ card to give the timing.

        @param float clock_frequency: if defined, this sets the frequency of the clock
        @param string clock_channel: if defined, this is the physical channel of the clock

        @return int: error code (0:OK, -1:error)
        """

        # NOTE: Do not do that here, it messes up the counts! (why?)
        """
        if clock_frequency is not None and float(clock_frequency) != self._clock_frequency:
            self._clock_frequency = float(clock_frequency)
            self._send_command('COUNT:StopOutput')
            self._send_command('COUNT:time {:f}'.format(1./self._clock_frequency))
            self.log.info("Red Pitaya Counter: COUNT:time is "+str(1./self._clock_frequency))
            self._send_command('COUNT:CountOutput 1000000,0')
        """

        self.log.info("Count frequency set to %fHz (=%fs)" % (clock_frequency, 1./clock_frequency))
        self._send_command('COUNTER:TIME {:f}'.format(1./clock_frequency))
        self._clock_frequency = clock_frequency

        return 0

    def set_up_counter(self,
                       counter_channels=None,
                       sources=None,
                       clock_channel=None,
                       counter_buffer=None):
        """ Configures the actual counter with a given clock.

        @param string counter_channel: if defined, this is the physical channel of the counter
        @param string photon_source: if defined, this is the physical channel where the photons are to count from
        @param string clock_channel: if defined, this specifies the clock for the counter

        @return int: error code (0:OK, -1:error)
        """

        return 0

    def get_counter(self, samples=None):
        """ Returns the current counts per second of the counter.

        @param int samples: if defined, number of samples to read in one go

        @return float: the photon counts per second
        """

        if samples is None:
            samples = int(self._samples_number)
        else:
            samples = int(samples)

        #self._count_time = float(self._query_command("COUNT:time?"))
        count_data = np.empty([self.source_channels+1, samples], dtype=np.uint32)
        for i in range(samples):
            counts = [float(x) for x in self._query_command('COUNTER:COUNTS?').split(',')]
            
            counts += [sum(counts)]
            if len(counts) < self.source_channels:
                self.log.error('Red Pitaya Counter got counts from {0} channels, but {1} '
                               'were expected configured.'.format(len(counts), self.source_channels))
                return 0
                
            for j in range(self.source_channels+1):
                count_data[j, i] = counts[j]
                
        return count_data

    def get_counter_channels(self):
        """ Returns the list of counter channel names.
        @return tuple(str): channel names
        Most methods calling this might just care about the number of channels, though.
        """
        return ['APD{0}'.format(i) for i in range(self.source_channels)]+["Sum"]

    def _check_error_response(self, resp):
        if resp.startswith("ERR:"):
            raise RuntimeError(resp[4:].strip())

    def _send_command(self, cmd):
        self._query_command(cmd)

    def _query_command(self, cmd):
        try:
            if not cmd.endswith("\r\n"):
                cmd += "\r\n"
            cmd = cmd.encode("ascii")

            # Sending command
            totalsent = 0
            while totalsent < len(cmd):
                sent = self._ip_connection.send(cmd[totalsent:])
                if sent == 0:
                    self.log.error("Connection error to redpitaya...Reconnecting")
                    self._connect()
                else:
                    totalsent += sent

            # Receiving response
            chunks = []
            while True:
                chunk = self._ip_connection.recv(2048)
                chunks.append(chunk)
                if chunk == b'':
                    self.log.error("Connection error to redpitaya...Reconnecting")
                    self._connect()
                elif len(chunk) >= 1 and chunk[-1] == 0xa and chunk[-2] == 0xd:
                    break
            resp = b''.join(chunks).decode("ascii")

            self._check_error_response(resp)
            return resp
        except:
            raise


    def close_counter(self):
        """ Closes the counter and cleans up afterwards.

        @return int: error code (0:OK, -1:error)
        """

        return 0

    def close_clock(self,power=0):
        """ Closes the clock and cleans up afterwards.

        @return int: error code (0:OK, -1:error)
        """

        return 0

    #### ODMR counter interface

    def set_up_odmr_clock(self, clock_frequency=None, clock_channel=None):
        """ Configures the hardware clock of the NiDAQ card to give the timing.
        @param float clock_frequency: if defined, this sets the frequency of the
                                      clock
        @param str clock_channel: if defined, this is the physical channel of
                                  the clock
        @return int: error code (0:OK, -1:error)
        """
        return self.set_up_clock(clock_frequency, clock_channel)

    def set_up_odmr(self, counter_channel=None, photon_source=None,
                    clock_channel=None, odmr_trigger_channel=None):
        """ Configures the actual counter with a given clock.
        @param str counter_channel: if defined, this is the physical channel of
                                    the counter
        @param str photon_source: if defined, this is the physical channel where
                                  the photons are to count from
        @param str clock_channel: if defined, this specifies the clock for the
                                  counter
        @param str odmr_trigger_channel: if defined, this specifies the trigger
                                         output for the microwave
        @return int: error code (0:OK, -1:error)
        """
        return 0

    def set_odmr_length(self, length=100):
        """Set up the trigger sequence for the ODMR and the triggered microwave.
        @param int length: length of microwave sweep in pixel
        @return int: error code (0:OK, -1:error)
        """
        self._odmr_length = length
        return 0

    def count_odmr(self, length = 100):
        """ Sweeps the microwave and returns the counts on that sweep.
        @param int length: length of microwave sweep in pixel
        @return float[]: the photon counts per second
        """
        self.set_odmr_length(length)
        counts = self.get_counter(length)
        return counts[0,:]+counts[1,:] # Return the sum of APD1+APD2

    def close_odmr(self):
        """ Close the odmr and clean up afterwards.
        @return int: error code (0:OK, -1:error)
        """
        return 0

    def close_odmr_clock(self):
        """ Close the odmr and clean up afterwards.
        @return int: error code (0:OK, -1:error)
        """
        return 0

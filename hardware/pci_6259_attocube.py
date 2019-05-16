# -*- coding: utf-8 -*-
"""
This file contains the Qudi module for the confocal scanner consiting of a pci-6259 NIcard for an Attocube stage.

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
import comedi
import time

from core.module import Base, Connector, ConfigOption
from interface.confocal_scanner_interface import ConfocalScannerInterface


class ConfocalScanner(Base, ConfocalScannerInterface):
    """ Confocal scanner for an attocube stage with an amplifier
    """
    _modclass = 'ConfocalScanner'
    _modtype = 'hardware'

    # connectors
    counter_logic = Connector(interface='CounterLogic')
    afm_logic = Connector(interface='AfmLogic')

    # --- config ---
    _clock_frequency = ConfigOption('clock_frequency', 100, missing='warn')
    # The path to the comedi device of the NI card
    _ni_device_path = ConfigOption('ni_device_path', '/dev/comedi0', missing='warn')
    # Path to the calibration file for the NI card
    _ni_comedi_calibration_file = ConfigOption('ni_comedi_calibration_file',
                                               '/opt/comedi/soft_calib_comedi0', missing='warn')
    # NI card subdevice/channel to use for the trigger signal (must be a DIO channel)
    _ni_trigger_dio_subdevice = ConfigOption('ni_trigger_dio_subdevice', 2, missing='warn')
    _ni_trigger_dio_channel = ConfigOption('ni_trigger_dio_channel', 11, missing='warn')
    # NI card subdevice to use for the scanner output (must be an analog output channel)
    _ni_scanner_ao_subdevice = ConfigOption('ni_scanner_ao_subdevice', 1, missing='warn')
    # The range of the analog output channels to use (0: [-10V,10V], 1: [-5V,5V], 2: [-1*EXT V,1*EXT V])
    _ni_scanner_ao_range = ConfigOption('ni_scanner_ao_range', 0, missing='warn')
    # Max output voltage allowed for the scanner piezos
    _max_output_voltage = ConfigOption('max_output_voltage', 3, missing='warn')
    # Piezo scanner calibration values
    _piezo_scanner_x_meter_per_volt = ConfigOption('piezo_scanner_x_meter_per_volt', 1.4e-5, missing='warn')
    _piezo_scanner_y_meter_per_volt = ConfigOption('piezo_scanner_y_meter_per_volt', 1.4e-5, missing='warn')
    _piezo_scanner_z_meter_per_volt = ConfigOption('piezo_scanner_z_meter_per_volt', 1e-6, missing='warn')


    def __init__(self, config, **kwargs):
        super().__init__(config=config, **kwargs)

        self._comedi_parsed_calibration = None
        
        # Used for scanning a line. When we are waiting for the counts
        # for the current pixel, set this to false and wait for it to
        # be turned True again. Then the pixel will have been processed.
        self.pixel_done = True

        # Internal parameters
        self._line_length = None
        self._max_voltage_range = [[0, 10], [0, 10], [0, 10]]
        self._voltage_range = [[0, self._max_output_voltage], [0, self._max_output_voltage],
                               [0, self._max_output_voltage]]
        #self._position_range = [[0, 32e-6], [0, 32e-6], [0, 3e-6], [0, 0]]
        # Calculate the position ranges from the max voltage
        self._position_range = [
            [0, self._voltage_to_position(i, self._voltage_range[i][1]-self._voltage_range[i][0])]
            for i in range(3)
        ]

        self._bit_range = [32767, 65535]
        self._settling_time = 0

        self._current_position = [0, 0, 0, 0][0:len(self.get_scanner_axes())]
        self._num_points = 500

    def on_activate(self):
        """ Initialisation performed during activation of the module.
        """
        self._counter_logic = self.counter_logic()
        self._afm_logic = self.afm_logic()
        self._comedi_device = comedi.comedi_open(self._ni_device_path)
        
        # Query the NI card for the resolution in bits
        self._bit_range[1] = comedi.comedi_get_maxdata(self._comedi_device, 1, 0)
        # The minimum voltage value allowed is 0, which corresponds to floor(max_bits/2) bits
        self._bit_range[0] = int(np.floor(self._bit_range[1]/2))
        # sets dio channel configured for the trigger to output mode
        comedi.comedi_dio_config(self._comedi_device,
                                 self._ni_trigger_dio_subdevice,
                                 self._ni_trigger_dio_channel,
                                 comedi.COMEDI_OUTPUT)
        self._counter_logic.sigCounterUpdated.connect(self._got_pixel_counts)
        self.log.info("NI-Card PCI-6259 is initialized")
        return 0

    def on_deactivate(self):
        """ sets all three outputs to 0 V
        """
        self._counter_logic.sigCounterUpdated.disconnect()
        return self.reset_hardware()

    def reset_hardware(self):
        """ Resets the hardware, so the connection is lost and other programs
            can access it.

        The scanner output voltage is set to 0V on all 3 axes.

        @return int: error code (0:OK, -1:error)
        """
        for chan in range(3):
            comedi.comedi_data_write(self._comedi_device, self._ni_scanner_ao_subdevice, chan,
                                     comedi.AREF_GROUND, self._bit_range[0])
        return 0

    def get_position_range(self):
        """ Returns the physical range of the scanner.

        @return float [4][2]: array of 4 ranges with an array containing lower
                              and upper limit
        """
        return self._position_range

    def set_position_range(self, myrange=None):
        """ Sets the physical range of the scanner.

        @param float [4][2] myrange: array of 4 ranges with an array containing
                                     lower and upper limit

        @return int: error code (0:OK, -1:error)
        """
        return 0

    def set_voltage_range(self, myrange=None):
        """ Sets the voltage range of the NI Card.

        @param float [2] myrange: array containing lower and upper limit

        @return int: error code (0:OK, -1:error)
        """
        return 0

    def get_scanner_axes(self):
        return ['x', 'y', 'z']

    def get_scanner_count_channels(self):
        """
        :return: list of channel names
        """
        # Counter + AFM channel
        return self._counter_logic._counting_device.get_counter_channels() + ['AFM elevation']

    def set_up_scanner_clock(self, clock_frequency=None, clock_channel=None):
        """ Configures the hardware clock of the NiDAQ card to give the timing.

        @param float clock_frequency: if defined, this sets the frequency of the
                                      clock
        @param str clock_channel: if defined, this is the physical channel of
                                  the clock

        @return int: error code (0:OK, -1:error)
        """
        self._counter_logic.set_count_frequency(clock_frequency)
        return 0

    def set_up_scanner(self, counter_channels=None, sources=None,
                       clock_channel=None, scanner_ao_channels=None):
        """ Configures the actual scanner with a given clock.

        @param str counter_channel: if defined, this is the physical channel of
                                    the counter
        @param str photon_source: if defined, this is the physical channel where
                                  the photons are to count from
        @param str clock_channel: if defined, this specifies the clock for the
                                  counter
        @param str scanner_ao_channels: if defined, this specifies the analoque
                                        output channels

        @return int: error code (0:OK, -1:error)
        """
        return 0

    def _comedi_to_voltage(self, axis, value):
        """Convert a comedi sample to a voltage"""
        conv = comedi.polynomial_t()
        # Get conversion polynomial for the specified channel
        if comedi.get_hardcal_converter(self._comedi_device, self._ni_scanner_ao_subdevice,
                                        axis, self._ni_scanner_ao_range,
                                        comedi.COMEDI_TO_PHYSICAL, conv) == -1:
            self.log.error("Could not get softcal converter for converting comedi sample to voltage!")
            raise RuntimeError("Could not get softcal converter for converting comedi sample to voltage!")
        return comedi.to_physical(value, conv)

    def _voltage_to_comedi(self, axis, value):
        """Convert a voltage to a comedi sample"""
        conv = comedi.polynomial_t()
        # Get conversion polynomial for the specified channel
        if comedi.get_hardcal_converter(self._comedi_device, self._ni_scanner_ao_subdevice,
                                        axis, self._ni_scanner_ao_range,
                                        comedi.COMEDI_FROM_PHYSICAL, conv) == -1:
            self.log.error("Could not get softcal converter for converting voltage to comedi sample!")
            raise RuntimeError("Could not get softcal converter for converting voltage to comedi sample!")
        return comedi.from_physical(value, conv)

    def _voltage_to_position(self, axis, value):
        """Convert a voltage to a position in meters"""
        conv = [
            self._piezo_scanner_x_meter_per_volt,
            self._piezo_scanner_y_meter_per_volt,
            self._piezo_scanner_z_meter_per_volt
        ][axis]
        return value * conv

    def _position_to_voltage(self, axis, value):
        """Convert a position in meters to a voltage"""
        conv = [
            self._piezo_scanner_x_meter_per_volt,
            self._piezo_scanner_y_meter_per_volt,
            self._piezo_scanner_z_meter_per_volt
        ]
        conv = conv[axis]
        return value / conv

    def _comedi_to_position(self, axis, value):
        """Convert a comedi sample to a position in meters"""
        voltage = self._comedi_to_voltage(axis, value)
        return self._voltage_to_position(axis, voltage)

    def _position_to_comedi(self, axis, value):
        """Convert a position in meters to a comedi sample"""
        voltage = self._position_to_voltage(axis, value)
        return self._voltage_to_comedi(axis, voltage)

    def scanner_set_position(self, x=None, y=None, z=None, a=None):
        """Move stage to x, y, z.

        @param float x: postion in x-direction  (in m)
        @param float y: postion in y-direction  (in m)
        @param float z: postion in z-direction  (in m)
        @param float a: irgnored

        @return int: error code (0:OK, -1:error)
        """
        if self.module_state() == 'locked':
            self.log.error('A Scanner is already running, close this one first.')
            return -1

        for (axis, value) in ((0, x), (1, y), (2, z)):

            comedi_sample = self._position_to_comedi(axis, value)
            
            if comedi_sample < self._bit_range[0] or comedi_sample > self._bit_range[1]:
                self.log.warning(
                    "Cannot set scanner position. Position is out of range!"+
                    " (comedi_sample=%d, voltage=%gV, position=%gm)" % (
                        comedi_sample,
                        self._comedi_to_voltage(axis, comedi_sample),
                        self._comedi_to_position(axis, comedi_sample)
                    )
                )
                return -1
            else:
                if comedi.comedi_data_write(self._comedi_device, self._ni_scanner_ao_subdevice, axis, self._ni_scanner_ao_range, comedi.AREF_GROUND, comedi_sample) < 0:
                    self.log.error("scanner_set_position: comedi_data_write failed")

        if self._settling_time > 0:
            time.sleep(self._settling_time)

        comedi.comedi_dio_write(self._comedi_device, 2, 11, 0)
        comedi.comedi_dio_write(self._comedi_device, 2, 11, 1) # digital trigger pulse for counter (ca. 5 micro seconds long)
        comedi.comedi_dio_write(self._comedi_device, 2, 11, 0)

        axis_num=len(self.get_scanner_axes())
        self._current_position[0:axis_num] = [x, y, z, a][0:axis_num]
        return 0

    def get_scanner_position(self):
        """ Get the current position of the scanner hardware.

        @return float[]: current position in (x, y, z, a).
        """
        return self._current_position[0:len(self.get_scanner_axes())]

    def _set_up_line(self, length=100):
        """ Sets up the analog output for scanning a line.

        @param int length: length of the line in pixel

        @return int: error code (0:OK, -1:error)
        """
        self._line_length = length
        return 0

    def _got_pixel_counts(self):
        if not self.pixel_done:
            self.current_pixel = [self._counter_logic.countdata[i, -1] for i in
                                  range(self._counter_logic.countdata.shape[0])]
            self.pixel_done = True

    def scan_line(self, line_path=None, pixel_clock=False, include_afm_elevation=False):
        """ Scans a line and returns the counts on that line.

        @param float[][4] line_path: array of 4-part tuples defining the voltage points
        @param bool pixel_clock: whether we need to output a pixel clock for this line

        @return numpy array with shape [[pixel1_counts_APD1,pixel1_countsADP2,....],
                                        [pixel2_counts_APD1,pixel2_countsADP2,....]]
        TODO: implement triggered counting for better performance
        """
        if not isinstance(line_path, (frozenset, list, set, tuple, np.ndarray,)):
            self.log.error('Given voltage list is no array type.')
            return np.array([[-1.]])

        if np.shape(line_path)[1] != self._line_length:
            self._set_up_line(np.shape(line_path)[1])

        self.count_data = np.zeros((len(line_path[0]), len(self.get_scanner_count_channels())))
        for pixel in range(len(line_path[0])):
            self.scanner_set_position(line_path[0, pixel], line_path[1, pixel], line_path[2, pixel])
            self.pixel_done = False
            while not self.pixel_done:
                time.sleep(0.0005)
            # Append the AFM elevation channel if requested
            if include_afm_elevation:
                self.current_pixel += [self._afm_logic.get_current_elevation()]
            else:
                self.current_pixel += [0]
            self.count_data[pixel] = self.current_pixel

        return self.count_data

    def close_scanner(self):
        """ Closes the scanner and cleans up afterwards.

        @return int: error code (0:OK, -1:error)
        """

        self.log.debug('ConfocalScannerDummy>close_scanner')
        return 0

    def close_scanner_clock(self, power=0):
        """ Closes the clock and cleans up afterwards.

        @return int: error code (0:OK, -1:error)
        """

        self.log.debug('ConfocalScannerDummy>close_scanner_clock')
        return 0

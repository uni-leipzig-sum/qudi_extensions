# -*- coding: utf-8 -*-
"""
This file contains the AFM logic class.

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


from qtpy import QtCore
from collections import OrderedDict
import numpy as np
import time
import matplotlib.pyplot as plt

from core.module import Connector, StatusVar
from logic.generic_logic import GenericLogic
from core.util.mutex import Mutex


class AfmLogic(GenericLogic):
    """ This logic module gathers elevation data from the Attocube AFM software.

    @signal sigHeightUpdated: there is new elevation data available
    @signal sigBacklogChanged: the backlog (number of measurements to keep in memory) changed
    @signal sigFrequencyChanged: the measurement frequency changed
    """

    sigElevationUpdated = QtCore.Signal()
    sigBacklogChanged = QtCore.Signal()
    sigFrequencyChanged = QtCore.Signal()
    sigMeasurementStateChanged = QtCore.Signal() # Measurement started/stopped
    # Internal signals
    _sigGetNextMeasurement = QtCore.Signal()

    _modclass = 'AfmLogic'
    _modtype = 'logic'

    # Status variables
    _saving = StatusVar('saving', False)     # Save the measurements to disk?
    _backlog = StatusVar('backlog', 300)     # Number of measurements to keep in memory
    _frequency = StatusVar('frequency', 100) # Measurement frequency in Hz

    ## Declare connectors
    afm1 = Connector(interface='AfmInterface')
    savelogic = Connector(interface='SaveLogic')

    ## Private variables
    _stop_requested = False
    _thread_lock = Mutex()
    _elevationdata = np.zeros([300])

    def __init__(self, config, **kwargs):
        """ Create AfmLogic object with connectors.

        @param dict config: module configuration
        @param dict kwargs: optional parameters
        """
        super().__init__(config=config, **kwargs)

        self._saving = False

    def on_activate(self):
        # Connect to hardware and save logic
        self._afm_device = self.afm1()
        self._save_logic = self.savelogic()

        # Initialize data arrays
        self._init_data_arrays()

        self._stop_requested = False

        # connect signals
        self._sigGetNextMeasurement.connect(self._measurement_loop_body, QtCore.Qt.QueuedConnection)

    def on_deactivate(self):
        # Stop measurement loop if it is running
        if self.module_state() == 'locked':
            self._stop_measurement_join()

        self._sigGetNextMeasurement.disconnect()


    def set_backlog(self, backlog=300):
        should_restart = (self.module_state() == 'locked')
        if backlog > 0:
            self._stop_measurement_join()
            self._backlog = int(backlog)
            if should_restart:
                self.start_measurement()
        else:
            self.log.warning('backlog has to be >0. Command ignored.')
        self.sigBacklogChanged.emit(self._backlog)

    def get_backlog(self):
        return self._backlog

    def set_frequency(self, frequency=100):
        should_restart = (self.module_state() == 'locked')
        if frequency > 0:
            self._stop_measurement_join()
            self._frequency = int(frequency)
            if should_restart:
                self.start_measurement()
        else:
            self.log.warning('frequency has to be >0. Command ignored.')
        self.sigFrequencyChanged.emit(self._frequency)

    def get_frequency(self):
        return self._frequency

    def start_measurement(self):
        """ Start the measurement loop.
        The loop will emit elevation measurements at the configured frequency.
        """
        with self._thread_lock:
            if self.module_state() != 'locked':
                self.module_state.lock()
            else:
                self.log.warning('Afm measurement is already running. Start call ignored.')
                return False

            self._init_data_arrays()

            # Start the measurement loop
            self.sigMeasurementStateChanged.emit(True)
            self._sigGetNextMeasurement.emit()

    def stop_measurement(self, wait=True, timeout=5.0):
        """ Stop the measurement loop.

        @param bool wait: Wait for the loop to terminate
        @param float timeout: Wait at most this number of seconds, then timeout
        @return bool: True when the loop finished, False otherwise. (If wait is
        False, always returns True)
        """
        if not wait:
            self._stop_measurement()
            return True
        else:
            return self._stop_measurement_join(timeout)

    def get_current_elevation(self):
        """Get the current elevation.

        If the measurement loop is running, return the last measurement.
        Otherwise, request a new sample from the AFM hardware.
        """
        if self.module_state() == 'locked':
            return self._elevationdata[-1]
        else:
            return self._afm_device.get_elevation()

    def _init_data_arrays(self):
        self._elevationdata = np.zeros([self._backlog])

    def _measurement_loop_body(self):
        if self.module_state() == 'locked':
            with self._thread_lock:
                # Stop the loop if requested
                if self._stop_requested:
                    err = self._afm_device.close()
                    if err:
                        self.log.error('Could not close the AFM connection.')
                    self._stop_requested = False
                    self.module_state().unlock()
                    self.sigMeasurementStateChanged.emit(False)

                # Get a new elevation sample from the AFM hardware
                sample = self._afm_device.get_elevation()

                # Append new sample to backlog
                self._elevationdata = np.roll(self._elevationdata, -1)
                self._elevationdata[-1] = sample

                # Emit new sample event
                self.sigElevationUpdated.emit(sample)

    def _stop_measurement(self):
        if self.module_state() == 'locked':
            self._stop_requested = True

    def _stop_measurement_join(self, timeout=5.0):

        start_time = time.time()
        while self.module_state() == 'locked':
            time.sleep(0.1)
            delta = time.time() - start_time
            if delta >= timeout:
                self.log.error('Stopping the Afm measurement timed out after %.2fs' % delta)
                return False
        return True

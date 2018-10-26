# -*- coding: utf-8 -*-

"""
This file contains the Qudi hardware file to control the Rohde & Schwarz SMW200A.
"""

import visa
import time
import numpy as np

from core.module import Base, ConfigOption
from interface.microwave_interface import MicrowaveInterface
from interface.microwave_interface import MicrowaveLimits
from interface.microwave_interface import MicrowaveMode
from interface.microwave_interface import TriggerEdge


class MicrowaveSmw200a(Base, MicrowaveInterface):
    """ This is the Interface class to define the controls for the simple
        microwave hardware.
    """

    _modclass = 'MicrowaveSmw200a'
    _modtype = 'hardware'
    _ip_adress = ConfigOption('ip_address', '192.168.1.7', missing='warn')

    # TODO : Check _FREQ_SWITCH_SPEED

    # Indicate how fast frequencies within a list or sweep mode can be changed:
    _FREQ_SWITCH_SPEED = 0.003  # Frequency switching speed in s (acc. to specs)

    def on_activate(self):
        """ Initialisation performed during activation of the module. """
        # trying to load the visa connection to the module
        self.rm = visa.ResourceManager("@py")
        try:
            self._lan_connection = self.rm.open_resource('TCPIP::{}::INSTR'.format(self._ip_adress))
        except:
            self.log.error('This is SMW200A: could not connect to LAN address >>{}<<.'
                           ''.format(self._ip_adress))
            raise

        self.log.info('SMW200A initialised and connected to hardware.')
        self.model = self._lan_connection.query('*IDN?').split(',')[1]
        self._command_wait('*CLS')
        self._command_wait('*RST')
        return

    def on_deactivate(self):
        """ Cleanup performed during deactivation of the module. """
        self._command_wait('*CLS')
        self._command_wait('*RST')
        return

    def _command_wait(self, command_str):
        """
        Writes the command in command_str via GPIB and waits until the device has finished
        processing it.

        @param command_str: The command to be written
        """
        self._lan_connection.write(command_str)
        self._lan_connection.write('*WAI')
        while int(float(self._lan_connection.query('*OPC?'))) != 1:
            time.sleep(0.2)
        return

    def get_limits(self):
        """ Create an object containing parameter limits for this microwave source.

            @return MicrowaveLimits: device-specific parameter limits
        """
        limits = MicrowaveLimits()
        limits.supported_modes = (MicrowaveMode.CW, MicrowaveMode.LIST, MicrowaveMode.SWEEP)

        limits.min_frequency = 300e3
        limits.max_frequency = 2e10

        limits.min_power = -145
        limits.max_power = 30  # 3 MHz < f

        limits.list_minstep = 0.001
        limits.list_maxstep = 2e10
        limits.list_maxentries = 10000

        limits.sweep_minstep = 0.001
        limits.sweep_maxstep = 2e10
        limits.sweep_maxentries = 10001

        limits.list_maxstep = limits.max_frequency
        limits.sweep_maxstep = limits.max_frequency
        return limits

    def off(self):
        """
        Switches off any microwave output.
        Must return AFTER the device is actually stopped.

        @return int: error code (0:OK, -1:error)
        """
        mode, is_running = self.get_status()
        if not is_running:
            return 0

        if mode == 'list':
            self._command_wait(':FREQ:MODE CW')

        self._lan_connection.write('OUTP:STAT OFF')
        self._lan_connection.write('*WAI')
        while int(float(self._lan_connection.query('OUTP:STAT?'))) != 0:
            time.sleep(0.2)

        if mode == 'list':
            self._command_wait(':LIST:LEARN')
            self._command_wait(':FREQ:MODE LIST')
        return 0

    def get_status(self):
        """
        Gets the current status of the MW source, i.e. the mode (cw, list or sweep) and
        the output state (stopped, running)

        @return str, bool: mode ['cw', 'list', 'sweep'], is_running [True, False]
        """
        is_running = bool(int(float(self._lan_connection.query('OUTP:STAT?'))))
        mode = self._lan_connection.query(':FREQ:MODE?').strip('\n').lower()
        if mode == 'swe':
            mode = 'sweep'
        return mode, is_running

    def get_power(self):
        """
        Gets the microwave output power.

        @return float: the power set at the device in dBm
        """
        mode, is_running = self.get_status()
        if mode == 'list':
            return float(self._lan_connection.query(':LIST:POW?'))
        else:
            # This case works for cw AND sweep mode
            return float(self._lan_connection.query(':POW?'))

    def get_frequency(self):
        """
        Gets the frequency of the microwave output.
        Returns single float value if the device is in cw mode.
        Returns list like [start, stop, step] if the device is in sweep mode.
        Returns list of frequencies if the device is in list mode.

        @return [float, list]: frequency(s) currently set for this device in Hz
        """
        mode, is_running = self.get_status()
        if 'cw' in mode:
            return_val = float(self._lan_connection.query(':FREQ?'))
        elif 'sweep' in mode:
            start = float(self._lan_connection.query(':FREQ:STAR?'))
            stop = float(self._lan_connection.query(':FREQ:STOP?'))
            step = float(self._lan_connection.query(':SWE:STEP?'))
            return_val = [start+step, stop, step]   # TODO: why start+step ?
        elif 'list' in mode:
            # Todo check this "trigger issues"
            # Exclude first frequency entry (duplicate due to trigger issues)
            frequency_str = self._lan_connection.query(':LIST:FREQ?').split(',', 1)[1]
            return_val = np.array([float(freq) for freq in frequency_str.split(',')])
        return return_val

    def cw_on(self):
        """
        Switches on cw microwave output.
        Must return AFTER the device is actually running.

        @return int: error code (0:OK, -1:error)
        """
        current_mode, is_running = self.get_status()
        if is_running:
            if current_mode == 'cw':
                return 0
            else:
                self.off()

        if current_mode != 'cw':
            self._command_wait(':FREQ:MODE CW')

        self._lan_connection.write(':OUTP:STAT ON')
        self._lan_connection.write('*WAI')
        dummy, is_running = self.get_status()
        while not is_running:
            time.sleep(0.2)
            dummy, is_running = self.get_status()
        return 0

    def set_cw(self, frequency=None, power=None):
        """
        Configures the device for cw-mode and optionally sets frequency and/or power

        @param float frequency: frequency to set in Hz
        @param float power: power to set in dBm

        @return tuple(float, float, str): with the relation
            current frequency in Hz,
            current power in dBm,
            current mode
        """
        mode, is_running = self.get_status()
        if is_running:
            self.off()

        # Activate CW mode
        if mode != 'cw':
            self._command_wait(':FREQ:MODE CW')

        # Set CW frequency
        if frequency is not None:
            self._command_wait(':FREQ {0:f}'.format(frequency))

        # Set CW power
        if power is not None:
            self._command_wait(':POW {0:f}'.format(power))

        # Return actually set values
        mode, dummy = self.get_status()
        actual_freq = self.get_frequency()
        actual_power = self.get_power()
        return actual_freq, actual_power, mode

    def list_on(self):
        """
        Switches on the list mode microwave output.
        Must return AFTER the device is actually running.

        @return int: error code (0:OK, -1:error)
        """
        self.log.info("list_on called")

        current_mode, is_running = self.get_status()
        if is_running:
            if current_mode == 'list':
                return 0
            else:
                self.off()

        self.cw_on()
        self._command_wait(':LIST:LEARN')
        self._command_wait(':FREQ:MODE LIST')
        dummy, is_running = self.get_status()
        while not is_running:
            time.sleep(0.2)
            dummy, is_running = self.get_status()

        # Trigger list scan
        self._lan_connection.write('*TRG')
        return 0

    def set_list(self, frequency=None, power=None):
        """
        Configures the device for list-mode and optionally sets frequencies and/or power

        @param list frequency: list of frequencies in Hz
        @param float power: MW power of the frequency list in dBm

        @return tuple(list, float, str):
            current frequencies in Hz,
            current power in dBm,
            current mode
        """
        self.log.info("set_list called")

        mode, is_running = self.get_status()
        if is_running:
            self.off()

        # Cant change list parameters if in list mode
        if mode != 'cw':
            self.set_cw()
        self.cw_on()

        self._lan_connection.write(":LIST:SEL 'QUDI'")
        self._lan_connection.write('*WAI')

        # Set list frequencies
        if frequency is not None:
            s = ''
            for f in frequency[:-1]:
                s += ' {0:f},'.format(f)
            s += ' {0:f}'.format(frequency[-1])
            self._lan_connection.write(':LIST:FREQ' + s)
            self._lan_connection.write('*WAI')
            self._lan_connection.write(':LIST:MODE STEP')
            self._lan_connection.write('*WAI')

        # Set list power
        # TODO: check if the power for all frequencies is changed
        if power is not None:
            self._lan_connection.write(':LIST:POW {0:f}'.format(power))
            self._lan_connection.write('*WAI')

        #self._command_wait(':TRIG1:LIST:SOUR EXT')
        
        # Apply settings in hardware
        self._command_wait(':LIST:LEARN')
        self._command_wait(':FREQ:MODE LIST')

        actual_freq = self.get_frequency()
        actual_power = self.get_power()
        mode, dummy = self.get_status()
        return actual_freq, actual_power, mode

    def reset_listpos(self):
        """
        Reset of MW list mode position to start (first frequency step)

        @return int: error code (0:OK, -1:error)
        """
        self._list_index = 0
        self._command_wait(':LIST:RES')
        return 0

    def sweep_on(self):
        """ Switches on the sweep mode.

        @return int: error code (0:OK, -1:error)
        """
        self.log.info("sweep_on called")

        current_mode, is_running = self.get_status()
        if is_running:
            if current_mode == 'sweep':
                return 0
            else:
                self.off()

        if current_mode != 'sweep':
            self._command_wait(':FREQ:MODE SWEEP')

        self._lan_connection.write(':OUTP:STAT ON')
        dummy, is_running = self.get_status()
        while not is_running:
            time.sleep(0.2)
            dummy, is_running = self.get_status()
        return 0

    def set_sweep(self, start=None, stop=None, step=None, power=None):
        """
        Configures the device for sweep-mode and optionally sets frequency start/stop/step
        and/or power

        @return float, float, float, float, str: current start frequency in Hz,
                                                 current stop frequency in Hz,
                                                 current frequency step in Hz,
                                                 current power in dBm,
                                                 current mode
        """
        self.log.info("set_sweep called")

        mode, is_running = self.get_status()
        if is_running:
            self.off()

        if mode != 'sweep':
            self._command_wait(':FREQ:MODE SWEEP')

        if (start is not None) and (stop is not None) and (step is not None):
            #TODO is this necessary?
            self._lan_connection.write(':SWE:FREQ:STEP:LIN')
            self._lan_connection.write('*WAI')
            #TODO Check why "start-step" and not only start
            self._lan_connection.write(':FREQ:START {0:f}'.format(start - step))
            self._lan_connection.write(':FREQ:STOP {0:f}'.format(stop))
            self._lan_connection.write(':SWE:STEP:LIN {0:f}'.format(step))
            self._lan_connection.write('*WAI')

        if power is not None:
            self._lan_connection.write(':POW {0:f}'.format(power))
            self._lan_connection.write('*WAI')

        self._command_wait(':TRIG1:FSW:SOUR EXT')

        actual_power = self.get_power()
        freq_list = self.get_frequency()
        mode, dummy = self.get_status()
        return freq_list[0], freq_list[1], freq_list[2], actual_power, mode

    def reset_sweeppos(self):
        """
        Reset of MW sweep mode position to start (start frequency)

        @return int: error code (0:OK, -1:error)
        """
        self._command_wait(':SWE:RES')
        return 0

    def set_ext_trigger(self, pol=TriggerEdge.RISING, timing=None):
        """ Set the external trigger for this device with proper polarization.

        @param TriggerEdge pol: polarisation of the trigger (basically rising edge or falling edge)
        @param timing: estimated time between triggers

        @return object, float: current trigger polarity [TriggerEdge.RISING, TriggerEdge.FALLING],
            trigger timing as queried from device
        """
        mode, is_running = self.get_status()
        if is_running:
            self.off()

        if pol == TriggerEdge.RISING:
            edge = 'POS'
        elif pol == TriggerEdge.FALLING:
            edge = 'NEG'
        else:
            self.log.warning('No valid trigger polarity passed to microwave hardware module.')
            edge = None

        if edge is not None:
            self._command_wait(':TRIG1:SLOP {0}'.format(edge))

        polarity = self._lan_connection.query(':TRIG1:SLOP?')
        if 'NEG' in polarity:
            return TriggerEdge.FALLING, timing
        else:
            return TriggerEdge.RISING, timing

    def trigger(self):
        """ Trigger the next element in the list or sweep mode programmatically.

        @return int: error code (0:OK, -1:error)

        Ensure that the Frequency was set AFTER the function returns, or give
        the function at least a save waiting time.
        """

        # WARNING:
        # The manual trigger functionality was not tested for this device!
        # Might not work well! Please check that!

        curr_index = self._lan_connection.query('SOUR:LIST:INDEX?')

        self._lan_connection.write('SOUR:LIST:INDEX %d' % (int(curr_index)+1))

        #self._lan_connection.write('*TRG')
        #time.sleep(self._FREQ_SWITCH_SPEED)  # that is the switching speed
        return 0

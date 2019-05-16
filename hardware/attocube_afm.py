# -*- coding: utf-8 -*-
"""
This file contains the Qudi module for communicating with the Attocube AFM control software.

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
import time
import queue, socket, struct, threading

from core.module import Base, Connector, ConfigOption
from interface.afm_interface import AfmInterface


class AttocubeAfm(Base, AfmInterface):
    _modclass = 'AttocubeAfm'
    _modtype = 'hardware'

    # Attocube address for Z-output value (in pm)
    ADDRESS = 0x1035

    _socket = None
    _socket_lock = None
    _ip_address = None
    _ip_port = None

    def __init__(self, config, **kwargs):
        super().__init__(config=config, **kwargs)
        self._socket_lock = threading.Lock()

    def on_activate(self):
        config = self.getConfiguration()

        if 'ip_address' in config.keys():
            self._ip_address = config['ip_address']
        else:
            self.log.error('No parameter "ip_address" configured in '
                           'AttocubeAfm. This parameter is mandatory.')
        if 'ip_port' in config.keys():
            self._ip_port = config['ip_port']
        else:
            self.log.error('No parameter "ip_port" configured in '
                           'AttocubeAfm. This parameter is mandatory.')

        self._connect()

    def on_deactivate(self):
        self._disconnect()

    def get_elevation(self):
        # Get z-out in pm and convert to nm
        try:
            return self._get_param(self.ADDRESS, 0) / 1000
        except queue.Empty:
            self.log.error("Could not get elevation data from AFM. Request timed out.")
            return 0

    def close(self):
        self._disconnect()

    def _connect(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        ip = socket.gethostbyname(self._ip_address)
        self._socket.settimeout(10.0)
        self._socket.connect((ip, self._ip_port))
        self._telegram_queues = {self.ADDRESS: queue.Queue()}
        self._read_telegrams = True
        self._worker = threading.Thread(target=self._read_telegram_loop)
        self._worker.start()

    def _disconnect(self):
        self._read_telegrams = False
        self._worker.join()
        self._socket.close()

    def _send_telegram(self, flag, address, index, fmt, data):
        with self._socket_lock:
            size = 16
            size += struct.calcsize(fmt)
            fmt = "iiiii"+fmt
            if not type(data) in (tuple, list):
                data = (data,)
            buf = struct.pack(fmt, size, flag, address, index, 0, *data)
            self._socket.send(buf)

    def _read_telegram(self):
        size, = struct.unpack("i", self._socket.recv(4))
        data = bytes()
        while len(data) < size:
            data += self._socket.recv(size)
        flag, address, index, = struct.unpack("iii", data[:12])
        if flag == 3:
            fmt = "{:d}i".format(int((len(data)-20)/4))
            values = struct.unpack(fmt.format(len(data)-20), data[20:])
        else:
            fmt = "{:d}i".format(int((len(data)-16)/4))
            values = struct.unpack(fmt.format(len(data)-16), data[16:])

        return (flag, address, index, values)

    def _get_from_queue(self, address):
        """This will block until a value is received for the given address
        If the get operation times out, a queue.Empty exception is thrown.
        """
        if not address in self._telegram_queues:
            self._telegram_queues[address] = queue.Queue()
        val = self._telegram_queues[address].get(True, 1.0)
        self._telegram_queues[address].task_done()
        return val

    def _clear_queue(self, address):
        if address in self._telegram_queues:
            self._telegram_queues[address] = queue.Queue()

    def _read_telegram_loop(self):
        while self._read_telegrams:
            try:
                flag, address, index, value = self._read_telegram()
                if address in self._telegram_queues:
                    self._telegram_queues[address].put((time.time(), flag, address, index, value,))
            except socket.timeout:
                continue
            except:
                raise

    def _set_param(self, address, index, value, get_response=False):
        self._send_telegram(0, address, index, "i", value)
        if get_response:
            _t, _flag, _addr, _ind, _val = self._get_from_queue(address)
            if not index is _ind:
                self.log.warning("Attocube: set_param got a response back with a different index.")
            return _val

    def _get_param(self, address, index):
        self._clear_queue(address)
        self._send_telegram(1, address, index, "", [])
        _t, _flag, _addr, _ind, _val = self._get_from_queue(address)

        if not index is _ind:
            self.log.warning("Attocube: get_param got a response back with a different index.")
        if len(_val) < 1:
            self.log.warning("Attocube: get_param failed. Got no value back.")
        return _val[0]

    def _enable_channel(self, channel):
        self._send_telegram(6, 0x1200, 0, "i", channel)
        self._send_telegram(6, 0x1202, 0, "i", channel)

    def _disable_channel(self, channel):
        self._send_telegram(6, 0x1201, 0, "i", channel)

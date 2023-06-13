
import logging
import serial
import serial.tools.list_ports
from struct import pack, unpack
from VRxControl import VRxController, VRxDevice, VRxDeviceMethod
import Config
import threading

logger = logging.getLogger(__name__)
RHAPI = None


def registerHandlers(args):
    if 'registerFn' in args:
        args['registerFn'](EspnowController('espnow', 'ESPNOW', RHAPI))

def initialize(**kwargs):
    if 'Events' in kwargs:
        kwargs['Events'].on('VRxC_Initialize', 'VRx_register_espnow', registerHandlers, {}, 75)
    if 'RHAPI' in kwargs:
        rhapi = kwargs['RHAPI']
        rhapi.register_pilot_attribute("mac", "ESPNOW MAC Address", "text")
        rhapi.register_ui_panel("ESPNOW", "ESPNOW Router Config", "settings")
        rhapi.register_general_setting("espnow-ap-ssid", "AP SSID", "ESPNOW", "text")
        rhapi.register_general_setting("espnow-ap-channel", "AP Channel", "ESPNOW", "text")
        rhapi.register_general_setting("espnow-address", "Address", "ESPNOW", "text")
        rhapi.register_general_setting("espnow-baud", "Baudrate", "ESPNOW", "text")
        rhapi.register_general_setting("espnow-race-director-mac", "RD MAC", "ESPNOW", "text")
        global RHAPI
        RHAPI = rhapi

def _logd(dbg: str):
    logger.debug(f"ESP-NOW: {dbg}")

def _logi(info: str):
    logger.info(f"ESP-NOW: {info}")

def _logw(wrn: str):
    logger.warning(f"ESP-NOW WARN: {wrn}")

def _loge(err: str):
    logger.error(f"ESP-NOW ERROR: {err}")


class EspnowController(VRxController):
    def __init__(self, name, label, api):
        self.ser = serial.Serial()
        self.ready = False
        self.config = {}
        self.seat_frequencies = [0] * 8
        self.current_heat = 0
        self.pilots_in_heat = []
        self._rhapi = api
        super().__init__(name, label)

    def _handle_serial(self, ser: serial.Serial):
        _logd("== SERIAL READ ==")
        _msp_msg = MSP()
        _log_str = ""
        try:
            while True:
                _byte = ser.read(1)
                if _byte:
                    try:
                        msp_received = _msp_msg.process(int.from_bytes(_byte, 'little'))
                    except MSP.MspError as error:
                        _loge(f"MSP parsing error {error}")
                        continue
                    if msp_received:
                        # MSP message received...
                        if _msp_msg.func == EspNowCommands.FUNC_LAP_TIMER:
                            _payload = _msp_msg.get_msg()
                            _command = EspNowCommands.get_subcommand(_payload)
                            if _command == EspNowCommands.SUBCMD_LAP_TIMER_START:
                                _logi("Laptimer start received!")
                                if self._rhapi:
                                    pass
                            elif _command == EspNowCommands.SUBCMD_LAP_TIMER_STOP:
                                _logi("Laptimer stop received!")
                                if self._rhapi:
                                    pass
                            else:
                                _logw(f"Laptimer command {_command} not handled!")
                    elif not _msp_msg.ongoing():
                        try:
                            _log_str += _byte.decode()
                            if '\n' in _log_str:
                                _logi(f"ROUTER LOG: '{_log_str.strip()}'")
                                _log_str = ""
                        except UnicodeDecodeError:
                            pass
        except serial.serialutil.SerialException as error:
            _loge(f"Serial connection exception: '{error}' -> exit loop")

    def _test_port(self, port: str) -> bool:
        if port:
            self.ser.port = port
            return self._check_device()
        return False

    def _discoverPort(self):
        # Automatic port discovery for ESP-NOW device
        _logd("Finding a serial port...")
        self.ser.timeout = 1
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            try:
                if self._test_port(p.device):
                    return
            except serial.serialutil.SerialException:
                pass
            _logd(f"No module at {p.device}")
            self.ser.close()
        _logw("No module discovered or configured")

    def _create_device(self, name, state=False, mac=None):
        device = VRxDevice()
        device.id = name
        device.name = name
        device.type = "ESP-NOW"
        device.connected = state
        device.ready = state
        if mac:
            device.name = mac
        device.map.method = VRxDeviceMethod.ALL
        self.addDevice(device)

    def _check_device(self) -> bool:
        _logi(f" Trying dev {self.ser.port} baud: {self.ser.baudrate}")
        ping = EspNowCommands.msg_ping()
        try:
            self.ser.open()
            # time.sleep(2)
            self.ser.reset_input_buffer()
            self.ser.write(ping)
            self.ser.flush()
            msp = EspNowCommands.msp_receive(self.ser, 2)
            if msp and msp.func == EspNowCommands.FUNC_ROUTER:
                ping_payload = msp.get_msg()
                if EspNowCommands.get_subcommand(ping_payload) == EspNowCommands.SUBCMD_ROUTER_PING:
                    mac_addr = ':'.join([hex(x) for x in ping_payload[4:]])
                    # _logi(f"ping resp: {[hex(x) for x in ping_payload]}")
                    _logi(f"Module Found at {self.ser.port}")
                    _logi(f"Module MAC address {mac_addr}")
                    self.ready = True

                    self._create_device(self.ser.port, True, mac_addr)

                    thread = threading.Thread(target=self._handle_serial, args=(self.ser,))
                    thread.daemon = True
                    thread.start()
                    return True
        except serial.serialutil.SerialException:
            pass
        return False

    def onStartup(self, _args):
        rhdata = self.racecontext.rhdata
        ap_ssid = rhdata.get_option("espnow-ap-ssid", "")
        ap_channel = rhdata.get_option("espnow-ap-channel", "")
        conn_addr = rhdata.get_option("espnow-address", "")
        conn_baud = rhdata.get_option("espnow-baud", "")
        if not ap_ssid:
            ap_ssid = "RotorHazard ESP-NOW Router"
            rhdata.set_option("espnow-ap-ssid", ap_ssid)
        if not ap_channel:
            ap_channel = "6"
            rhdata.set_option("espnow-ap-channel", ap_channel)
        _logi(f"********* WIFI AP:'{ap_ssid}', CH:{ap_channel}")
        _logi(f"********* SERIAL addr:{conn_addr}, baud:{conn_baud}")

        # Fetch config.json
        self.config = Config.VRX_CONTROL
        # Fetch used node freq
        self.seat_frequencies = [node.frequency for node in self.racecontext.interface.nodes]

        baud = self.config.get('espnow_baudrate', 921600)
        # baud = self.racecontext.rhdata.get_option('espnow_baudrate', baud)
        if conn_baud:
            baud = conn_baud
        try:
            baud = int(baud, 10)
        except ValueError:
            _loge(f"Unable to convert baudrate value '{conn_baud}' to integer")
            return
        port = self.config.get('espnow_port', None)
        # port = self.racecontext.rhdata.get_option('espnow_port', port)
        if conn_addr:
            port = conn_addr
        self.ser.baudrate = baud
        if not self._test_port(port):
            self._discoverPort()
        if self.ser.isOpen():
            # Connection ok, setup AP
            self._sendMessage(EspNowCommands.msg_router_set_ssid(ap_ssid, ap_channel))
            rd_mac = rhdata.get_option("espnow-race-director-mac", "")
            if rd_mac:
                _logi(f"********* RD's MAC: {rd_mac}")
                self._sendMessage(EspNowCommands.msg_router_set_rd(rd_mac))
        # Make sure the initially loaded heat is also taken into account
        self.onHeatSet({})

    def onHeatSet(self, _args):
        """
        This function is called when new heat is selected.
        Send pilots info to ESP-NOW sender and prepare ESP-NOW peers.
        """
        # logger.info(f"onHeatSet: _args {_args}")
        if not self.ready:
            return
        # Remove all existing peers
        msg = EspNowCommands.msg_router_reset()
        self._sendMessage(msg)
        self.pilots_in_heat = []

        race = self.racecontext.race
        seat_pilots = race.node_pilots
        rhdata = self.racecontext.rhdata
        self.current_heat = current_heat = race.current_heat
        # heat = rhdata.get_heat(current_heat)
        _logd(f"seat_pilots: {seat_pilots} in heat {current_heat}")
        for seat_id, pilot_id in seat_pilots.items():
            if pilot_id:
                freq = self.seat_frequencies[seat_id]
                pilot = rhdata.get_pilot(pilot_id)
                if not freq:
                    _loge(f"Not frequency for pilot {pilot} on seat {seat_id}... ignored")
                    # Invalid config!
                    continue
                address = rhdata.get_pilot_attribute_value(pilot_id, 'mac')
                round_num = rhdata.get_max_round(current_heat) or 0
                if address:
                    _logd(f"  ! {pilot.callsign}, addr: {address}, "
                          f"heat: {current_heat}, round: {round_num}, freq: {freq}")
                    # Add peer
                    msg = EspNowCommands.msg_router_peer_add(address, seat_id, current_heat, freq)
                    self._sendMessage(msg)
                    self.pilots_in_heat.append(pilot)

    def onRaceStage(self, _args):
        # logger.info(f"onRaceStage: _args {_args} ... ARM NOW (race is about to start)")
        if not self.ready:
            return
        self._sendRaceStart()
        '''
        # Debug stuff:
        seat_pilots = self.racecontext.race.node_pilots
        rhdata = self.racecontext.rhdata
        for seat_id, pilot_id in seat_pilots.items():
            if pilot_id:
                pilot = rhdata.get_pilot(pilot_id)
                if pilot not in self.pilots_in_heat:
                    # Ignore pilot if not configured
                    continue
                address = rhdata.get_pilot_attribute_value(pilot_id, 'mac')
                if address:
                    _logi(f"  ! {pilot.callsign}, addr: {address}")
        '''

    def onRaceStart(self, _args):
        # logger.info(f"onRaceStart: _args {_args} ... RACE ON")
        if not self.ready:
            return
        self._sendRaceStart()

    def onRaceFinish(self, _args):
        # logger.info(f"onRaceFinish: _args {_args}")
        if not self.ready:
            return
        self._sendRaceStop()

    def onRaceStop(self, _args):
        # logger.info(f"onRaceStop: _args {_args}")
        if not self.ready:
            return

    def onRaceLapRecorded(self, args):
        # logger.info(f"onRaceLapRecorded: _args {args}")
        if not self.ready:
            return
        if 'node_index' not in args:
            logger.error('Failed to send results: Seat not specified')
            return False

        current_heat = self.racecontext.race.current_heat
        round_num = self.racecontext.rhdata.get_max_round(current_heat) or 0

        node_index = args['node_index']
        lap = args['lap']
        lap_number = lap['lap_number']
        lap_time_ms = int(lap['lap_time'] + 0.5)
        # round_num = self.racecontext.rhdata.get_max_round(current_heat) or 0

        # _logd(f"LAP! node {node_index}, lap {lap_number}, time {lap_time_ms}")

        msg = EspNowCommands.msg_laptime(lap_time_ms, lap_number, current_heat, node_index, round_num)
        self._sendMessage(msg)
        """
        # Get relevant results
        if 'gap_info' in args:
            info = args['gap_info']
        else:
            info = Results.get_gap_info(self.racecontext, node_index)
        # send message to crosser
        address = self.racecontext.rhdata.get_pilot_attribute_value(info.current.pilot_id, 'mac')
        if address:
            _logi(f"  ... send to address: {address}")
        """

    def onLapsClear(self, _args):
        # logger.info(f"onLapsClear: _args {_args}")
        if not self.ready:
            return

    def onSendMessage(self, args):
        # logger.info(f"onSendMessage: _args {args}")
        if not self.ready:
            return

    def _sendRaceStart(self):
        current_heat = self.racecontext.race.current_heat
        round_num = self.racecontext.rhdata.get_max_round(current_heat) or 0
        msg = EspNowCommands.msg_start(current_heat, 0xff, round_num)
        self._sendMessage(msg)

    def _sendRaceStop(self):
        current_heat = self.racecontext.race.current_heat
        round_num = self.racecontext.rhdata.get_max_round(current_heat) or 0
        msg = EspNowCommands.msg_stop(current_heat, 0xff, round_num)
        self._sendMessage(msg)

    def _sendMessage(self, payload):
        try:
            if not self.ser.isOpen():
                self.ser.open()
            self.ser.write(payload)
            self.ser.flush()
        except Exception as ex:
            _loge(f"Unable to send data: {ex}")


class EspNowCommands:
    FUNC_LAP_TIMER = 0x4c54  # ['L', 'T']
    SUBCMD_LAP_TIMER_REGISTER = 0x01
    SUBCMD_LAP_TIMER_SET_NODE = 0x02
    SUBCMD_LAP_TIMER_START = 0x03
    SUBCMD_LAP_TIMER_STOP = 0x04
    SUBCMD_LAP_TIMER_LAP = 0x05

    FUNC_ROUTER = 0x5254  # ['R', 'T'] for ESP-NOW router as RT
    SUBCMD_ROUTER_RESET = 0x00
    SUBCMD_ROUTER_ADD = 0x01
    SUBCMD_ROUTER_PING = 0x02
    SUBCMD_ROUTER_WIFI = 0x03
    SUBCMD_ROUTER_RD = 0x04

    FLAG_BROADCAST = 0x1

    @staticmethod
    def fill_header(function: int, payload: bytes, flags: int = 0) -> bytes:
        length = len(payload)
        return pack(f"<B H H {length}s", flags, function, length, payload)

    @classmethod
    def _generate_msp(cls, message: bytes, command: bool = False) -> bytes:
        """
        Generate MSPv2 message

        Note: RESP by default
        """
        crc = 0
        for x in message:
            crc = MSP.calc_crc(x, crc)
        # MSPv2 COMMAND
        return pack(f"<B B B {len(message)}s B", ord('$'), ord('X'), ord(['>', '<'][command]), message, crc)

    @classmethod
    def get_subcommand(cls, data):
        return unpack("<I", bytes(data[:4]))[0]

    @classmethod
    def msg_start(cls, race_id: int, node_id: int, round_id: int = 0):
        """
        typedef struct {
            uint32_t subcommand;
            uint16_t node_index;
            uint16_t race_id;
            uint16_t round_id;
        } laptimer_start_t;
        """
        payload = pack("<I H H H", cls.SUBCMD_LAP_TIMER_START, node_id, race_id, round_id)
        return cls._generate_msp(cls.fill_header(cls.FUNC_LAP_TIMER, payload, cls.FLAG_BROADCAST))

    @classmethod
    def msg_stop(cls, race_id: int, node_id: int, round_id: int = 0):
        """
        typedef struct {
            uint32_t subcommand;
            uint16_t node_index;
            uint16_t race_id;
            uint16_t round_id;
        } laptimer_stop_t;
        """
        payload = pack("<I H H H", cls.SUBCMD_LAP_TIMER_STOP, node_id, race_id, round_id)
        return cls._generate_msp(cls.fill_header(cls.FUNC_LAP_TIMER, payload, cls.FLAG_BROADCAST))

    @classmethod
    def msg_laptime(cls, laptime_ms: int, lap_idx: int, race_id: int, node_id: int, round_id: int = 0):
        """
        typedef struct {
            uint32_t subcommand;
            uint32_t lap_time_ms;
            uint16_t node_index;
            uint16_t race_id;
            uint16_t round_id;
            uint8_t lap_index;
        } laptimer_lap_t;
        """
        payload = pack("<I I H H H B", cls.SUBCMD_LAP_TIMER_LAP, laptime_ms, node_id, race_id, round_id, lap_idx)
        return cls._generate_msp(cls.fill_header(cls.FUNC_LAP_TIMER, payload))

    @classmethod
    def msg_ping(cls):
        """
        typedef struct {
            uint32_t subcommand;
        } esp_now_router_ping_t;
        """
        payload = pack("<I", cls.SUBCMD_ROUTER_PING)
        return cls._generate_msp(cls.fill_header(cls.FUNC_ROUTER, payload))

    @classmethod
    def msg_router_reset(cls):
        """
        typedef struct {
            uint32_t subcommand;
        } esp_now_router_reset_t;
        """
        payload = pack("<I", cls.SUBCMD_ROUTER_RESET)
        return cls._generate_msp(cls.fill_header(cls.FUNC_ROUTER, payload))

    @classmethod
    def msg_router_peer_add(cls, mac: str, node_id: int, race_id: int, freq: int):
        """
        typedef struct {
            uint32_t subcommand;
            uint16_t node_id;
            uint16_t race_id;
            uint16_t freq;
            uint8_t mac[6];
        } esp_now_router_peer_add_t;

        ==> will generate&send 'laptimer_register_resp_t' to peer
        """
        mac = mac.strip()
        if ',' in mac:
            _mac = mac.split(',')
            mac = 0
            for val in _mac:
                mac <<= 8
                mac += int(val, 10)
        elif ":" in mac:
            mac = int(mac.replace(':', '')[:12], 16)
        else:
            mac = int(mac[:12], 16)
        mac = mac.to_bytes(6, 'big')
        payload = pack("<I H H H 6s", cls.SUBCMD_ROUTER_ADD, node_id, race_id, freq, mac)
        return cls._generate_msp(cls.fill_header(cls.FUNC_ROUTER, payload))

    @classmethod
    def msg_router_set_ssid(cls, ssid: str, channel: str):
        """
        typedef struct {
            uint32_t subcommand;
            uint8_t channel;
            uint8_t ssid[1];
        } esp_now_router_set_wifi_t;
        """
        if type(ssid) != bytes:
            ssid = bytes(ssid)
        ssid = ssid[:32]  # Cut SSID to its max length
        channel = int(channel) % 15  # Max 14 channels and 0 is auto
        payload = pack(f"<I B {len(ssid)}s B", cls.SUBCMD_ROUTER_WIFI, channel, ssid, 0)
        return cls._generate_msp(cls.fill_header(cls.FUNC_ROUTER, payload))

    @classmethod
    def msg_router_set_rd(cls, mac: str):
        """
        typedef struct {
            uint32_t subcommand;
            uint8_t mac[6];
        } esp_now_router_set_rd_t;
        """
        mac = mac.strip()
        if ',' in mac:
            _mac = mac.split(',')
            mac = 0
            for val in _mac:
                mac <<= 8
                mac += int(val, 10)
        elif ":" in mac:
            mac = int(mac.replace(':', '')[:12], 16)
        else:
            mac = int(mac[:12], 16)
        mac = mac.to_bytes(6, 'big')
        payload = pack("<I 6s", cls.SUBCMD_ROUTER_RD, mac)
        return cls._generate_msp(cls.fill_header(cls.FUNC_ROUTER, payload))

    @staticmethod
    def msp_receive(ser: serial.Serial, timeout: int = 2):
        msp_msg = MSP()
        _timeout = ser.timeout
        ser.timeout = timeout
        try:
            while True:
                _byte = ser.read(1)
                if not _byte:
                    # timeout
                    break
                _byte = int.from_bytes(_byte, 'little')
                if msp_msg.process(_byte):
                    # MSP parsing done
                    break
        except serial.serialutil.SerialException:
            msp_msg = None
        except MSP.MspError as _err:
            msp_msg = None
            _loge(f"MSP parsing error {_err}")
        ser.timeout = _timeout
        return msp_msg


class MSP:
    class MspError(Exception):
        pass

    def __init__(self):
        self._crc = 0
        self._msg = []
        self._ongoing = False
        self._state = "type"
        self._len = 0

        self.resp = False  # False = COMMAND, True = RESP
        self.func = None
        self.flags = None

    @staticmethod
    def calc_crc(data: int, crc: int = 0):
        crc = crc ^ data
        for ii in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0xD5
            else:
                crc = crc << 1
        return crc & 0xFF

    def ongoing(self):
        return self._ongoing

    def reset(self):
        self._ongoing = False
        self._state = "type"

    def process(self, byte):
        if not self._ongoing:
            self._ongoing = byte == ord('$')
            return False
        _state = self._state
        if _state == "type":
            if byte != ord('X'):  # Only MSPv2
                self.reset()
                raise self.MspError("Not MSPv2")
            self._state = "cmd"
            return False
        if _state == "cmd":
            if byte not in [ord('<'), ord('>')]:
                self.reset()
                raise self.MspError("Invalid MSP message type")
            self.resp = byte == ord('>')
            self._state = "hdr"
            self._crc = 0
            self._msg = []
            return False
        if _state == "hdr":
            self._msg.append(byte)
            self._crc = MSP.calc_crc(byte, self._crc)
            if len(self._msg) == 5:
                self._state = "data"
                self.flags = self._msg[0]
                self.func = self._msg[1] + (self._msg[2] << 8)
                self._len = self._msg[3] + (self._msg[4] << 8)
                self._msg = []
                # _logd(f"msp flags {self.flags}, func {self.func}, len: {self._len}")
            return False
        if _state == "data":
            self._msg.append(byte)
            self._crc = MSP.calc_crc(byte, self._crc)
            if self._len <= len(self._msg):
                self._state = "crc"
            return False
        if _state == "crc":
            self.reset()
            if self.func not in [EspNowCommands.FUNC_LAP_TIMER, EspNowCommands.FUNC_ROUTER]:
                raise self.MspError("Not a lap timer message")
            if byte != self._crc:
                raise self.MspError("CRC mismatch")
            return True
        self.reset()
        raise MSP.MspError(f"Parsing failed in state: {self._state}")

    def isOngoing(self) -> bool:
        return self._ongoing

    def get_msg(self):
        return self._msg

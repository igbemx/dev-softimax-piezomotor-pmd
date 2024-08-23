import socket
import queue
import time
from threading import Thread, Lock, Event, Condition

import tango
from tango import DeviceClass, DevState, DevFailed
from tango.server import Device, attribute, command, device_property

ATTR_UPDATE_DELAY = 0.0001  # Attribute update period
SEND_REQ_DELAY = 0.001
TIMEOUT = 1
POS_TOLERANCE = 0.1
JOG_STEPS_N = 32



class PiezoMotorPMDCtrl(Device):
    # Device Properties
    moxa_host = device_property(dtype=str, default_value="b-softimax-moxa-0")
    moxa_port = device_property(dtype=int, default_value=4001)
    moxa_reconnect_delay = device_property(dtype=float, default_value=5)
    enc_res = device_property(dtype=float, default_value=1) # Encoder resolution in nm
    enc_sign = device_property(dtype=int, default_value=-1)
    max_step_rate = device_property(dtype=int, default_value=972)


    status_table = {
        'd1': ["comError", "encError", "voltageError", "cmdError"],
        'd2': ["reset", "xLimit", "script", "index"],
        'd3': ["servoMode", "targetLimit", "targetMode", "targetReached"],
        'd4': ["parked", "overheat", "reverse", "running"]
    }

    # Device Attributes
    position = attribute(dtype=float, access=tango.AttrWriteType.READ_WRITE,
                         label="Position", unit="micron", format="%.2f")
    
    enc_pos = attribute(dtype=int, access=tango.AttrWriteType.READ,
                         label="EncPos", unit="counts", format="%10d")

    update_rate = attribute(dtype=float, access=tango.AttrWriteType.READ,
                         label="UpdateRate", unit="ms", format="%.1f")

    velocity = attribute(dtype=float, access=tango.AttrWriteType.READ_WRITE,
                         label="Velocity", unit="micron/s", format="%.2f")

    step_rate = attribute(dtype=int, access=tango.AttrWriteType.READ_WRITE,
                         label="StepRate", unit="Hz", format="%4d")

    spc = attribute(dtype=int, access=tango.AttrWriteType.READ,
                         label="SPC", unit="steps", format="%4d")

    status_ctrl = attribute(dtype=str, access=tango.AttrWriteType.READ,
                         label="StatusCtrl")

    in_pos = attribute(dtype=bool, access=tango.AttrWriteType.READ,
                         label="InPos")
    
    parked = attribute(dtype=bool, access=tango.AttrWriteType.READ,
                         label="Parked")

    reverse = attribute(dtype=bool, access=tango.AttrWriteType.READ,
                         label="Reverse")

    overheat = attribute(dtype=bool, access=tango.AttrWriteType.READ,
                         label="Overheat")

    ext_lim = attribute(dtype=bool, access=tango.AttrWriteType.READ,
                        label="External Limit")
    
    script = attribute(dtype=bool, access=tango.AttrWriteType.READ,
                        label="Script Running")

    index = attribute(dtype=bool, access=tango.AttrWriteType.READ_WRITE,
                        label="Index Found")
    


    def __init__(self, *args, **kwargs):
        print("Executing __init__")
        super().__init__(*args, **kwargs)
    
    # Initialization
    def init_device(self):
        print("Executing init_device")
        Device.init_device(self)
        self.set_state(DevState.ON)
        self.previous_state = DevState.ON
        self._position = 0.0
        self._enc_pos = 0
        self._update_rate = 0.0
        self._step_rate = 0 # wfm /s, Hz
        self._spc = 0
        self._velocity = 0.0
        self._t0 = 0.0
        self._pos0 = 0.0
        self._status_ctrl = ""
        self._in_pos = False
        self._index = False

        # Queues for communication between threads
        self.write_queue = queue.Queue()
        self.read_queue = queue.Queue()

        # Lock for socket access
        self.socket_lock = Lock()
        self.acq_lock = Lock()

        # Connect to the serial-to-Ethernet device
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.moxa_host, self.moxa_port))
        except socket.error as e:
            print(f"Unable to connect to the moxa device: {e}")
            return

        # Start the reading and writing threads
        self.read_thread = Thread(target=self.read_from_socket, args=(self.sock,), daemon=True)
        self.write_thread = Thread(target=self.write_to_socket, args=(self.sock,), daemon=True)
        
        self.read_thread.start()
        self.write_thread.start()

        print("Initializing PiezoMotorPMDCtrl device...")
        # self.info_stream("PiezoMotorPMDCtrl device initialized")

        
        # Attr update thread
        self._stop_attr_update_thread = Event()
        self._attr_update_thread = Thread(target=self._update_attributes, args=(self,))
        self._attr_update_thread.start()

        self.write_step_rate(self.max_step_rate)
        self._switch_ext_limit()
        
    
    def read_from_socket(self, sock):
        while True:
            try:
                data = sock.recv(1024)
                if data:
                    with self.socket_lock:
                        self.read_queue.put(data.decode('utf-8'))
            except socket.error as e:
                print(f"Socket error: {e}")
                break

    def write_to_socket(self, sock):
        while True:
            if not self.write_queue.empty():
                data = self.write_queue.get()
                with self.socket_lock:
                    try:
                        formatted_request = f"{data}\n"
                        request_bytes = formatted_request.encode('utf-8')
                        sock.sendall(request_bytes)
                    except socket.error as e:
                        print(f"Socket error: {e}")
                        break
    
    def _update_attributes(self, main_thread):
        while not main_thread._stop_attr_update_thread.is_set():
            start_time = time.time()
            # Update the position attribute from hardware
            self._read_hw_enc_pos()
            self._read_ctrl_stat()
            
            #print("Length of the self.write_queue: ", self.write_queue.qsize())
            # Sleep for a specified period before the next update
            time.sleep(ATTR_UPDATE_DELAY)
            self._update_rate = (time.time() - start_time) * 1000
    
    def always_executed_hook(self):
        with self.acq_lock:
            first_double = self._status_ctrl.split(',')[0]
        ctrl_status = self.decode_status_bits(first_double)
        status_str = ", ".join(ctrl_status)
        self.set_status(f"Controller status is: {status_str}")

        if any(elem in self.status_table['d1'] for elem in ctrl_status):
            if self.get_state() != DevState.ALARM:
                self.previous_state = DevState.ALARM
                self.set_state(DevState.ALARM)
        
        
        if 'running' in ctrl_status:
            if self.get_state() != DevState.MOVING:
                self.set_state(DevState.MOVING)
        else:
            if self.get_state() != self.previous_state:
                self.set_state(self.previous_state)
        
        if 'targetMode' in ctrl_status:
            if 'targetReached' not in ctrl_status:
                if self.get_state() != DevState.MOVING:
                    self.previous_state = DevState.MOVING
                    self.set_state(DevState.MOVING)
            else:
                self.set_state(DevState.ON)
                self.previous_state = DevState.ON
        

        self._in_pos = True if 'targetReached' in ctrl_status else False
        self._parked = True if 'parked' in ctrl_status else False
        self._reverse = True if 'reverse' in ctrl_status else False
        self._overheat = True if 'overheat' in ctrl_status else False
        self._ext_lim = True if 'xLimit' in ctrl_status else False
        self._script = True if 'script' in ctrl_status else False
        if 'index' in ctrl_status: self._index = True

        if self._ext_lim:
            self.set_state(DevState.ALARM)

    def _read_hw_enc_pos(self):
        try:
            with self.acq_lock:
                resp = self.send_request("X0E")
                enc_resp = resp.split(':')
                self._enc_pos = int(enc_resp[1].strip())
        except Exception as e:
            print(f"Error reading encoder position: {e}")
            self.error_stream(f"Error reading encoder position: {e}")

    def _read_ctrl_stat(self):
        try:
            with self.acq_lock:
                resp = self.send_request("X0U4")
                status_ctrl_resp = resp.split(':')
                self._status_ctrl = str(status_ctrl_resp[1].strip())
        except Exception as e:
            print(f"Error reading ctrl status: {e}")
            self.error_stream(f"Error reading ctrl status: {e}")
    

    def _read_hw_velocity(self):
        pass

    # Attribute Read/Write Methods
    def read_position(self):
        self._position = self._enc_pos * self.enc_res * self.enc_sign * 1e-3
        return self._position

    def write_position(self, value):
        pos = round(value / (self.enc_res * self.enc_sign * 1e-3))
        self.set_state(DevState.MOVING)
        # self.previous_state = DevState.MOVING
        try:
            with self.acq_lock:
                received_data = self.send_request(f'X0T{pos}')
            if received_data.strip()[-1] == '!':
                self.previous_state = DevState.ALARM   
        except Exception as e:
            self.error_stream(f"Error in write_position: {e}")
            return "Error executing write_position()"
        
        
    def read_enc_pos(self):
        return self._enc_pos

    def read_update_rate(self):
        return self._update_rate
    
    
    def _hex_to_bin(self, hex_string):
        binary_list = [bin(int(char, 16))[2:].zfill(4) for char in hex_string]
        return binary_list
    
    def decode_status_bits(self, hex_string):
        binary_list = self._hex_to_bin(hex_string)
        decoded_status = []

        for col, binary in enumerate(binary_list):
            col_key = f'd{col+1}'
            for bit_pos, bit in enumerate(binary):
                if bit == '1':
                    decoded_status.append(self.status_table[col_key][bit_pos])

        return decoded_status

    def read_status_ctrl(self):
        return self._status_ctrl

    def read_velocity(self):
        return self._velocity

    def write_velocity(self, value):
        self._velocity = value
        self._meas_spc_man()

    def read_step_rate(self):
        return self._step_rate

    def read_spc(self):
        return self._spc

    def write_step_rate(self, value):
        self._step_rate = value
        try:
            with self.acq_lock:
                received_data = self.send_request(f'XY8={self._step_rate}')
            if received_data.strip()[-1] == '!':
                self.previous_state = DevState.ALARM
            with self.acq_lock:
                received_data = self.send_request(f'XH={self._step_rate}')
            if received_data.strip()[-1] == '!':
                self.previous_state = DevState.ALARM  
        except Exception as e:
            self.error_stream(f"Error in write_step_rate: {e}")
            return "Error executing write_step_rate()"

    def read_in_pos(self):
        return self._in_pos

    def read_parked(self):
        return self._parked

    def read_reverse(self):
        return self._reverse

    def read_overheat(self):
        return self._overheat

    def read_ext_lim(self):
        return self._ext_lim

    def read_script(self):
        return self._script

    def read_index(self):
        return self._index
    
    def write_index(self, value):
        self._index = False

    def send_request(self, request):
        try:
            start_time = time.time()
            self.write_queue.put(request)

            while self.read_queue.empty():
                time.sleep(SEND_REQ_DELAY)
                elapsed_time = time.time() - start_time
                if elapsed_time > TIMEOUT:
                    self.set_state(DevState.UNKNOWN)
                    self.previous_state = DevState.UNKNOWN
                    return

            received_data = self.read_queue.get()
            return received_data
            
        except ConnectionError as e:
            print(f"Failed to send data: {e}")
    
    def _switch_ext_limit(self):
        try:
            with self.acq_lock:
                    received_data = self.send_request(f'XY2=2')
            if received_data.strip()[-1] == '!':
                self.previous_state = DevState.ALARM
        except ConnectionError as e:
            print(f"Failed to send data: {e}")


    @command
    def Start(self):
        print("Changing state")
        self.set_state(DevState.UNKNOWN)
        #self.info_stream("Motor started")

    @command
    def Park(self):
        with self.acq_lock:
                received_data = self.send_request('XM4')
        self.set_state(DevState.OFF)
        self.previous_state = DevState.OFF

    @command
    def UnPark(self):
        with self.acq_lock:
                received_data = self.send_request('XM2')
        self.set_state(DevState.ON)
        self.previous_state = DevState.ON

    @command
    def Stop(self):
        try:
            with self.acq_lock:
                received_data = self.send_request('X0S')
            self.set_state(DevState.ON)
            #self.info_stream("Motor stopped")   
            
        except Exception as e:
            self.error_stream(f"Error in SendRequest: {e}")
            return "Error processing request"
        
    @command
    def ResetError(self):
        self.set_state(DevState.ON)
        self.previous_state = DevState.ON

    @command(dtype_in=tango.DevDouble, dtype_out=tango.DevDouble, doc_in="Motion span to try")
    def CheckVelocity(self, span):
        self._velocity = 0.0
        self._check_velocity_thread = Thread(target=self._check_velocity, args=(span,))
        self._check_velocity_thread.start()
        return self._velocity
        

    def _check_velocity(self, span):
        # Velocity estimation
        self._t0 = time.time()
        self._pos0 = self._position
        target = self._position + span
        self.write_position(target)
        while abs(target - self._position) > POS_TOLERANCE:
            # print(f'motor moving, positoin is: {self._position}, target is: {target}')
            self.read_position()
            time.sleep(0.001)
        elapsed_time = time.time() - self._t0
        pos_diff = self._position - self._pos0
        self._velocity = pos_diff / elapsed_time
        target = self._pos0
        self.write_position(self._pos0)
        while abs(target - self._position) > POS_TOLERANCE:
            # print(f'motor moving, positoin is: {self._position}, target is: {target}')
            self.read_position()
            time.sleep(0.001)
        #return self._velocity

    @command
    def GetSPC(self):
        self._spc = 0
        self._meas_spc_thread = Thread(target=self._meas_spc_man)
        self._meas_spc_thread.start()
        self._spc = 5
        # self._meas_spc_man(self)

    def _meas_spc_man(self):
        enc0 = self._enc_pos
        # print('Jogging forward with JOG_STEPS_N: ', JOG_STEPS_N)
        try:
            spc = self.SendRequest(f'XJ{JOG_STEPS_N}')
            if spc.strip()[-1] == '!':
                self.previous_state = DevState.ALARM
        except ConnectionError as e:
            print(f"Failed to send data: {e}")
  
        time.sleep(1)
        while self.get_state() == tango.DevState.MOVING:
                time.sleep(0.1)
        enc_diff = abs(self._enc_pos - enc0)

        # print('Jogging backward with JOG_STEPS_N: ', JOG_STEPS_N)
        try:
            spc = self.SendRequest(f'XJ=-{JOG_STEPS_N}')
            if spc.strip()[-1] == '!':
                self.previous_state = DevState.ALARM
        except ConnectionError as e:
            print(f"Failed to send data: {e}")
        
        time.sleep(1)
        while self.get_state() == tango.DevState.MOVING:
                time.sleep(0.1)
        X = round(enc_diff / JOG_STEPS_N)
        # print(f'Number of encoder steps per jog step is: {X}, which equals to {X * 50} nm jog step')
        # To keep a certain speed e.g. 1 mm/s one has to adjust the wfm-step rate as:
        # print(f'Rate for 1 mm/s is : {round(1000000 / (X * self.enc_res))}')
        if abs(self._velocity) > 0:
            step_rate = round((self._velocity * 1000) / (X * self.enc_res))
            self.write_step_rate(step_rate)
        self._spc = round(65546 * 4 / X)
        


    def delete_device(self):
        self.info_stream("Stopping motor and cleaning up resources before exiting.")
        try:
            self.Stop()
        except Exception as e:
            self.error_stream(f"Error stopping motor during delete_device: {e}")
        finally:
            self._socket_thread.stop()
            self._socket_thread.join()

            self._stop_attr_update_thread.set()
            self._attr_update_thread.join()

            Device.delete_device(self)

    @command(dtype_in=tango.DevString, dtype_out=tango.DevString, doc_in="Request string to send", doc_out="Response string received")
    def SendRequest(self, request):
        try:
            with self.acq_lock:
                received_data = self.send_request(request)
                return str(received_data)
            
        except Exception as e:
            self.error_stream(f"Error in SendRequest: {e}")
            return "Error processing request"

# Device class
class PiezoMotorPMDCtrlClass(DeviceClass):
    # Device Class Properties
    class_property_list = {}

    # Device Class Properties
    device_property_list = {
        'moxa_host': [tango.DevString, "IP Address of the Moxa IP/serial hub", []],
        'moxa_port': [tango.DevShort, "Port of the Moxa IP/serial hub", []],
        'moxa_reconnect_delay': [tango.DevFloat, "Timeout before reconnecting attempt", []],   
    }

    # Device Class Commands
    cmd_list = {
        'Start': [[tango.DevVoid, "Start the motor"], [tango.DevVoid, ""]],
        'Stop': [[tango.DevVoid, "Stop the motor"], [tango.DevVoid, ""]],
        'ResetError': [[tango.DevVoid, "Reset errors"], [tango.DevVoid, ""]],
        'SendRequest': [[tango.DevString, "Send a request"], [tango.DevString, ""]],
        'Park': [[tango.DevVoid, "Park the motor"], [tango.DevVoid, ""]],
        'UnPark': [[tango.DevVoid, "Unpark the motor"], [tango.DevVoid, ""]],
        'CheckVelocity': [[tango.DevDouble, "Estimates Velocity"], [tango.DevDouble, ""]],
        'GetSPC': [[tango.DevVoid, "Get SPC"], [tango.DevVoid, ""]],
    }

    # Device Class Attributes
    attr_list = {
        'position': [[tango.DevDouble, tango.SCALAR, tango.READ_WRITE]],
        'enc_pos': [[tango.DevLong, tango.SCALAR, tango.READ]],
        'update_rate': [[tango.DevDouble, tango.SCALAR, tango.READ]],
        'velocity': [[tango.DevDouble, tango.SCALAR, tango.READ_WRITE]],
        'step_rate': [[tango.DevLong, tango.SCALAR, tango.READ_WRITE]],
        'spc': [[tango.DevLong, tango.SCALAR, tango.READ]],
        'status_ctrl': [[tango.DevString, tango.SCALAR, tango.READ]],
        'in_pos': [[tango.DevBoolean, tango.SCALAR, tango.READ]],
        'parked': [[tango.DevBoolean, tango.SCALAR, tango.READ]],
        'reverse': [[tango.DevBoolean, tango.SCALAR, tango.READ]],
        'overheat': [[tango.DevBoolean, tango.SCALAR, tango.READ]],
        'ext_lim': [[tango.DevBoolean, tango.SCALAR, tango.READ]],
        'script': [[tango.DevBoolean, tango.SCALAR, tango.READ]],
        'index': [[tango.DevBoolean, tango.SCALAR, tango.READ_WRITE]],
    }

# Run the server
if __name__ == '__main__':
    import sys
    tango.server.run((PiezoMotorPMDCtrl,))

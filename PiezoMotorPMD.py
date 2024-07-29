import socket
import queue
import time
from threading import Thread, Lock, Event, Condition

import tango
from tango import DeviceClass, DevState, DevFailed
from tango.server import Device, attribute, command, device_property

COMM_LATENCY = 0.001  # Communication latency
ATTR_UPDATE_DELAY = 0.0001  # Attribute update period
SEND_REQ_DELAY = 0.001
MAX_RESP_N = 1000  # Maximum number of responses to store




class PiezoMotorPMDCtrl(Device):
    # Device Properties
    moxa_host = device_property(dtype=str, default_value="b-softimax-moxa-0")
    moxa_port = device_property(dtype=int, default_value=4001)
    moxa_reconnect_delay = device_property(dtype=float, default_value=5)

    # Device Attributes
    position = attribute(dtype=float, access=tango.AttrWriteType.READ_WRITE,
                         label="Position", unit="micron", format="%.2f")
    
    enc_pos = attribute(dtype=int, access=tango.AttrWriteType.READ,
                         label="EncPos", unit="counts", format="%10d")

    update_rate = attribute(dtype=float, access=tango.AttrWriteType.READ,
                         label="UpdateRate", unit="ms", format="%.1f")

    velocity = attribute(dtype=float, access=tango.AttrWriteType.READ_WRITE,
                         label="Velocity", unit="micron/s", format="%.2f")

    status_ctrl = attribute(dtype=str, access=tango.AttrWriteType.READ,
                         label="StatusCtrl")

    def __init__(self, *args, **kwargs):
        print("Executing __init__")
        super().__init__(*args, **kwargs)
    
    # Initialization
    def init_device(self):
        print("Executing init_device")
        Device.init_device(self)
        self.set_state(DevState.ON)
        self._position = 0.0
        self._enc_pos = 0
        self._update_rate = 0.0
        self._velocity = 0.0
        self._status_ctrl = ""

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

        self._connection_lost = False
        print("Initializing PiezoMotorPMDCtrl device...")
        # self.info_stream("PiezoMotorPMDCtrl device initialized")

        
        # Attr update thread
        self._stop_attr_update_thread = Event()
        self._attr_update_thread = Thread(target=self._update_attributes, args=(self,))
        self._attr_update_thread.start()
        

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
        return self._position
        
    def read_enc_pos(self):
        return self._enc_pos

    def read_update_rate(self):
        return self._update_rate

    def read_status_ctrl(self):
        return self._status_ctrl

    def write_position(self, value):
        self._position = value
        self.info_stream(f"Motor position set to {value}")

    def read_velocity(self):
        return self._velocity

    def write_velocity(self, value):
        self._velocity = value
        self.info_stream(f"Motor velocity set to {value}")

    def send_request(self, request):
        try:
            self.write_queue.put(request)

            while self.read_queue.empty():
                time.sleep(SEND_REQ_DELAY)

            received_data = self.read_queue.get()
            return received_data
            
        except ConnectionError as e:
            print(f"Failed to send data: {e}")


    @command
    def Start(self):
        self.set_state(DevState.MOVING)
        self.info_stream("Motor started")
        # Here you would add the actual code to start the motor

    @command
    def Stop(self):
        self.set_state(DevState.ON)
        self.info_stream("Motor stopped")
        # Here you would add the actual code to stop the motor

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
        'SendRequest': [[tango.DevString, "Send a request"], [tango.DevString, ""]],
    }

    # Device Class Attributes
    attr_list = {
        'position': [[tango.DevDouble, tango.SCALAR, tango.READ_WRITE]],
        'enc_pos': [[tango.DevLong, tango.SCALAR, tango.READ]],
        'update_rate': [[tango.DevLong, tango.SCALAR, tango.READ]],
        'velocity': [[tango.DevDouble, tango.SCALAR, tango.READ_WRITE]],
        'status_ctrl': [[tango.DevString, tango.SCALAR, tango.READ]],
    }

# Run the server
if __name__ == '__main__':
    import sys
    tango.server.run((PiezoMotorPMDCtrl,))

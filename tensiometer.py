#!/usr/bin/env python
'''
Python script to interface with an RS485 UGT tensiometer using serial to RS485 converter.
'''
from io import TextIOWrapper
import os
import traceback
import time
import serial
import serial.tools.list_ports

# CONFIG
SAVE_TO_CSV = True
CSV_FILE_PATH = "tensiometer.csv"

# Measurement channel constants
CHANNEL_SERVICE_1 = 1
CHANNEL_TEMPERATURE = 2
CHANNEL_PRESSURE = 3
CHANNEL_SERVICE_2 = 4

BUS_ADDR_ALL = 0 #Value of bus address for OP for all connected FRTs

## Constants for tensiometer
BIT_CONVERSION_FACTOR = 16384
UGT_CONVERSION_FACTOR_32768 = 32768
UGT_CONVERSION_FACTOR_31 = 31


def open_csv(csv_file_name):
    '''
    Create csv file and add headers
    '''
    # Check if file exist to add column headings
    if not os.path.isfile(csv_file_name):
        csv_fid = open(csv_file_name, 'w', encoding="utf-8") # Open file for writing
        csv_fid.write("Timestamp,Temperature (Digital),Pressure (Digital),Temperature (C),Pressure (kPa),Suction (kPa)\n") # Allocate column names
    else:
        csv_fid = open(csv_file_name, 'a', encoding="utf-8") # Open file for appending
    return csv_fid


def get_com_ports_list():
    '''
    Gets a list of comm ports
    '''
    return(sorted(serial.tools.list_ports.comports()))


def print_ports_list(ports_list):
    '''
    Prints the list of detected ports
    '''
    i = 0
    print("\r\nPlease select the port where tensiometer is connected to.")
    for device, desc, hwid in ports_list:
        print("\t{} )   {} - {} [{}]".format(i, device, desc, hwid))
        i = i + 1
    print("\t---")
    print("\t{} )   {}".format("r", "Rescan ports"))


def calculate_temperature_compensated_reference_pressure(temperature:float, p_0:float, c_t1:float, c_t2:float):
    '''
    Calculates temperature compensated reference pressure
    '''
    return c_t1 * temperature**2 + c_t2 * temperature + p_0


def calculate_tesion(pressure, p_compensated_ref):
    '''
    Calculate tension from temperature compensated reference and measured pressure
    '''
    return p_compensated_ref - pressure

class UGTTensiometer():
    '''
    Class object for tensiometer.
    '''

    # Class variables
    _ser = None                     # Reference to serial device tensiometer is attached to
    _serial_number = None           # Serial number of tensiometer
    _temperature_bus_addr = None    # Bus address for temperature
    _pressure_bus_addr = None       # Bus address for pressure
    _c_t1:float                     # Temperature coefficient 1, C_t1, [kPa/C^2]
    _c_t2:float                     # Temperature coefficient 2, C_t2 [kPa/C]
    _p_0:float                      # Reference pressure at 0 degrees celsius [kPa]

    # FRT calibration values
    _calibration = None

    def __init__(self, ser:serial.Serial, serial_number:str, t1_coeff:float, t2_coeff:float, p0_ref:float, \
            bus_temp:str=None, bus_pressure:str=None):
        if ser is not None:
            self.attach(ser)
        self._serial_number = serial_number
        self._c_t1 = t1_coeff
        self._c_t2 = t2_coeff
        self._p_0 = p0_ref
        self._temperature_bus_addr = bus_temp
        self._pressure_bus_addr = bus_pressure

    @staticmethod
    def configure_serial(ser:serial.Serial):
        '''
        Wrapper for configuration of serial object
        Parameters for the serial interface (UART) are 9600 8N1
        (9600 baud, 8 data bits, no parity, 1 stop bit)
        '''
        ser.baudrate = 9600
        ser.bytesize = serial.EIGHTBITS
        ser.parity = serial.PARITY_NONE
        ser.stopbits = serial.STOPBITS_ONE
        ser.timeout = 5
        if ser.isOpen() is False:
            ser.open()

    @staticmethod
    def send_command(ser:serial.Serial, message):
        '''
        Serial write wrapper to package command and flush before sending command.
        '''
        ser.write(F"{message}\r".encode())
        ser.flush()

    @staticmethod
    def get_reply(ser:serial.Serial):
        '''
        Wrapper for serial read_until(), returns a string without "\\r"
        '''
        return ser.read_until(b"\r").decode().rstrip()

    @staticmethod
    def ack_ok(ser:serial.Serial):
        '''
        Check if sensor returned OK, returns True or False
        '''
        ok_ack = UGTTensiometer.get_reply(ser)
        return ok_ack == "OK"

    @staticmethod
    def request_serial(ser:serial.Serial):
        '''
        Send get serial number command, need to open bus OPx first.
        '''
        UGTTensiometer.send_command(ser, "GSN")

    @staticmethod
    def request_bus_address(ser:serial.Serial, channel):
        '''
        Send get bus address command, need to open bus using OPx first.
        '''
        if channel < 1 or channel > 4:
            raise ValueError("Incorrect channel value")
        UGTTensiometer.send_command(ser, F"GAD{channel}")

    @staticmethod
    def open_bus(ser:serial, bus_address):
        '''
        Send open bus command, returns True or False
        '''
        UGTTensiometer.send_command(ser, F"OP{bus_address}")
        return UGTTensiometer.ack_ok(ser)

    @staticmethod
    def get_raw(ser:serial.Serial):
        '''
        Send request measurement command, need to open bus first using OPx
        '''
        UGTTensiometer.send_command(ser, "GN")
        return UGTTensiometer.get_reply(ser)

    @staticmethod
    def change_channel_bus(ser:serial.Serial, channel, new_bus_address):
        '''
        Change bus address, Returns True if successfull.
        Need to open correct bus to FRT with OPx first.
        '''
        UGTTensiometer.send_command(ser, F"SAD{channel} {new_bus_address}")
        return UGTTensiometer.ack_ok

    @staticmethod
    def raw_to_celsius(digital_value: float):
        '''
        Convert measured temperature digital output to celsius
        '''
        if isinstance(digital_value, str):
            digital_value = float(digital_value)
        # return (digital_value - BIT_CONVERSION_FACTOR) / 320
        return (digital_value - UGT_CONVERSION_FACTOR_32768) / 100

    @staticmethod
    def raw_to_kpa(digital_value: float):
        '''
        Convert measured pressure digital output to kPa
        '''
        if isinstance(digital_value, str):
            digital_value = float(digital_value)
        return (((digital_value - BIT_CONVERSION_FACTOR) * UGT_CONVERSION_FACTOR_31 / UGT_CONVERSION_FACTOR_32768) - 1) * 100

    def attach(self, ser:serial.Serial):
        '''
        Reference a serial port for the tensiometer to communicate on.
        '''
        if not isinstance(ser, serial.Serial):
            raise TypeError("Tensiometer needs to be attached to a valid serial object")
        self._ser = ser
        if self._ser.isOpen() is False:
            self._ser.open()

    @property
    def serial_number(self):
        '''
        Returns string representation of the
        '''
        return self._serial_number

    @serial_number.setter
    def serial_number(self, serial_number):
        '''
        Change temperature bus address
        '''
        if serial_number is not None and isinstance(serial_number, str):
            serial_number = serial_number.strip()
        self._serial_number = serial_number

    @property
    def temperature_bus_addr(self):
        '''
        Returns string representation of temperature bus address
        '''
        return str(self._temperature_bus_addr)

    @temperature_bus_addr.setter
    def temperature_bus_addr(self, address):
        '''
        Change temperature bus address
        '''
        if address is not None and isinstance(address, str):
            address = address.strip()
        self._temperature_bus_addr = address

    @property
    def pressure_bus_addr(self):
        '''
        Returns string representation of pressure bus address
        '''
        return str(self._pressure_bus_addr)

    @pressure_bus_addr.setter
    def pressure_bus_addr(self, address):
        '''
        Change pressure bus address
        '''
        if address is not None and isinstance(address, str):
            address = address.strip()
        self._pressure_bus_addr = address

    def update_temperature_bus_address(self):
        '''
        Gets and update the temperature bus address.
        '''
        if self._serial_number is None:
            raise RuntimeError("Device does not have a serial number")
        if UGTTensiometer.open_bus(self._ser, self._serial_number):
            UGTTensiometer.request_bus_address(self._ser, CHANNEL_TEMPERATURE)
            address = UGTTensiometer.get_reply(self._ser)
            if address == "":
                return False
        self._temperature_bus_addr = address
        return True

    def update_pressure_bus_address(self):
        '''
        Gets and update the pressure bus address.
        '''
        if self._serial_number is None:
            raise RuntimeError("Device does not have a serial number")
        if UGTTensiometer.open_bus(self._ser, self._serial_number):
            UGTTensiometer.request_bus_address(self._ser, CHANNEL_PRESSURE)
            address = UGTTensiometer.get_reply(self._ser)
            if address == "":
                return False
        self._pressure_bus_addr = address
        return True

    def get_temperature_raw(self):
        '''
        Get digital raw temperature reading from sensor
        '''

        if self._temperature_bus_addr is None:
            raise RuntimeError("Temperature bus address not set")

        if UGTTensiometer.open_bus(self._ser, self._temperature_bus_addr) is False:
            raise RuntimeError("Unable to communicate with sensor")

        return UGTTensiometer.get_raw(self._ser)

    def get_pressure_raw(self):
        '''
        Get digital raw pressure reading from sensor
        '''

        if self._pressure_bus_addr is None:
            raise RuntimeError("Pressure bus address not set")

        if UGTTensiometer.open_bus(self._ser, self._pressure_bus_addr) is False:
            raise RuntimeError("Unable to communicate with sensor")

        return UGTTensiometer.get_raw(self._ser)

    def get_temperature(self):
        '''
        Get temperature reading from tensiometer in celsius.
        '''
        value = UGTTensiometer.get_temperature_raw(self)
        return UGTTensiometer.raw_to_celsius(value)

    def get_pressure(self):
        '''
        Get pressure reading from tensiometer in kPa.
        '''
        value = UGTTensiometer.get_pressure_raw(self)
        return UGTTensiometer.raw_to_kpa(value)

    def get_suction(self, temperature:float, pressure:float):
        '''
        Return the tension as calculated from tensiometer calibrated info.
        '''
        p_ref = calculate_temperature_compensated_reference_pressure(temperature=temperature,
                p_0=self._p_0, c_t1=self._c_t1, c_t2=self._c_t2)
        return p_ref - pressure


def serial_session(serial_obj: serial):
    '''
    Serial session to interface with sensor
    '''
    while True:
        user_input = input("Command to send: ").strip().upper()
        if user_input in ("X", "EXIT"):
            break
        UGTTensiometer.send_command(serial_obj, user_input)
        time.sleep(0.1)
        while serial_obj.in_waiting:
            message = UGTTensiometer.get_reply(serial_obj)
            print(message)


def sample_single(tensiometer:UGTTensiometer, csv_file:TextIOWrapper=None):
    '''
    Main function single sampling of UGT tensiometer
    '''
    time_now_local_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())

    temperature_raw = tensiometer.get_temperature_raw() # temp in digital value
    temperature = UGTTensiometer.raw_to_celsius(temperature_raw) # temp in celsius
    pressure_raw = tensiometer.get_pressure_raw() # pressure in digital value
    pressure = UGTTensiometer.raw_to_kpa(pressure_raw) # pressure in kPa
    suction = tensiometer.get_suction(temperature, pressure)

    print(F"Temperature: {temperature_raw} (Digital), {temperature} (C)")
    print(F"Pressure: {pressure_raw} (Digital), {pressure} (kPa)")
    print(F"Suction: {suction} (kPa)")

    if SAVE_TO_CSV and csv_file is not None:
        csv_file.write(F"{time_now_local_str},{temperature_raw},{pressure_raw},{temperature},{pressure},{suction}\n")
        # csv_file.flush()


def sample_loop(tensiometer:UGTTensiometer, csv_file:TextIOWrapper=None):
    '''
    Prompts user for sampling interval in seconds to sample at specified intervals.
    '''
    interval = int(input("Time interval between sampling in seconds: "))
    print(F"Sampling interval set at {interval} seconds")
    while True:
        sample_single(tensiometer, csv_file)
        print(F"{time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())} - Waiting {interval} seconds")
        time.sleep(interval)


if __name__ == '__main__':
    ports = get_com_ports_list()
    print_ports_list(ports)

    user_port_selection = -1
    while(user_port_selection < 0 or user_port_selection >= len(ports)):
        user_port_selection = input("Enter a number: ").strip()

        if user_port_selection == "r":
            ports = get_com_ports_list()
            print_ports_list(ports)
            user_port_selection = -1
        elif user_port_selection == "x":
            exit()
        else:
            user_port_selection = int(user_port_selection)

    port_address = ports[user_port_selection].device

    tensiometer_comm = serial.Serial(port_address)
    UGTTensiometer.configure_serial(tensiometer_comm)

    try:
        # Create tensiometer object and update bus addresses for the sensor
        tensiometer = UGTTensiometer(ser=tensiometer_comm, serial_number="202000024", \
                t1_coeff=-0.1718, t2_coeff=28.7730, p0_ref=1925.25)
        # tensiometer = UGTTensiometer(ser=tensiometer_comm, serial_number="20180005", \
        #         t1_coeff=-0.1173, t2_coeff=24.595, p0_ref=1462.1)
        tensiometer.update_temperature_bus_address()
        tensiometer.update_pressure_bus_address()

        # serial_session(tensiometer_comm)
        if SAVE_TO_CSV:
            csv_file = open_csv(CSV_FILE_PATH)
        else:
            csv_file = None

        # sample_single(tensiometer, csv_file)
        sample_loop(tensiometer, csv_file)

    except KeyboardInterrupt:
        print("")
    except Exception:
        traceback.print_exc()
        # print(sys.exc_info()[0])
    finally:
        # Cleaning up
        if tensiometer_comm is not None and tensiometer_comm.isOpen():
            tensiometer_comm.close()
        try:
            if SAVE_TO_CSV:
                # csv_file.write("\r\n")
                if not csv_file.closed:
                    csv_file.close()
        except (NameError):
            pass

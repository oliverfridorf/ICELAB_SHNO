'''
This files contains the classes for the different intruments that are 
used for the SHNO measurement setup. 

Requirements: 

    Python packages
- pyvisa (Note: Instruments connected using TCPIP do not appear in RM list)
- numpy
- matplotlib
- tqdm
- RsInstrument
- serial
- glob
- Jupyter Notebook

    Drivers
- NI-VISA
- R&S HO720 (Manual install)
- VELMAX VXM USB Serial+

'''

# Import required packages
import pyvisa
import numpy as np
import matplotlib.pyplot as plt
from RsInstrument import *
from tqdm.notebook import tqdm
import time
import serial
import sys
import glob
from IPython.display import clear_output

# Ininitilise the resource manager to be used for 
# starting communication in each intrument class
rm = pyvisa.ResourceManager()

# ═══════════════════════════════════
# ════ Keysight E36300 PSU═══════════
# ═══════════════════════════════════

class keysight36300:
    """Class for interfacing with the Keysight E36300 series power supply."""
    
    def __init__(self, addr="TCPIP::192.168.0.3::inst0::INSTR"):
        """Initialize connection to the power supply."""
        try:
            self.psu = rm.open_resource(addr)
            self.psu.write("*RST")
            self.psu.timeout = 5000  # Set timeout to 5s
            idn = self.psu.query('*IDN?')
            print("IDN:", idn)
        except:
            print("Failed to connect")
    
    def reset(self):
        """Reset the power supply to its default settings."""
        self.psu.write("*RST")
    
    def close(self):
        """Close the connection to the power supply."""
        self.psu.close()
        print("Connection closed")
    
    def beeper(self):
        """Activate the power supply's beeper."""
        self.psu.write("SYST:BEEP:STAT 1")
        self.psu.write("SYST:BEEP")

    def set_output(self, V, I, ch=1, turn_on=False):
        """Set voltage and current limits for a given channel.
        
        Args:
            V (float): Voltage limit.
            I (float): Current limit (minimum 2e-3 A).
            ch (int): Channel number (default is 1).
            turn_on (bool): If True, turns on the output.
        """
        self.psu.write(f'SOURCE:CURRENT {I}, (@{ch})')
        self.psu.write(f'SOURCE:VOLTAGE {V}, (@{ch})')
        if turn_on:
            self.psu.write(f'OUTP ON, (@{ch})')
    
    def measure(self, ch=1):
        """Measure the voltage and current on a specified channel.
        
        Args:
            ch (int): Channel number (default is 1).
        
        Returns:
            tuple: Measured voltage and current.
        """
        voltage = float(self.psu.query(f'MEASURE:VOLTAGE? CH{ch}'))
        current = float(self.psu.query(f'MEASURE:CURRENT? CH{ch}'))
        return voltage, current
    
    def on(self, ch=[]):
        """Turn on specified channels.
        
        Args:
            ch (list): List of channels to turn on. Defaults to all channels.
        """
        if ch == []:
            ch = [1, 2, 3]
        try:
            for i in ch:
                self.psu.write(f'OUTP ON, (@{i})')
        except:
            self.psu.write(f'OUTP ON, (@{ch})')
    
    def off(self, ch=[]):
        """Turn off specified channels.
        
        Args:
            ch (list): List of channels to turn off. Defaults to all channels.
        """
        if ch == []:
            ch = [1, 2, 3]
        try:
            for i in ch:
                self.psu.write(f'OUTP OFF, (@{i})')
        except:
            self.psu.write(f'OUTP OFF, (@{ch})')


# ═══════════════════════════════════
# ════ R&S FVS Signal Analyser ══════
# ═══════════════════════════════════

class rs_fsv:
    """Class for interfacing with the R&S FSV Signal Analyzer."""
    
    def __init__(self, address='tcpip::192.168.0.2::inst0::INSTR'):
        """Initialize connection to the signal analyzer."""
        try:
            self.inst = rm.open_resource(address)
            self.inst.timeout = 25000  # Set timeout to 25s
            idn = self.inst.query('*IDN?')
            self.inst.write('*RST')
            print(idn)
            self.inst.write('SYST:DISP:UPD ON')  # Turn on the display
        except Exception as e:
            print(f"Connection failed: {e}")
    
    def get_trace(self, avg_count=0):
        """Retrieve the trace data from the signal analyzer.
        
        Args:
            avg_count (int): Number of averages. If 0, averaging is disabled.
        
        Returns:
            np.array: Trace data as an array of float values.
        """
        if avg_count != 0:
            self.inst.write("INIT:CONT OFF")
            self.inst.write(f"AVER:COUN {avg_count}")
            self.inst.write("AVER:STAT ON")
            self.inst.write("INIT;*WAI")
        else:
            self.inst.write("AVER:STAT OFF")
        data = self.inst.query('TRAC? TRACE1').split(',')
        return np.array([float(x) for x in data])
    
    def show_screen(self, start_f=150E6, stop_f=300E6, bandwidth=0, points=2048):
        """Configure and display the signal analyzer settings.
        
        Args:
            start_f (float): Start frequency in Hz.
            stop_f (float): Stop frequency in Hz.
            bandwidth (float): Resolution bandwidth. If 0, set to AUTO.
            points (int): Number of sweep points.
        """
        self.reset_instrument()
        self.inst.write(f"SENSe:FREQuency:STOP {stop_f}")
        self.inst.write(f"SENSe:FREQuency:STARt {start_f}")
        if bandwidth != 0:
            self.inst.write(f"SENSe:BANDwidth {bandwidth}")
        self.inst.write(f"SENSe:SWEep:POINts {points}")
        self.inst.write("AVER:COUN 20")
        self.inst.write("AVER:STAT ON")
    
    def reset_instrument(self):
        """Reset the signal analyzer to its default state."""
        try:
            self.inst.write("*RST")
            self.inst.write("*CLS")
            self.inst.write("INST SAN")
            self.inst.write('SYST:DISP:UPD ON')
        except:
            print("Reset failed")
    
    def close(self):
        """Close the connection to the signal analyzer."""
        self.inst.write('*RST')
        self.inst.close()
        print("Connection closed")


# ═══════════════════════════════════
# ════ R&S HMP 2030 PSU ═════════════
# ═══════════════════════════════════
class rs_hmp_psu:
    """Class for interfacing with the R&S HMP 2030 Power Supply Unit (PSU)."""
    
    def __init__(self, address='ASRL4::INSTR'):
        """Initialize connection to the PSU."""
        try:
            self.inst = rm.open_resource(address)
            self.inst.write("*RST")
            self.inst.write("*WAI")
            self.inst.timeout = 5000  # Set timeout to 5s
            idn = self.inst.query("*IDN?")
            print(idn)
        except Exception as e:
            print(f"Connection failed: {e}")
    
    def close(self):
        """Close the connection to the PSU."""
        self.inst.close()
        print("Connection closed")
    
    def beeper(self):
        """Activate the PSU's beeper."""
        self.inst.write("SYSTEM:BEEPER")
    
    def on(self, ch=[], beep=True):
        """Turn on specified channels.
        
        Args:
            ch (list): List of channels to turn on.
            beep (bool): If True, activate the beeper.
        """
        for i in ch:
            self.inst.write(f'INSTRUMENT:NSELECT {i}')
            self.inst.write(f'OUTPUT:SELECT 1')   
        self.inst.write('OUTP:GEN 1')
        if beep:
            self.inst.write("SYSTEM:BEEPER")
    
    def off(self, ch=[]):
        """Turn off specified channels.
        
        Args:
            ch (list): List of channels to turn off. Defaults to all channels.
        """
        self.inst.write('OUTP:GEN 0')
        if ch == []:
            ch = [1, 2, 3]
        for i in ch:
            self.inst.write(f'INSTRUMENT:NSELECT {i}')
            self.inst.write(f'OUTPUT:SELECT 0')
    
    def set_channel_output(self, V, I, ch=1, turn_on=False):
        """Set voltage and current for a specific channel.
        
        Args:
            V (float): Voltage level.
            I (float): Current limit.
            ch (int): Channel number.
            turn_on (bool): If True, turns on the output.
        """
        self.inst.write(f'INSTRUMENT:NSELECT {ch}')
        self.inst.write(f'VOLT {V}')
        self.inst.write(f'CURR {I}')
        if turn_on:
            self.on(ch=[ch])
    
    def set_all_output(self, V, I, turn_on=False):
        """Set voltage and current for all channels.
        
        Args:
            V (float): Voltage level.
            I (float): Current limit.
            turn_on (bool): If True, turns on all outputs.
        """
        for i in range(1, 4):
            self.inst.write(f'INSTRUMENT:NSELECT {i}')
            self.inst.write(f'APPLY {V},{I}')
        if turn_on:
            self.on(ch=[1, 2, 3], beep=False)
        else:
            self.off(ch=[1, 2, 3])
    
    def measure(self, ch=[1]):
        """Measure the voltage and current on specified channels.
        
        Args:
            ch (list): List of channels to measure.
        
        Returns:
            tuple: Measured voltages and currents.
        """
        voltage, current = [], []
        for i in ch:
            self.inst.write(f'INSTRUMENT:NSELECT {i}')
            voltage.append(float(self.inst.query('MEAS:VOLT?')))
            current.append(float(self.inst.query('MEAS:CURR?')))
        return voltage, current
    
    def field_strength(self):
        """Calculate the magnetic field strength based on the current measurement.
        
        Returns:
            float: Calculated magnetic field strength in Tesla.
        """
        _, current = self.measure([1, 2, 3])
        current = sum(current) / 3
        Bz = round(0.1525 * current, 3)  # Scaling factor based on magnet specifications
        return Bz
    
    def set_calibrated_current(self, I, magnet_resistance=2):
        """Set the PSU to output a calibrated current based on magnet resistance.
        
        Args:
            I (float): Desired current.
            magnet_resistance (float): Initial resistance estimate (default is 2 Ohms).
        
        Returns:
            float: Final estimated magnet resistance.
        """
        voltage = magnet_resistance * I
        self.set_all_output(voltage, 5, turn_on=True)
        time.sleep(1)
        v, curr = self.measure()
        
        while not np.allclose(I - curr[0], 0, atol=0.002):
            self.set_all_output(voltage, 5, turn_on=True)
            time.sleep(1)
            v, curr = self.measure()
            magnet_resistance = v[0] / curr[0]
            voltage += magnet_resistance * (I - curr[0])
        
        return magnet_resistance


# ═══════════════════════════════════
# ════ VELMEX VXM MOTOR DRIVER ══════
# ═══════════════════════════════════            '
class Velmex_VXM:
    '''Degree measurement is relative to the starting position of the stage'''

    def __init__(self, com_port="COM3", baudrate=9600):
        """
        Initializes communication parameters and connects to the motor controller.

        Args:
            com_port (str): Communication port (default "COM3").
            baudrate (int): Baud rate for communication (default 9600).
        """
        self.com_port = com_port
        self.baudrate = baudrate
        self.vxm = None
        self.max_retries = 1
        self.connect_and_verify()

    def connect_and_verify(self):
        """
        Attempts to connect to the controller, reset it, and verify its status.

        Retries connection up to max_retries times, printing status and errors.
        """
        for attempt in range(self.max_retries):
            try:
                self.vxm = serial.Serial(self.com_port, self.baudrate, timeout=1)
                self.vxm.write(b'res')
                self.vxm.write(b'C,F')
                self.vxm.write(b'C,V')
                time.sleep(0.1)
                resp = self.vxm.readlines()
                status = resp[0].decode(encoding="utf-8")
                print(f"VXM status {status}")
                return
            except Exception as e:
                print(f"Connection failed (attempt {attempt + 1}): {e}")
                time.sleep(0.2)
        print("Connection failed after multiple attempts.")
                
    def get_pos(self):
        """
        Retrieves the current motor position in degrees.

        Returns:
            float: Current position in degrees, rounded to 4 decimal places.
        """
        self.vxm.write(b'C,X')
        pos = self.vxm.readlines()[0].decode(encoding="utf-8")
        pos = round(int(pos.split("\r")[0].split("^")[-1].split("X")[-1])*0.0125*-1, 4)
        return pos
    
    def move_degree(self, degree, speed=750, safe_time=True):
        """
        Moves the motor by a specified number of degrees.

        Args:
            degree (float): Degrees to move. (Minimum step is 0.0125 degrees).
            speed (int): Speed of movement (default 750).
            safe_time (bool): Wait for movement to complete if True (default True).
        """
        steps = int(degree / 0.0125) * -1
        if steps == 0:
            print("Step too small")
            return
        command = f"C,S1M{speed},I1M{steps},R"
        self.vxm.write(command.encode())
        time.sleep(0.1 * np.abs(degree))
        if safe_time:
            self.vxm.write(b'C,V')
            status = self.vxm.readlines()[0].decode(encoding="utf-8")
            while status == "B":
                self.vxm.write(b'C,V')
                status = self.vxm.readlines()[0].decode(encoding="utf-8")
                print("Status:", status)
        
    def close(self):
        """
        Closes the connection to the motor controller.
        """
        self.vxm.write(b'C,Q')
        serial.Serial.close(self.vxm)
        print("Connection closed")

# ═══════════════════════════════════
# ════ Keithley 2450 SMU ════════════
# ═══════════════════════════════════    
class keithley2450:
    def __init__(self, addr = 0):
        """
        Opens a connection to the Keithley 2450 instrument and configures basic settings.

        Args:
            addr (int or str): The address of the instrument (default 0).
        """
        # keithley left: USB0::0x05E6::0x2450::04393544::INSTR 
        # keithley right: USB0::0x05E6::0x2450::04502593::INSTR
        if addr == 0:
            self.address = "USB0::0x05E6::0x2450::04393544::INSTR" 
        elif addr == 1:
            self.address = "USB0::0x05E6::0x2450::04502593::INSTR"
        elif addr == 2:
            self.address = "TCPIP::192.168.0.4::inst0::INSTR"
        else:
            self.address = addr
            return
        
        try:
            self.rm = pyvisa.ResourceManager()
            self.smu = self.rm.open_resource(self.address)
            print(self.smu.query("*IDN?"))
            self.smu.write("reset()")
            self.smu.write("*LANG TSP")
            self.smu.read_termination = "\n"
        except Exception as e:
            print(f"Connection failed: {e}")

    def set_output(self, V, I, turn_on=False):
        """
        Sets the output voltage and current, and optionally turns the output on.

        Args:
            V (float): Voltage limit.
            I (float): Current level.
            turn_on (bool): Whether to turn the output on (default False).
        """
        self.smu.write("smu.source.func = smu.FUNC_DC_CURRENT")
        self.smu.write("smu.source.autorange = smu.ON")
        self.smu.write(f"smu.source.level = {I}")
        self.smu.write(f"smu.source.vlimit.level = {V}")
        self.smu.write("smu.measure.func = smu.FUNC_DC_VOLTAGE")
        if turn_on:
            self.smu.write("smu.source.output = smu.ON")
            time.sleep(0.1)
            self.smu.write("smu.measure.read()")        
        
    def get_buff(self):
        """
        Retrieves voltage and current data from the instrument's buffer.

        Returns:
            tuple: Current and voltage values stored in the buffer.
        """
        endindex = int(self.smu.query("defbuffer1.endindex"))
        V = np.array((self.smu.query\
            ("printbuffer(1,%d,defbuffer1.readings)" %endindex))\
                .split(",")).astype(float)

        I = np.array((self.smu.query\
            ("printbuffer(1,%d,defbuffer1.sourcevalues)" %endindex))\
                .split(",")).astype(float)
        return I, V

    def resistance(self, I, V, turn_off = True):
        """
        Measures resistance based on supplied current and voltage.

        Args:
            I (float): Current level.
            V (float): Compliance voltage limit.
            turn_off (bool): Whether to turn off the output after measurement (default True).

        Returns:
            str: Measured resistance value [Ohm].
        """
        self.smu.write("smu.measure.func = smu.FUNC_DC_VOLTAGE")
        self.smu.write("smu.measure.autorange = smu.ON")
        self.smu.write("smu.measure.unit = smu.UNIT_OHM")
        self.smu.write("smu.source.func = smu.FUNC_DC_CURRENT")
        self.smu.write("smu.source.autorange = smu.ON")
        self.smu.write(f"smu.source.level = {I}")
        self.smu.write(f"smu.source.vlimit.level = {V}")
        self.smu.write("smu.source.output = smu.ON")
        time.sleep(0.1)
        self.smu.write("smu.measure.read(defbuffer1)")
        self.smu.write("smu.measure.read(defbuffer1)")
        R = self.smu.query("print(defbuffer1.readings[1])")
        if turn_off:   
            self.smu.write("smu.source.output = smu.OFF")
        self.smu.write("defbuffer1.clear()")

        return R
    
    def set_current(self, I, V_lim = 0.1, turn_on = True):
        """
        Sets the current to a specified value with a voltage limit.

        Args:
            I (float): Current level.
            V_lim (float): Voltage limit (default 0.1).
            turn_on (bool): Whether to turn on the output (default True).
            
        """
        self.smu.write("smu.source.func = smu.FUNC_DC_CURRENT")
        self.smu.write(f"smu.source.level = {I}")
        self.smu.write(f"smu.source.vlimit.level = {V_lim}")
        self.smu.write("smu.source.autorange = smu.ON")
        self.smu.write("smu.measure.func = smu.FUNC_DC_VOLTAGE")
        self.smu.write("smu.measure.read(defbuffer1)")
        self.smu.write("smu.source.output = smu.ON")
    
    def off(self):
        """
        Turns off the instrument's output.
        """
        self.smu.write("smu.source.output = smu.OFF")
    
    def on(self, cont = True):
        """
        Turns on the instrument's output and optionally sets continuous measurement mode.

        Args:
            cont (bool): Whether to enable continuous measurement (default True).
        """
        self.smu.write("smu.source.output = smu.ON")
        if cont:
            self.smu.write('trigger.continuous = trigger.CONT_RESTART') # Set inturment to LOCAL and starts continuous measurement	
            
    def reset(self):
        """
        Resets the instrument to its default settings.
        """
        self.smu.write("reset()")
        time.sleep(1)
        
    def chirp(self, mode = 0):
        """
        Produces a chirp sound, indicating the instrument status.

        Args:
            mode (int): 0 for good status, 1 for bad status (default 0).
        """
        if mode == 0:
            self.smu.write("beeper.beep(0.1, 1000)")
            time.sleep(0.15)
            self.smu.write("beeper.beep(0.1, 1000)")
            time.sleep(0.15)
            self.smu.write("beeper.beep(0.1, 1500)")            
        elif mode == 1:
            self.smu.write("beeper.beep(0.25, 880)")
            time.sleep(0.25)
            self.smu.write("beeper.beep(0.25, 880)")
            time.sleep(0.25)
            self.smu.write("beeper.beep(0.25, 880)")
  
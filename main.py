#!/usr/bin/env python3

import time
import sys
import os
import threading
import spidev
import RPi.GPIO as GPIO
import smbus2
import serial
import pynmea2
import struct

# ==========================================
# CONFIGURATION
# ==========================================
# Radio
RADIO_FREQ_MHZ = 433.0
NODE_ADDRESS = 0x01
TX_INTERVAL = 0.2  # 5 Hz update rate

# I2C Buses
I2C_BUS_IMU = 0  # GPIO 0/1
I2C_BUS_MAG = 1  # GPIO 2/3

# Addresses
ADDR_ISM330 = 0x6A  # Common for ISM330DHCX (or 0x6B)
ADDR_MAG = 0x0D     # QMC5883L

# Pins
PIN_BUZZER = 18

# ==========================================
# RFM69HCW DRIVER (Embedded)
# ==========================================
# Register addresses
REG_FIFO = 0x00
REG_OPMODE = 0x01
REG_DATAMODUL = 0x02
REG_BITRATEMSB = 0x03
REG_BITRATELSB = 0x04
REG_FDEVMSB = 0x05
REG_FDEVLSB = 0x06
REG_FRFMSB = 0x07
REG_FRFMID = 0x08
REG_FRFLSB = 0x09
REG_VERSION = 0x10
REG_PALEVEL = 0x11
REG_OCP = 0x13
REG_RXBW = 0x19
REG_IRQFLAGS1 = 0x27
REG_IRQFLAGS2 = 0x28
REG_SYNCCONFIG = 0x2E
REG_SYNCVALUE1 = 0x2F
REG_SYNCVALUE2 = 0x30
REG_PACKETCONFIG1 = 0x37
REG_PAYLOADLENGTH = 0x38
REG_FIFOTHRESH = 0x3C
REG_PACKETCONFIG2 = 0x3D
REG_TESTPA1 = 0x5A
REG_TESTPA2 = 0x5C
REG_TESTDAGC = 0x6F

MODE_SLEEP = 0x00
MODE_STANDBY = 0x04
MODE_TX = 0x0C
MODE_RX = 0x10

class RFM69:
    def __init__(self, spi_bus=0, spi_device=0, reset_pin=25, freq_mhz=433.0):
        self.reset_pin = reset_pin
        self.freq_mhz = freq_mhz
        self.mode = MODE_SLEEP
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.reset_pin, GPIO.OUT)
        
        self.spi = spidev.SpiDev()
        self.spi.open(spi_bus, spi_device)
        self.spi.max_speed_hz = 4000000
        self.spi.mode = 0b00
        
        self.reset()
        self.init_radio()
        
    def reset(self):
        GPIO.output(self.reset_pin, GPIO.HIGH)
        time.sleep(0.1)
        GPIO.output(self.reset_pin, GPIO.LOW)
        time.sleep(0.1)
        
    def write_reg(self, addr, value):
        self.spi.xfer2([addr | 0x80, value])

    def read_reg(self, addr):
        return self.spi.xfer2([addr & 0x7F, 0x00])[1]

    def init_radio(self):
        self.write_reg(REG_OPMODE, MODE_STANDBY)
        time.sleep(0.01)
        
        # Config: Packet mode, FSK
        config = [
            (REG_DATAMODUL, 0x00),
            (REG_BITRATEMSB, 0x06), (REG_BITRATELSB, 0x83), # 19.2 kbps
            (REG_FDEVMSB, 0x01), (REG_FDEVLSB, 0x9A),       # 25kHz dev
            (REG_RXBW, 0x42),                                 # RxBw 83.3kHz
            (REG_SYNCCONFIG, 0x88), (REG_SYNCVALUE1, 0x2D), (REG_SYNCVALUE2, 0xD4),
            (REG_PACKETCONFIG1, 0x90), # Variable len, CRC on
            (REG_PAYLOADLENGTH, 66),
            (REG_FIFOTHRESH, 0x8F),
            (REG_PACKETCONFIG2, 0x02),
            (REG_TESTDAGC, 0x30)
        ]
        for reg, val in config:
            self.write_reg(reg, val)
            
        # Set Frequency
        frf = int((self.freq_mhz * 1000000) / 61.03515625)
        self.write_reg(REG_FRFMSB, (frf >> 16) & 0xFF)
        self.write_reg(REG_FRFMID, (frf >> 8) & 0xFF)
        self.write_reg(REG_FRFLSB, frf & 0xFF)
        
        # High Power Settings (+20dBm)
        self.write_reg(REG_OCP, 0x0F)
        self.write_reg(REG_PALEVEL, 0x60 | 31) # Max power
        self.write_reg(REG_TESTPA1, 0x5D)
        self.write_reg(REG_TESTPA2, 0x7C)
        
    def send(self, data):
        if isinstance(data, str): data = data.encode('utf-8')
        
        self.write_reg(REG_OPMODE, MODE_STANDBY)
        timeout = time.time() + 0.5
        while not (self.read_reg(REG_IRQFLAGS1) & 0x80):
            if time.time() > timeout: return False
            time.sleep(0.001)
        
        # Write FIFO
        self.spi.xfer2([REG_FIFO | 0x80, len(data)] + list(data))
        
        # TX
        self.write_reg(REG_OPMODE, MODE_TX)
        
        # Wait for PacketSent
        start = time.time()
        while not (self.read_reg(REG_IRQFLAGS2) & 0x08):
            if time.time() - start > 1.0: return False
            time.sleep(0.001)
        
        self.write_reg(REG_OPMODE, MODE_STANDBY)
        return True
        
    def close(self):
        self.write_reg(REG_OPMODE, MODE_SLEEP)
        self.spi.close()

# ==========================================
# BUZZER DRIVER
# ==========================================
class Buzzer:
    def __init__(self, pin):
        self.pin = pin
        self._lock = threading.Lock()
        GPIO.setup(pin, GPIO.OUT)
        self.pwm = GPIO.PWM(pin, 1000) # 1kHz
        self.pwm.start(0)
        
    def _beep_sync(self, freq=2000, duration=0.1):
        """Blocking beep - call from a thread."""
        with self._lock:
            try:
                self.pwm.ChangeFrequency(freq)
                self.pwm.ChangeDutyCycle(50)
                time.sleep(duration)
                self.pwm.ChangeDutyCycle(0)
            except Exception:
                pass

    def beep(self, freq=2000, duration=0.1):
        """Blocking beep for startup/error (used before main loop)."""
        self._beep_sync(freq, duration)

    def beep_async(self, freq=2000, duration=0.1):
        """Non-blocking beep for use in main loop."""
        threading.Thread(target=self._beep_sync, args=(freq, duration), daemon=True).start()
        
    def startup_sequence(self):
        self.beep(1000, 0.1)
        time.sleep(0.05)
        self.beep(1500, 0.1)
        time.sleep(0.05)
        self.beep(2000, 0.2)

    def error_tone(self):
        self.beep(500, 0.5)

    def lock_tone(self):
        def _seq():
            self._beep_sync(2000, 0.05)
            time.sleep(0.05)
            self._beep_sync(2000, 0.05)
        threading.Thread(target=_seq, daemon=True).start()

    def heartbeat_tone(self):
        def _seq():
            self._beep_sync(800, 0.05)
            time.sleep(0.05)
            self._beep_sync(800, 0.05)
        threading.Thread(target=_seq, daemon=True).start()

    def cleanup(self):
        """Safe PWM cleanup."""
        try:
            self.pwm.ChangeDutyCycle(0)
            self.pwm.stop()
        except Exception:
            pass

# ==========================================
# SENSOR DRIVERS
# ==========================================
# ==========================================
# LOGGING & UI
# ==========================================
from collections import deque
import traceback

class Dashboard:
    def __init__(self):
        self.logs = deque(maxlen=10)
        self.errors = deque(maxlen=5)
    
    def log(self, msg):
        timestamp = time.strftime("%H:%M:%S")
        self.logs.append(f"[{timestamp}] {msg}")
        
    def error(self, msg):
        timestamp = time.strftime("%H:%M:%S")
        self.errors.append(f"[{timestamp}] {msg}")

    def render(self, sensor_data, radio_status, video_status="N/A"):
        # ANSI Escape Codes: Clear Screen, Home Cursor
        # \033[2J clears screen, \033[H moves to top-left
        output = "\033[2J\033[H"
        
        output += "==================================================\n"
        output += "           ERIS AVIONICS DASHBOARD               \n"
        output += "==================================================\n\n"
        
        # SENSOR DATA SECTION
        d = sensor_data
        output += f"SYSTEM TIME : {time.strftime('%H:%M:%S')}\n"
        gps_t = d.get('time', 0)
        if isinstance(gps_t, int) and gps_t > 0:
            gps_time_str = f"{gps_t // 3600:02d}:{(gps_t % 3600) // 60:02d}:{gps_t % 60:02d}"
        else:
            gps_time_str = "No Fix"
        output += f"GPS TIME    : {gps_time_str}\n"
        output += f"SATELLITES  : {d.get('sats', 0)}\n"
        output += f"LATITUDE    : {d.get('lat', 0.0):.6f}\n"
        output += f"LONGITUDE   : {d.get('lon', 0.0):.6f}\n"
        output += f"ALTITUDE    : {d.get('alt', 0.0):.1f} m\n"
        output += "\n"
        output += f"ACCEL (mg)  : X={d.get('ax',0):>6} Y={d.get('ay',0):>6} Z={d.get('az',0):>6}\n"
        output += f"GYRO (dps)  : X={d.get('gx',0):>6} Y={d.get('gy',0):>6} Z={d.get('gz',0):>6}\n"
        output += f"MAG (uT)    : X={d.get('mx',0):>6} Y={d.get('my',0):>6} Z={d.get('mz',0):>6}\n"
        output += "\n"
        output += f"RADIO STATE : {radio_status}\n"
        output += f"VIDEO STATE : {video_status}\n"
        output += f"RAW GPS     : {d.get('raw_gps', '')[:50]}\n" # Show first 50 chars of raw GPS
        
        output += "\n" + "="*50 + "\n"
        output += "LOGS\n"
        output += "="*50 + "\n"
        
        for log in self.logs:
            output += f" > {log}\n"
            
        if self.errors:
            output += "\n" + "!"*50 + "\n"
            output += "ERRORS / TRACEBACKS\n"
            output += "!"*50 + "\n"
            for err in self.errors:
                output += f"{err}\n"
        
        # DEBUG MENU
        output += "\n" + "-"*50 + "\n"
        output += "DEBUG CONTROLS (Type & Enter):\n"
        output += " 'arm'   -> Force ARM\n"
        output += " 'launch'-> Force ASCENT\n"
        output += " 'land'  -> Force LANDED\n"
        output += " 'video' -> Toggle Video\n"
        output += " 'reset' -> Reset State\n"
                
        print(output)

dashboard = Dashboard()

# ==========================================
# VIDEO RECORDER
# ==========================================
import subprocess
import shutil

class VideoRecorder:
    def __init__(self):
        self.process = None
        self.filename = None
        self.cmd_tool = None
        
        # Check for camera tool in order of preference:
        # 1. rpicam-vid   (Pi OS Bookworm / Debian 12+)
        # 2. libcamera-vid (Pi OS Bullseye / Debian 11)
        # 3. raspivid      (Legacy camera stack)
        for tool in ["rpicam-vid", "libcamera-vid", "raspivid"]:
            if shutil.which(tool):
                self.cmd_tool = tool
                dashboard.log(f"[OK] Video tool: {tool}")
                break
        
        if self.cmd_tool is None:
            dashboard.error("ERR: No video tool found (tried rpicam-vid, libcamera-vid, raspivid)")

    def start_recording(self):
        if self.process is not None:
            return # Already recording
        
        if self.cmd_tool is None:
            dashboard.error("Video Fail: No camera tool installed")
            return False
            
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        self.filename = f"flight_{timestamp}.h264"
        
        cmd = []
        if self.cmd_tool in ("rpicam-vid", "libcamera-vid"):
            cmd = [
                self.cmd_tool,
                "-t", "0",
                "--inline",
                "--width", "1920",
                "--height", "1080",
                "-o", self.filename,
                "--nopreview"
            ]
        elif self.cmd_tool == "raspivid":
            cmd = [
                "raspivid",
                "-t", "0",
                "-w", "1920",
                "-h", "1080",
                "-o", self.filename
            ]
        
        try:
            # shell=False is safer, uses list of args
            self.process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            dashboard.log(f"Video START: {self.filename}")
            return True
        except Exception as e:
            dashboard.error(f"Video Fail: {e}")
            return False

    def stop_recording(self):
        if self.process is None:
            return
            
        try:
            self.process.terminate()
            self.process.wait(timeout=2)
            dashboard.log("Video STOPPED")
        except:
            try:
                self.process.kill()
            except: pass
        finally:
            self.process = None

# ==========================================
# SENSOR DRIVERS
# ==========================================
# ==========================================
# LOGGING & STATE MANAGEMENT
# ==========================================
import csv
import math

# ==========================================
# FLIGHT COMPUTER
# ==========================================
class FlightState:
    IDLE = 0     # Pre-flight / No GPS
    ARMED = 1    # Ready on Pad
    ASCENT = 2   # Powered/Coast
    DESCENT = 3  # Apogee Reached
    LANDED = 4   # Stationary on ground

    @staticmethod
    def to_str(state):
        return ["IDLE", "ARMED", "ASCENT", "DESCENT", "LANDED"][state]

class CSVLogger:
    def __init__(self, filename="flight_log.csv"):
        self.filename = filename
        self.start_time = time.time()
        with open(self.filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "SystemTime", "RelTime", "State", 
                "GPSTime", "Sats", "Lat", "Lon", "Alt", 
                "Ax", "Ay", "Az", "Gx", "Gy", "Gz", "Mx", "My", "Mz"
            ])
            
    def log(self, state, data):
        try:
            with open(self.filename, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    time.strftime("%H:%M:%S"),
                    f"{time.time() - self.start_time:.2f}",
                    FlightState.to_str(state),
                    data.get('time', ''),
                    data.get('sats', 0),
                    f"{data.get('lat', 0.0):.6f}",
                    f"{data.get('lon', 0.0):.6f}",
                    f"{data.get('alt', 0.0):.2f}",
                    data.get('ax', 0), data.get('ay', 0), data.get('az', 0),
                    data.get('gx', 0), data.get('gy', 0), data.get('gz', 0),
                    data.get('mx', 0), data.get('my', 0), data.get('mz', 0),
                ])
        except Exception:
            pass

class FlightComputer:
    def __init__(self, video_recorder=None):
        self.state = FlightState.IDLE
        self.max_altitude = -9999.0
        self.ground_altitude = 0.0
        self.video = video_recorder
        
        # Detection Counters
        self.launch_samples = 0
        self.apogee_samples = 0
        self.land_samples = 0
        
        # Config Thresholds (Based on Z=2060 @ 1G)
        self.ACC_1G = 2060
        self.LAUNCH_THRESH_RAW = 4000  # ~2G
        self.LAUNCH_CONFIRM = 3        # Samples > Thresh to confirm launch
        
        self.APOGEE_DROP_M = 10.0      # Meters below max alt to confirm descent
        self.LAND_STABLE_TIME = 5.0    # Seconds of stable stats to confirm land
        self.last_alt = 0.0
        self.stable_start_time = 0
        
    def update(self, data):
        current_time = time.time()
        
        # Raw Data
        az = data.get('az', 0)
        alt = data.get('alt', 0.0)
        sats = data.get('sats', 0)
        
        # Track Max Altitude
        if alt > self.max_altitude:
            self.max_altitude = alt

        # ==========================
        # STATE MACHINE
        # ==========================
        
        # 1. IDLE -> ARMED
        if self.state == FlightState.IDLE:
            if sats >= 4:
                self.ground_altitude = alt
                self.max_altitude = alt # Reset max to current
                dashboard.log(f"ARMED. Ground Alt: {self.ground_altitude:.1f}m")
                self.state = FlightState.ARMED
                if self.video: self.video.start_recording()
        
        # 2. ARMED -> ASCENT
        elif self.state == FlightState.ARMED:
            # Detect Launch: Z-Accel > 2G approx
            if abs(az) > self.LAUNCH_THRESH_RAW:
                self.launch_samples += 1
            else:
                self.launch_samples = 0
                
            if self.launch_samples >= self.LAUNCH_CONFIRM:
                dashboard.log("!!! LAUNCH DETECTED !!!")
                self.state = FlightState.ASCENT
                self.launch_samples = 0
        
        # 3. ASCENT -> DESCENT (Apogee Detect)
        elif self.state == FlightState.ASCENT:
            # Detect Apogee: Current Alt < Max Alt - Threshold
            # (Simple drop detection)
            if alt < (self.max_altitude - self.APOGEE_DROP_M):
                self.apogee_samples += 1
            else:
                self.apogee_samples = 0
                
            if self.apogee_samples >= 3: # 3 consecutive samples showing drop
                dashboard.log(f"APOGEE DETECTED: {self.max_altitude:.1f}m")
                self.state = FlightState.DESCENT
        
        # 4. DESCENT -> LANDED
        elif self.state == FlightState.DESCENT:
            # Detect Landing: Altitude stable for N seconds
            # Check if alt changed less than 2m since last check
            if abs(alt - self.last_alt) < 2.0 and abs(az) < (self.ACC_1G + 500):
                if self.stable_start_time == 0:
                    self.stable_start_time = current_time
                elif (current_time - self.stable_start_time) > self.LAND_STABLE_TIME:
                    dashboard.log("LANDING CONFIRMED")
                    self.state = FlightState.LANDED
                    if self.video: self.video.stop_recording()
            else:
                self.stable_start_time = 0
                
            self.last_alt = alt

    def force_state(self, new_state):
        dashboard.log(f"DEBUG: Force State -> {FlightState.to_str(new_state)}")
        self.state = new_state
        
        # Handle side effects of forcing
        if new_state == FlightState.ARMED:
            if self.video: self.video.start_recording()
        elif new_state == FlightState.LANDED:
            if self.video: self.video.stop_recording()
        elif new_state == FlightState.IDLE:
            if self.video: self.video.stop_recording()



# ==========================================
# SENSOR DRIVERS
# ==========================================
class Sensors:
    def __init__(self):
        self.data = {
            "time": 0,  # Seconds since midnight (int) - ground station expects int
            "lat": 0.0, "lon": 0.0, "alt": 0.0, "sats": 0,
            "ax": 0, "ay": 0, "az": 0,
            "gx": 0, "gy": 0, "gz": 0,
            "mx": 0, "my": 0, "mz": 0,
            "raw_gps": "[Waiting...]"
        }
        self.lock = threading.Lock()
        
    def init_imu(self):
        try:
            self.bus_imu = smbus2.SMBus(I2C_BUS_IMU)
            # Init Accel (CTRL1_XL) - 104Hz, 16g range
            # 16g range: Sensitivity is ~0.488 mg/LSB
            self.bus_imu.write_byte_data(ADDR_ISM330, 0x10, 0x08 | 0x40) # 104Hz, 4g (adjust if needed, 16g is 0x08|0x40?? Check datasheet)
            # Correction: 
            # 0x40 = ODR 104Hz. 
            # FS Selection: see datasheet. 
            # Let's stick to previous code 0x44 (104Hz, 16g) if that worked.
            self.bus_imu.write_byte_data(ADDR_ISM330, 0x10, 0x44) 
            
            # Init Gyro
            self.bus_imu.write_byte_data(ADDR_ISM330, 0x11, 0x4C)
            self.bus_imu.write_byte_data(ADDR_ISM330, 0x12, 0x04)
            dashboard.log("[OK] IMU Init")
            return True
        except Exception as e:
            dashboard.error(f"IMU Init: {e}")
            return False

    def init_mag(self):
        try:
            self.bus_mag = smbus2.SMBus(I2C_BUS_MAG)
            self.bus_mag.write_byte_data(ADDR_MAG, 0x09, 0x1D)
            self.bus_mag.write_byte_data(ADDR_MAG, 0x0B, 0x01)
            dashboard.log("[OK] Mag Init")
            return True
        except Exception as e:
            dashboard.error(f"Mag Init: {e}")
            return False

    def read_imu(self):
        try:
            block = self.bus_imu.read_i2c_block_data(ADDR_ISM330, 0x22, 12)
            def parse(idx):
                val = block[idx] | (block[idx+1] << 8)
                return val - 65536 if val > 32767 else val
            with self.lock:
                self.data['gx'] = parse(0)
                self.data['gy'] = parse(2)
                self.data['gz'] = parse(4)
                self.data['ax'] = parse(6)
                self.data['ay'] = parse(8)
                self.data['az'] = parse(10)
        except: pass

    def read_mag(self):
        try:
            block = self.bus_mag.read_i2c_block_data(ADDR_MAG, 0x00, 6)
            def parse(idx):
                val = block[idx] | (block[idx+1] << 8)
                return val - 65536 if val > 32767 else val
            with self.lock:
                self.data['mx'] = parse(0)
                self.data['my'] = parse(2)
                self.data['mz'] = parse(4)
        except: pass

    def run_gps(self):
        try:
            ser = serial.Serial('/dev/serial0', 115200, timeout=1)
            dashboard.log("GPS Opened @ 115200")
            while True:
                try:
                    raw = ser.readline()
                    if not raw: continue
                    line = raw.decode('ascii', errors='ignore').strip()
                    with self.lock: self.data['raw_gps'] = line
                    if line.startswith('$G'):
                        msg = pynmea2.parse(line)
                        if isinstance(msg, pynmea2.types.talker.GGA):
                            with self.lock:
                                # Convert timestamp to seconds-since-midnight int
                                # Ground station expects int for T: field
                                ts = msg.timestamp
                                if ts is not None:
                                    self.data['time'] = int(ts.hour * 3600 + ts.minute * 60 + ts.second)
                                else:
                                    self.data['time'] = 0
                                try:
                                    self.data['sats'] = int(msg.num_sats) if msg.num_sats else 0
                                except (ValueError, TypeError):
                                    self.data['sats'] = 0
                                if msg.gps_qual and msg.gps_qual > 0:
                                    self.data['lat'] = msg.latitude
                                    self.data['lon'] = msg.longitude
                                    self.data['alt'] = msg.altitude if msg.altitude else 0.0
                except Exception:
                    pass
        except Exception as e:
            dashboard.error(f"GPS Thread: {e}")

# ==========================================
# MAIN LOOP
# ==========================================
def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    buzzer = Buzzer(PIN_BUZZER)
    buzzer.startup_sequence()
    
    video = VideoRecorder()
    
    sensors = Sensors()
    sensors.init_imu()
    sensors.init_mag()
    
    csv_logger = CSVLogger()
    flight_computer = FlightComputer(video_recorder=video)
    
    # Start GPS
    threading.Thread(target=sensors.run_gps, daemon=True).start()
    
    # Init Radio
    radio = None
    try:
        radio = RFM69(freq_mhz=RADIO_FREQ_MHZ)
        dashboard.log("Radio Init OK")
    except Exception as e:
        dashboard.error(f"Radio Fail: {e}")
        buzzer.error_tone()

    count = 0
    radio_status = "IDLE"
    
    # Non-blocking input setup
    import select
    
    print("\033[2J") # Clear once
    
    try:
        while True:
            start_t = time.time()
            
            # 1. Read Data
            sensors.read_imu()
            sensors.read_mag()
            
            with sensors.lock:
                d = sensors.data.copy()

            # 2. Update Flight Computer
            flight_computer.update(d)

            # 3. Log to CSV
            csv_logger.log(flight_computer.state, d)
            
            # 4. Prepare Packet
            # Protocol: "State,Time,Sats,Lat,Lon,Alt,Az,MaxAlt"
            state_char = FlightState.to_str(flight_computer.state)[0] # I, A, L...
            packet = (f"St:{state_char},T:{d['time']},S:{d['sats']},"
                      f"L:{d['lat']:.4f},{d['lon']:.4f},A:{d['alt']:.1f},"
                      f"Z:{d['az']},Max:{flight_computer.max_altitude:.1f}")
            
            # 5. Send Radio
            if radio:
                if radio.send(packet):
                    radio_status = "TX OK"
                else:
                    radio_status = "TX FAIL"
            
            # 6. UI Updates & DEBUG INPUT
            if count % 10 == 0:
                vid_state = "REC" if (video.process is not None) else "STOP"
                dashboard.render(d, radio_status, vid_state)
                #print(f"STATE: {FlightState.to_str(flight_computer.state)}") # Duplicate with dashboard

            # Check for Debug Input
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                cmd = sys.stdin.readline().strip().lower()
                if cmd == 'arm': flight_computer.force_state(FlightState.ARMED)
                elif cmd == 'launch': flight_computer.force_state(FlightState.ASCENT)
                elif cmd == 'land': flight_computer.force_state(FlightState.LANDED)
                elif cmd == 'reset': flight_computer.force_state(FlightState.IDLE)
                elif cmd == 'video':
                    if video.process: video.stop_recording()
                    else: video.start_recording()


            # Heartbeat / Lock Tone (already non-blocking now)
            if d['sats'] > 3 and count % 50 == 0: buzzer.lock_tone()
            if count % 20 == 0: buzzer.heartbeat_tone()

            count += 1
            elapsed = time.time() - start_t
            if elapsed < TX_INTERVAL:
                time.sleep(TX_INTERVAL - elapsed)
                
    except KeyboardInterrupt:
        print("\nStopping...")
        if radio: radio.close()
        video.stop_recording()
        buzzer.cleanup()
        try:
            GPIO.cleanup()
        except Exception:
            pass

if __name__ == "__main__":
    main()

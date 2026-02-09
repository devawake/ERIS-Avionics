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
            (REG_BITRATEMSB, 0x1A), (REG_BITRATELSB, 0x0B), # 4.8 kbps
            (REG_FDEVMSB, 0x00), (REG_FDEVLSB, 0x52),       # 5kHz dev
            (REG_RXBW, 0x55),
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
        while not (self.read_reg(REG_IRQFLAGS1) & 0x80): pass
        
        # Write FIFO
        self.spi.xfer2([REG_FIFO | 0x80, len(data)] + list(data))
        
        # TX
        self.write_reg(REG_OPMODE, MODE_TX)
        
        # Wait for PacketSent
        start = time.time()
        while not (self.read_reg(REG_IRQFLAGS2) & 0x08):
            if time.time() - start > 1.0: return False
        
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
        GPIO.setup(pin, GPIO.OUT)
        self.pwm = GPIO.PWM(pin, 1000) # 1kHz
        self.pwm.start(0)
        
    def beep(self, freq=2000, duration=0.1):
        self.pwm.ChangeFrequency(freq)
        self.pwm.ChangeDutyCycle(50)
        time.sleep(duration)
        self.pwm.ChangeDutyCycle(0)
        
    def startup_sequence(self):
        self.beep(1000, 0.1)
        time.sleep(0.05)
        self.beep(1500, 0.1)
        time.sleep(0.05)
        self.beep(2000, 0.2)

    def error_tone(self):
        self.beep(500, 0.5)

    def lock_tone(self):
        self.beep(2000, 0.05)
        time.sleep(0.05)
        self.beep(2000, 0.05)

    def heartbeat_tone(self):
        # "Lub-dub" sound: Low pitch, short duration
        self.beep(800, 0.05)
        time.sleep(0.05)
        self.beep(800, 0.05)

# ==========================================
# SENSOR DRIVERS
# ==========================================
class Sensors:
    def __init__(self):
        self.data = {
            "time": "00:00:00",
            "lat": 0.0, "lon": 0.0, "alt": 0.0, "sats": 0,
            "ax": 0, "ay": 0, "az": 0,
            "gx": 0, "gy": 0, "gz": 0,
            "mx": 0, "my": 0, "mz": 0
        }
        self.lock = threading.Lock()
        
    def init_imu(self):
        try:
            self.bus_imu = smbus2.SMBus(I2C_BUS_IMU)
            # Check ID
            who_am_i = self.bus_imu.read_byte_data(ADDR_ISM330, 0x0F)
            if who_am_i != 0x6B:
                print(f"[WARN] Unknown IMU ID: 0x{who_am_i:02X}")
            
            # Init Accel (CTRL1_XL) - 104Hz, 16g
            self.bus_imu.write_byte_data(ADDR_ISM330, 0x10, 0x44)
            # Init Gyro (CTRL2_G) - 104Hz, 2000dps
            self.bus_imu.write_byte_data(ADDR_ISM330, 0x11, 0x4C)
            # CTRL3_C - Auto-increment
            self.bus_imu.write_byte_data(ADDR_ISM330, 0x12, 0x04)
            
            print("[OK] IMU Initialized (Bus 0)")
            return True
        except Exception as e:
            print(f"[ERR] IMU Init Failed: {e}")
            return False

    def init_mag(self):
        try:
            self.bus_mag = smbus2.SMBus(I2C_BUS_MAG)
            # QMC5883L Init
            self.bus_mag.write_byte_data(ADDR_MAG, 0x09, 0x1D) # OSR=512, RNG=8G, ODR=200Hz, CONT
            self.bus_mag.write_byte_data(ADDR_MAG, 0x0B, 0x01)
            print("[OK] Magnetometer Initialized (Bus 1)")
            return True
        except Exception as e:
            print(f"[ERR] Mag Init Failed: {e}")
            return False

    def read_imu(self):
        try:
            # Read 12 bytes: Gx, Gy, Gz, Ax, Ay, Az
            block = self.bus_imu.read_i2c_block_data(ADDR_ISM330, 0x22, 12)
            
            # Helper to parse signed 16-bit
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
        except Exception:
            pass

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
        except Exception:
            pass

    def run_gps(self):
        try:
            ser = serial.Serial('/dev/serial0', 9600, timeout=1)
            while True:
                line = ser.readline().decode('ascii', errors='ignore').strip()
                if line.startswith('$G'):
                    try:
                        msg = pynmea2.parse(line)
                        if isinstance(msg, pynmea2.types.talker.GGA):
                            with self.lock:
                                self.data['time'] = str(msg.timestamp)
                                self.data['sats'] = int(msg.num_sats)
                                if msg.gps_qual > 0:
                                    self.data['lat'] = msg.latitude
                                    self.data['lon'] = msg.longitude
                                    self.data['alt'] = msg.altitude
                    except: pass
        except Exception as e:
            print(f"[ERR] GPS Thread: {e}")

# ==========================================
# MAIN LOOP
# ==========================================
def main():
    print("--- INTEGRATED AVIONICS ---")
    
    # Global GPIO Setup
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    buzzer = Buzzer(PIN_BUZZER)
    buzzer.startup_sequence()
    
    sensors = Sensors()
    if not sensors.init_imu(): buzzer.error_tone()
    if not sensors.init_mag(): buzzer.error_tone()
    
    # Start GPS Thread
    gps_thread = threading.Thread(target=sensors.run_gps, daemon=True)
    gps_thread.start()
    
    # Init Radio
    try:
        radio = RFM69(freq_mhz=RADIO_FREQ_MHZ)
        print("[OK] Radio Initialized")
    except Exception as e:
        print(f"[ERR] Radio Failed: {e}")
        buzzer.error_tone()
        return

    print("Starting loop...")
    count = 0
    
    try:
        while True:
            start_t = time.time()
            
            # Read Sensors
            sensors.read_imu()
            sensors.read_mag()
            
            # Prepare packet
            with sensors.lock:
                d = sensors.data
                # Format: "T:12:01:00,Lat:0.00,Lon:0.00,Alt:0,Ax:100...."
                packet = (f"T:{d['time']},S:{d['sats']},"
                          f"L:{d['lat']:.4f},{d['lon']:.4f},A:{d['alt']:.1f},"
                          f"Imu:{d['ax']},{d['ay']},{d['az']}")
            
            print(f"[TX] {packet[:60]}...")
            
            # Send
            if radio.send(packet):
                print("   -> Sent OK")
            else:
                print("   -> Send Failed")
                
            # Status Logic
            if d['sats'] > 3 and count % 50 == 0: # Every ~10s
                # GPS Locked Pulse
                print("   [GPS LOCKED]")
                buzzer.lock_tone()
            
            # Heartbeat every ~2s (10 packets @ 5Hz)
            if count % 10 == 0:
                threading.Thread(target=buzzer.heartbeat_tone).start()

            count += 1
            elapsed = time.time() - start_t
            if elapsed < TX_INTERVAL:
                time.sleep(TX_INTERVAL - elapsed)
                
    except KeyboardInterrupt:
        print("\nStopping...")
        radio.close()
        GPIO.cleanup()

if __name__ == "__main__":
    main()

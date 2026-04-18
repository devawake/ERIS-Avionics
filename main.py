#!/usr/bin/env python3
"""
ERIS Avionics — Main Flight Computer
UKROC Competition Build

State Machine:
  IDLE   -> ARMED   : Both GPIO 16 AND GPIO 26 shorted together
                      GPIO 26 = OUTPUT HIGH (3.3 V source)
                      GPIO 16 = INPUT with pull-down
                      Shorting the two pins drives GPIO 16 HIGH -> ARMED
  ARMED  -> ASCENT  : az > 2G for 10 consecutive samples (2 s @ 5 Hz)
                      AND altitude >= ground_alt + 5 m (secondary check, GPS optional)
  ASCENT -> DESCENT : alt drops >= 10 m below max for 5 consecutive samples
  DESCENT-> LANDED  : altitude stable (< 2 m change) + low accel for 5 s
  LANDED   (video stops 10 s after landing confirmed)

ARM wiring:
  GPIO 26 (BCM) — configured as OUTPUT HIGH (acts as 3.3 V source).
  GPIO 16 (BCM) — configured as INPUT with internal pull-down.
  Bridge/short the two pins with a wire or removable link to arm.
  Remove the link to disarm (safe on pad).

Data logging:
  All sensor data is logged to CSV from the moment the system boots.
  No radio / ground-station link required.
"""

import time
import sys
import os
import signal
import threading
import RPi.GPIO as GPIO
import smbus2
import serial
import pynmea2
import subprocess
import shutil
import select
import csv
from collections import deque

# ==========================================
# CONFIGURATION
# ==========================================
# Loop rate
LOOP_INTERVAL = 0.2   # 5 Hz update rate

# I2C Buses
I2C_BUS_IMU = 0   # GPIO 0/1
I2C_BUS_MAG = 1   # GPIO 2/3

# Addresses
ADDR_ISM330 = 0x6A   # ISM330DHCX (or 0x6B)
ADDR_MAG    = 0x0D   # QMC5883L

# Pins
PIN_BUZZER     = 18
PIN_ARM_IN     = 16   # BCM — INPUT with pull-down. HIGH when shorted to PIN_ARM_OUT.
PIN_ARM_OUT    = 26   # BCM — OUTPUT HIGH (3.3 V source).  Short to PIN_ARM_IN to arm.

# Flight detection tuning
ACC_1G              = 2060    # Raw LSB value at 1G (calibrate if needed)
LAUNCH_THRESH_RAW   = 4120    # ~2G in raw LSBs
LAUNCH_CONFIRM      = 10      # Consecutive samples > thresh to confirm launch (2 s @ 5 Hz)
LAUNCH_ALT_GAIN_M   = 5.0     # Minimum altitude gain (m) above ground to confirm launch (secondary)
APOGEE_DROP_M       = 10.0    # Metres below max alt to start counting apogee samples
APOGEE_CONFIRM      = 5       # Consecutive samples confirming altitude drop
LAND_STABLE_TIME    = 5.0     # Seconds of stable readings to confirm landing
LAND_STOP_DELAY     = 10.0    # Seconds after LANDED before stopping video

# Video
RECORDINGS_DIR       = "/home/eris/ERIS-Avionics/recordings"
VIDEO_FPS            = 30        # Recording frame rate
VIDEO_MAX_DURATION_S = 600       # Auto stop recording 10 minutes after arming (SD card safety)

# RFM69 driver removed — no ground station required.


# ==========================================
# BUZZER DRIVER
# ==========================================
class Buzzer:
    def __init__(self, pin):
        self.pin   = pin
        self._lock = threading.Lock()
        GPIO.setup(pin, GPIO.OUT)
        self.pwm = GPIO.PWM(pin, 1000)
        self.pwm.start(0)

    def _beep_sync(self, freq=2000, duration=0.1):
        with self._lock:
            try:
                self.pwm.ChangeFrequency(freq)
                self.pwm.ChangeDutyCycle(50)
                time.sleep(duration)
                self.pwm.ChangeDutyCycle(0)
            except Exception:
                pass

    def beep(self, freq=2000, duration=0.1):
        self._beep_sync(freq, duration)

    def beep_async(self, freq=2000, duration=0.1):
        threading.Thread(target=self._beep_sync, args=(freq, duration), daemon=True).start()

    def startup_sequence(self):
        """Ascending tones — system armed / boot."""
        self.beep(1000, 0.1); time.sleep(0.05)
        self.beep(1500, 0.1); time.sleep(0.05)
        self.beep(2000, 0.2)

    def shutdown_sequence(self):
        """Descending tones (reverse of startup) — recording stopped."""
        self.beep(2000, 0.1); time.sleep(0.05)
        self.beep(1500, 0.1); time.sleep(0.05)
        self.beep(1000, 0.2)

    def armed_sequence(self):
        """Three quick high beeps — confirms ARM state."""
        def _seq():
            for _ in range(3):
                self._beep_sync(2500, 0.08)
                time.sleep(0.08)
        threading.Thread(target=_seq, daemon=True).start()

    def launch_tone(self):
        """Single long high tone — launch detected."""
        threading.Thread(target=self._beep_sync, args=(3000, 0.5), daemon=True).start()

    def landed_tone(self):
        """Descending tones — landed."""
        def _seq():
            for freq in [2000, 1500, 1000]:
                self._beep_sync(freq, 0.15)
                time.sleep(0.05)
        threading.Thread(target=_seq, daemon=True).start()

    def error_tone(self):
        self.beep(500, 0.5)

    def lock_tone(self):
        def _seq():
            self._beep_sync(2000, 0.05); time.sleep(0.05)
            self._beep_sync(2000, 0.05)
        threading.Thread(target=_seq, daemon=True).start()

    def heartbeat_tone(self):
        def _seq():
            self._beep_sync(800, 0.05); time.sleep(0.05)
            self._beep_sync(800, 0.05)
        threading.Thread(target=_seq, daemon=True).start()

    def cleanup(self):
        try:
            self.pwm.ChangeDutyCycle(0)
            self.pwm.stop()
        except Exception:
            pass


# ==========================================
# DASHBOARD / LOGGING
# ==========================================
class Dashboard:
    def __init__(self):
        self.logs   = deque(maxlen=12)
        self.errors = deque(maxlen=5)

    def log(self, msg):
        print(f"[LOG {time.strftime('%H:%M:%S')}] {msg}")
        self.logs.append(f"[{time.strftime('%H:%M:%S')}] {msg}")

    def error(self, msg):
        print(f"[ERR {time.strftime('%H:%M:%S')}] {msg}")
        self.errors.append(f"[{time.strftime('%H:%M:%S')}] {msg}")

    def render(self, sensor_data, video_status, state_str, arm_pins):
        output = "\033[2J\033[H"
        output += "==================================================\n"
        output += "           ERIS AVIONICS DASHBOARD               \n"
        output += "==================================================\n\n"

        d = sensor_data
        output += f"SYSTEM TIME : {time.strftime('%H:%M:%S')}\n"

        gps_t = d.get('time', 0)
        if isinstance(gps_t, int) and gps_t > 0:
            gps_str = f"{gps_t // 3600:02d}:{(gps_t % 3600) // 60:02d}:{gps_t % 60:02d}"
        else:
            gps_str = "No Fix"
        output += f"GPS TIME    : {gps_str}\n"
        output += f"SATELLITES  : {d.get('sats', 0)}\n"
        output += f"LATITUDE    : {d.get('lat', 0.0):.6f}\n"
        output += f"LONGITUDE   : {d.get('lon', 0.0):.6f}\n"
        output += f"ALTITUDE    : {d.get('alt', 0.0):.1f} m\n"
        output += "\n"
        output += f"ACCEL (raw) : X={d.get('ax',0):>7} Y={d.get('ay',0):>7} Z={d.get('az',0):>7}\n"
        output += f"GYRO  (raw) : X={d.get('gx',0):>7} Y={d.get('gy',0):>7} Z={d.get('gz',0):>7}\n"
        output += f"MAG   (raw) : X={d.get('mx',0):>7} Y={d.get('my',0):>7} Z={d.get('mz',0):>7}\n"
        output += "\n"

        arm_str  = "SHORTED (ARMED)" if arm_pins else "OPEN    (SAFE) "
        output += f"ARM PINS    : GPIO {PIN_ARM_IN} + GPIO {PIN_ARM_OUT} — {arm_str}\n"
        output += f"FLIGHT STATE: {state_str}\n"
        output += f"VIDEO       : {video_status}\n"
        output += f"RAW GPS     : {d.get('raw_gps', '')[:60]}\n"

        output += "\n" + "=" * 50 + "\n"
        output += "LOGS\n"
        output += "=" * 50 + "\n"
        for log in self.logs:
            output += f" > {log}\n"

        if self.errors:
            output += "\n" + "!" * 50 + "\n"
            output += "ERRORS\n" + "!" * 50 + "\n"
            for err in self.errors:
                output += f"  {err}\n"

        output += "\n" + "-" * 50 + "\n"
        output += "DEBUG CONTROLS (stdin):\n"
        output += "  'arm'    -> Force ARMED\n"
        output += "  'launch' -> Force ASCENT\n"
        output += "  'land'   -> Force LANDED\n"
        output += "  'reset'  -> Force IDLE\n"
        output += "  'video'  -> Toggle Video\n"

        print(output)

dashboard = Dashboard()


# ==========================================
# VIDEO RECORDER
# ==========================================
class VideoRecorder:
    def __init__(self):
        self.process  = None
        self.filename = None
        self.cmd_tool = None

        os.makedirs(RECORDINGS_DIR, exist_ok=True)

        for tool in ["rpicam-vid", "libcamera-vid", "raspivid"]:
            if shutil.which(tool):
                self.cmd_tool = tool
                dashboard.log(f"[OK] Video tool: {tool}")
                break

        if self.cmd_tool is None:
            dashboard.error("ERR: No video tool found (rpicam-vid / libcamera-vid / raspivid)")

    def start_recording(self):
        if self.process is not None:
            return True  # Already recording
        if self.cmd_tool is None:
            dashboard.error("Video: No camera tool installed")
            return False

        timestamp     = time.strftime("%Y%m%d-%H%M%S")
        self.filename = os.path.join(RECORDINGS_DIR, f"flight_{timestamp}.h264")

        if self.cmd_tool in ("rpicam-vid", "libcamera-vid"):
            cmd = [
                self.cmd_tool,
                "-t", "0",           # Run indefinitely (we stop it ourselves)
                "--inline",
                "--width",     "1920",
                "--height",    "1080",
                "--framerate", str(VIDEO_FPS),
                "-o", self.filename,
                "--nopreview",
            ]
        else:  # raspivid
            cmd = [
                "raspivid",
                "-t", "0",
                "-w", "1920",
                "-h", "1080",
                "-fps", str(VIDEO_FPS),
                "-o", self.filename,
            ]

        try:
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
            # Give camera 2 s to initialise
            time.sleep(2.0)
            ret = self.process.poll()
            if ret is not None:
                err_out   = self.process.stderr.read().decode('utf-8', errors='ignore').strip()
                self.process = None
                err_lines = [l for l in err_out.splitlines()
                             if 'ERROR' in l or 'error' in l or 'fail' in l.lower()]
                msg = err_lines[-1][-120:] if err_lines else f"exit code {ret}"
                dashboard.error(f"Video Fail: {msg}")
                return False

            dashboard.log(f"Video REC started -> {self.filename}")
            return True
        except Exception as e:
            dashboard.error(f"Video Fail: {e}")
            self.process = None
            return False

    def stop_recording(self):
        if self.process is None:
            return

        import signal
        try:
            self.process.send_signal(signal.SIGINT)
            self.process.wait(timeout=5)
            if self.filename and os.path.exists(self.filename):
                size_kb = os.path.getsize(self.filename) / 1024
                dashboard.log(f"Video SAVED: {size_kb:.0f} KB -> {self.filename}")
            else:
                dashboard.error("Video: File not found after stop!")
        except subprocess.TimeoutExpired:
            dashboard.error("Video: Graceful stop timed out, killing...")
            try:
                self.process.kill()
                self.process.wait(timeout=2)
            except Exception:
                pass
        except Exception as e:
            dashboard.error(f"Video stop error: {e}")
            try:
                self.process.kill()
            except Exception:
                pass
        finally:
            self.process = None


# ==========================================
# FLIGHT STATE
# ==========================================
class FlightState:
    IDLE    = 0
    ARMED   = 1
    ASCENT  = 2
    DESCENT = 3
    LANDED  = 4

    _NAMES = ["IDLE", "ARMED", "ASCENT", "DESCENT", "LANDED"]

    @staticmethod
    def to_str(state):
        return FlightState._NAMES[state]


# ==========================================
# CSV LOGGER
# ==========================================
class CSVLogger:
    def __init__(self, filename="/home/eris/ERIS-Avionics/flight_log.csv"):
        self.filename   = filename
        self.start_time = time.time()
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        with open(self.filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "SystemTime", "RelTime", "State",
                "GPSTime", "Sats", "Lat", "Lon", "Alt",
                "Ax", "Ay", "Az", "Gx", "Gy", "Gz", "Mx", "My", "Mz",
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


# ==========================================
# FLIGHT COMPUTER
# ==========================================
class FlightComputer:
    """
    State machine for flight phase detection.

    Fully GPS-optional — the system degrades gracefully if GPS is unavailable:

    Arming:
      - Requires only the physical screw switch (GPIO 16) to be HIGH.
      - GPS satellites are logged as info but NOT required.
      - Ground altitude is captured if GPS has a fix; otherwise alt-based
        checks are skipped and only IMU data is used.

    Ascent Detection:
      - Primary (always):  raw az > LAUNCH_THRESH_RAW for LAUNCH_CONFIRM
                           consecutive samples (2 s @ 5 Hz).
      - Secondary (GPS):   altitude has risen >= LAUNCH_ALT_GAIN_M above
                           ground_altitude. Skipped when GPS unavailable.

    Apogee / Descent:
      - With GPS:    altitude drops >= APOGEE_DROP_M below max for
                     APOGEE_CONFIRM consecutive samples.
      - Without GPS: az stays below (ACC_1G + 300) — near-freefall /
                     coast / descent — for APOGEE_CONFIRM consecutive
                     samples after launch.

    Landing:
      - With GPS:    altitude stable < 2 m change AND low acceleration
                     for LAND_STABLE_TIME seconds.
      - Without GPS: low acceleration near 1G for LAND_STABLE_TIME seconds.

    Video:
      - Starts when entering ARMED.
      - Stops LAND_STOP_DELAY seconds after LANDED is confirmed.
    """

    def __init__(self, video_recorder=None, buzzer=None):
        self.state          = FlightState.IDLE
        self.video          = video_recorder
        self.buzzer         = buzzer

        self.ground_altitude = 0.0
        self.max_altitude    = 0.0
        self.last_alt        = 0.0

        # Detection counters
        self.launch_samples  = 0   # Consecutive samples above launch G threshold
        self.apogee_samples  = 0   # Consecutive samples showing altitude drop
        self.stable_start    = 0.0 # Timestamp when stable descent-to-land began
        self.land_time       = 0.0 # Timestamp when LANDED was confirmed
        self.arm_time        = 0.0 # Timestamp when armed (for video duration cutoff)

        # Pin toggle state — counts rising edges on GPIO 16.
        # Edge 1 = ARM + start recording. Edge 2 = stop recording.
        self._arm_count  = 0      # How many times the pin pair has been shorted
        self._pin_prev   = False  # Previous GPIO 16 level (for edge detection)

        # Setup arm GPIO pins
        # GPIO 26 = OUTPUT HIGH (acts as 3.3 V source)
        # GPIO 16 = INPUT with pull-down; reads HIGH when shorted to GPIO 26
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(PIN_ARM_OUT, GPIO.OUT)
        GPIO.output(PIN_ARM_OUT, GPIO.HIGH)
        GPIO.setup(PIN_ARM_IN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        dashboard.log(f"ARM pins configured — OUT: GPIO {PIN_ARM_OUT} (HIGH), IN: GPIO {PIN_ARM_IN} (pull-down)")
        dashboard.log("Tap GPIO 16 + GPIO 26 to ARM.  Tap again to stop & save recording.")

    def _rising_edge(self):
        """Returns True on a LOW→HIGH transition of GPIO 16 (one pulse per touch)."""
        current = GPIO.input(PIN_ARM_IN) == GPIO.HIGH
        edge = current and not self._pin_prev
        self._pin_prev = current
        return edge

    @property
    def arm_switch(self):
        """True once the pins have been touched at least once (for dashboard display)."""
        return self._arm_count >= 1

    def update(self, data):
        now  = time.time()
        az   = data.get('az',   0)
        alt  = data.get('alt',  0.0)
        sats = data.get('sats', 0)

        # GPS is considered available only when we have a real fix
        gps_ok = sats >= 4 and alt != 0.0

        # ── PIN TOGGLE DETECTION ──────────────────────────────────────────
        if self._rising_edge():
            self._arm_count += 1

            if self._arm_count == 1:
                # ── FIRST TAP: ARM + start recording ───────────────────────
                self.launch_samples  = 0
                self.apogee_samples  = 0
                self.stable_start    = 0.0
                if gps_ok:
                    self.ground_altitude = alt
                    self.max_altitude    = alt
                    self.last_alt        = alt
                    dashboard.log(f">>> ARMED (GPS OK) — Ground alt: {self.ground_altitude:.1f} m, Sats: {sats}")
                else:
                    self.ground_altitude = 0.0
                    self.max_altitude    = 0.0
                    self.last_alt        = 0.0
                    dashboard.log(f">>> ARMED (NO GPS) — IMU-only mode. Sats: {sats}")
                self.state    = FlightState.ARMED
                self.arm_time = now
                if self.buzzer: self.buzzer.startup_sequence()   # Ascending tone — armed
                if self.video:  self.video.start_recording()

            elif self._arm_count == 2:
                # ── SECOND TAP: stop recording ──────────────────────────
                dashboard.log(">>> PIN TAP 2 — stopping and saving recording.")
                if self.buzzer: self.buzzer.shutdown_sequence()  # Descending tone — done
                if self.video:  self.video.stop_recording()

        # ── VIDEO DURATION CUTOFF ───────────────────────────────────────
        # Fallback: auto-stop if recording has run for VIDEO_MAX_DURATION_S.
        if (self.arm_time > 0
                and self.video and self.video.process is not None
                and (now - self.arm_time) >= VIDEO_MAX_DURATION_S):
            mins = VIDEO_MAX_DURATION_S // 60
            dashboard.log(f"Video auto-cutoff: {mins:.0f} min limit reached — stopping recording.")
            if self.buzzer: self.buzzer.shutdown_sequence()
            self.video.stop_recording()

        # ── IDLE ──────────────────────────────────────────────────────────────
        # Arm transition is now handled by pin-toggle detection above.
        if self.state == FlightState.IDLE:
            pass  # Waiting for first pin tap (handled above)
            # Primary: sustained high-G for LAUNCH_CONFIRM consecutive samples
            if abs(az) > LAUNCH_THRESH_RAW:
                self.launch_samples += 1
            else:
                self.launch_samples = 0   # Reset on any sample below threshold

            # Secondary: if GPS available, altitude must have gained >= LAUNCH_ALT_GAIN_M.
            # Without GPS, IMU confirmation alone is sufficient.
            if gps_ok:
                alt_confirmed = (alt - self.ground_altitude) >= LAUNCH_ALT_GAIN_M
            else:
                alt_confirmed = True  # GPS not available — trust IMU only

            if self.launch_samples >= LAUNCH_CONFIRM and alt_confirmed:
                if gps_ok:
                    dashboard.log(f">>> LAUNCH DETECTED (GPS+IMU) — az={az}, alt gain={(alt - self.ground_altitude):.1f} m")
                else:
                    dashboard.log(f">>> LAUNCH DETECTED (IMU only) — az={az}, {self.launch_samples} samples > 2G")
                self.state          = FlightState.ASCENT
                self.max_altitude   = alt if gps_ok else 0.0
                self.launch_samples = 0
                if self.buzzer: self.buzzer.launch_tone()

            # No disarm path — arm trigger is a one-shot latch.

        # ── ASCENT ────────────────────────────────────────────────────────────
        elif self.state == FlightState.ASCENT:
            if gps_ok:
                # GPS available: classic altitude-drop apogee detection
                if alt < (self.max_altitude - APOGEE_DROP_M):
                    self.apogee_samples += 1
                else:
                    self.apogee_samples = 0
            else:
                # No GPS: detect apogee/coast by sustained near-1G or lower.
                # After motor burnout the rocket is in freefall/coast — az drops
                # to near or below 1G. Count consecutive samples meeting this.
                if abs(az) < (ACC_1G + 300):
                    self.apogee_samples += 1
                else:
                    self.apogee_samples = 0

            if self.apogee_samples >= APOGEE_CONFIRM:
                if gps_ok:
                    dashboard.log(f">>> APOGEE (GPS) — Max alt: {self.max_altitude:.1f} m")
                else:
                    dashboard.log(f">>> APOGEE (IMU) — low-G detected for {APOGEE_CONFIRM} samples")
                self.state          = FlightState.DESCENT
                self.apogee_samples = 0

        # ── DESCENT ───────────────────────────────────────────────────────────
        elif self.state == FlightState.DESCENT:
            # Acceleration near 1G = stationary on ground (always available)
            acc_stable = abs(az) < (ACC_1G + 500)

            if gps_ok:
                # GPS available: also require altitude to be stable
                alt_stable = abs(alt - self.last_alt) < 2.0
                is_stable  = alt_stable and acc_stable
            else:
                # No GPS: rely purely on accelerometer stability
                is_stable  = acc_stable

            if is_stable:
                if self.stable_start == 0.0:
                    self.stable_start = now
                elif (now - self.stable_start) >= LAND_STABLE_TIME:
                    if gps_ok:
                        dashboard.log(f">>> LANDED CONFIRMED (GPS+IMU) — alt: {alt:.1f} m")
                    else:
                        dashboard.log(">>> LANDED CONFIRMED (IMU only) — accel stable")
                    self.state     = FlightState.LANDED
                    self.land_time = now
                    if self.buzzer: self.buzzer.landed_tone()
            else:
                self.stable_start = 0.0   # Reset stability timer on movement

            self.last_alt = alt

        # ── LANDED ────────────────────────────────────────────────────────────
        elif self.state == FlightState.LANDED:
            # Stop video LAND_STOP_DELAY seconds after landing
            if self.video and self.video.process is not None:
                if (now - self.land_time) >= LAND_STOP_DELAY:
                    dashboard.log(f"Video stopping {LAND_STOP_DELAY:.0f} s after landing")
                    self.video.stop_recording()

    def force_state(self, new_state):
        """Debug override — bypass normal transition logic."""
        dashboard.log(f"[DEBUG] Force state: {FlightState.to_str(self.state)} -> {FlightState.to_str(new_state)}")
        self.state = new_state

        if new_state == FlightState.ARMED:
            if self.video: self.video.start_recording()
        elif new_state == FlightState.ASCENT:
            self.max_altitude = self.ground_altitude  # Will be updated via update()
        elif new_state in (FlightState.LANDED, FlightState.IDLE):
            self.land_time = time.time()
            if new_state == FlightState.IDLE and self.video:
                self.video.stop_recording()


# ==========================================
# SENSOR DRIVERS
# ==========================================
class Sensors:
    def __init__(self):
        self.data = {
            "time": 0,
            "lat": 0.0, "lon": 0.0, "alt": 0.0, "sats": 0,
            "ax": 0, "ay": 0, "az": 0,
            "gx": 0, "gy": 0, "gz": 0,
            "mx": 0, "my": 0, "mz": 0,
            "raw_gps": "[Waiting...]",
        }
        self.lock = threading.Lock()

    def init_imu(self):
        try:
            self.bus_imu = smbus2.SMBus(I2C_BUS_IMU)
            # CTRL1_XL: 104 Hz, 16 g range
            self.bus_imu.write_byte_data(ADDR_ISM330, 0x10, 0x44)
            # CTRL2_G:  104 Hz, 2000 dps
            self.bus_imu.write_byte_data(ADDR_ISM330, 0x11, 0x4C)
            # CTRL3_C:  BDU enable
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
                val = block[idx] | (block[idx + 1] << 8)
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
                val = block[idx] | (block[idx + 1] << 8)
                return val - 65536 if val > 32767 else val
            with self.lock:
                self.data['mx'] = parse(0)
                self.data['my'] = parse(2)
                self.data['mz'] = parse(4)
        except Exception:
            pass

    def run_gps(self):
        """Background thread — reads NMEA from serial and updates sensor data."""
        try:
            ser = serial.Serial('/dev/serial0', 115200, timeout=1)
            dashboard.log("GPS Opened @ 115200")
            while True:
                try:
                    raw = ser.readline()
                    if not raw:
                        continue
                    line = raw.decode('ascii', errors='ignore').strip()
                    with self.lock:
                        self.data['raw_gps'] = line

                    if line.startswith('$G'):
                        msg = pynmea2.parse(line)
                        if isinstance(msg, pynmea2.types.talker.GGA):
                            with self.lock:
                                ts = msg.timestamp
                                self.data['time'] = (
                                    int(ts.hour * 3600 + ts.minute * 60 + ts.second)
                                    if ts else 0
                                )
                                try:
                                    self.data['sats'] = int(msg.num_sats) if msg.num_sats else 0
                                except (ValueError, TypeError):
                                    self.data['sats'] = 0

                                if msg.gps_qual and msg.gps_qual > 0:
                                    self.data['lat'] = msg.latitude
                                    self.data['lon'] = msg.longitude
                                    self.data['alt'] = float(msg.altitude) if msg.altitude else 0.0
                except Exception:
                    pass
        except Exception as e:
            dashboard.error(f"GPS Thread: {e}")


# ==========================================
# MAIN
# ==========================================
def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Catch SIGTERM (sent by sudo shutdown / systemd stop) so the finally block
    # runs and video is saved cleanly before the process is killed.
    def _sigterm_handler(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _sigterm_handler)

    buzzer = Buzzer(PIN_BUZZER)
    buzzer.startup_sequence()

    video   = VideoRecorder()
    sensors = Sensors()
    sensors.init_imu()
    sensors.init_mag()

    # CSV logger starts immediately — all data logged from boot
    csv_logger      = CSVLogger()
    flight_computer = FlightComputer(video_recorder=video, buzzer=buzzer)

    # GPS background thread
    threading.Thread(target=sensors.run_gps, daemon=True).start()

    count = 0
    print("\033[2J")  # Initial clear

    dashboard.log("System ready — tap GPIO 16 + GPIO 26 to ARM and start recording.")
    dashboard.log(f"Recordings: {RECORDINGS_DIR}")
    dashboard.log("CSV logging active from boot.")

    try:
        while True:
            loop_start = time.time()

            # 1. Read sensors
            sensors.read_imu()
            sensors.read_mag()
            with sensors.lock:
                d = sensors.data.copy()

            # 2. Flight computer update (state machine)
            flight_computer.update(d)

            # 3. Log ALL data to CSV on every cycle (from boot, regardless of state)
            csv_logger.log(flight_computer.state, d)

            # 4. Render dashboard (every 10 cycles ≈ 2 s)
            state_str = FlightState.to_str(flight_computer.state)
            if count % 10 == 0:
                vid_str = "REC" if (video.process is not None) else "STOP"
                dashboard.render(
                    d,
                    vid_str,
                    state_str,
                    flight_computer.arm_switch,
                )

            # 5. Debug console input (non-blocking)
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                cmd = sys.stdin.readline().strip().lower()
                if   cmd == 'arm':    flight_computer.force_state(FlightState.ARMED)
                elif cmd == 'launch': flight_computer.force_state(FlightState.ASCENT)
                elif cmd == 'land':   flight_computer.force_state(FlightState.LANDED)
                elif cmd == 'reset':  flight_computer.force_state(FlightState.IDLE)
                elif cmd == 'video':
                    if video.process:
                        video.stop_recording()
                    else:
                        video.start_recording()

            # 6. Buzzer heartbeats
            if d['sats'] > 3 and count % 50 == 0:
                buzzer.lock_tone()
            if count % 20 == 0:
                buzzer.heartbeat_tone()

            count += 1
            elapsed = time.time() - loop_start
            sleep_t = LOOP_INTERVAL - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        video.stop_recording()
        buzzer.cleanup()
        try:
            GPIO.cleanup()
        except Exception:
            pass


if __name__ == "__main__":
    main()

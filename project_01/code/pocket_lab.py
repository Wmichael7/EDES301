# -*- coding: utf-8 -*-
"""
--------------------------------------------------------------------------
PocketLab Data Logger
--------------------------------------------------------------------------
License:
Copyright 2025 - Michael Wu

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------

PocketLab Data Logger for PocketBeagle

Features:
  - Reads DHT11 (temperature/humidity) on GPIO
  - Reads photocell (LDR) via ADC
  - Button click via GPIO for manual snapshots
  - Displays status on 16x2 I2C LCD
  - Beeps buzzer and toggles LED for heartbeat/faults
  - Writes timestamped CSV

Hardware:
  - PocketBeagle with DHT11, LDR, button, buzzer, LED, 16x2 I2C LCD

Requirements:
  pip3 install Adafruit_BBIO Adafruit_DHT smbus2

Enable I2C and ADC:
  config-pin P1_26 i2c   # I2C1 SDA
  config-pin P1_28 i2c   # I2C1 SCL
  config-pin P1_19 ain   # AIN0 (LDR)
"""

import csv
import os
import time
from collections import deque
from datetime import datetime

# Configuration
CFG = {
    "i2c_bus": 1,
    "lcd_addr": 0x27,
    "button": "P2_02",
    "dht_pin": "P2_04",
    "buzzer": "P2_06",
    "led": "P2_08",
    "ldr_ain": "P1_19",
    "dht_type": 11,
    "log_dir": "./logs",
    "sample_interval_s": 10,
}

# Hardware imports with fallback
try:
    import Adafruit_BBIO.ADC as ADC
    import Adafruit_BBIO.GPIO as GPIO
    import Adafruit_DHT

    HW = True
except ImportError:
    HW, ADC, GPIO, Adafruit_DHT = False, None, None, None

try:
    import smbus2
except ImportError:
    smbus2 = None


# 16x2 I2C LCD driver with console fallback
class LCD16x2:
    """16x2 I2C LCD driver with console fallback."""

    def __init__(self, bus=1, addr=0x27):
        self.addr, self.bus, self.stub = addr, None, True
        if smbus2:
            try:
                self.bus = smbus2.SMBus(bus)
                for cmd in [0x33, 0x32, 0x06, 0x0C, 0x28, 0x01]:
                    self._cmd(cmd)
                self.stub = False
            except Exception:
                pass

    def _cmd(self, data, mode=0):
        if not self.bus:
            return
        for nibble in [(data & 0xF0), ((data << 4) & 0xF0)]:
            b = nibble | mode | 0x08
            try:
                self.bus.write_byte(self.addr, b)
                self.bus.write_byte(self.addr, b | 0x04)
                self.bus.write_byte(self.addr, b & ~0x04)
            except Exception:
                pass

    def render(self, l1, l2=""):
        l1, l2 = (l1 or "")[:16], (l2 or "")[:16]
        if self.stub:
            print(f"[LCD] {l1.ljust(16)}\n[LCD] {l2.ljust(16)}")
        else:
            for line, addr in [(l1, 0x80), (l2, 0xC0)]:
                self._cmd(addr)
                for c in line.ljust(16):
                    self._cmd(ord(c), 1)

    def close(self):
        if self.bus:
            try:
                self.bus.close()
            except Exception:
                pass


# CSV logger
class Logger:
    """CSV logger."""

    HEADER = ["ts", "t_C", "h_%", "ldr"]

    def __init__(self, log_dir):
        self.log_dir = log_dir
        self.file = None
        self.writer = None
        os.makedirs(log_dir, exist_ok=True)
        self._open()

    def _open(self):
        path = os.path.join(self.log_dir, "pocketlab.csv")
        exists = os.path.exists(path)
        self.file = open(path, "a", newline="", buffering=1)
        self.writer = csv.writer(self.file)
        if not exists:
            self.writer.writerow(self.HEADER)
            self.file.flush()
            os.fsync(self.file.fileno())

    def append(self, row):
        if self.writer:
            self.writer.writerow(row)
            self.file.flush()
            os.fsync(self.file.fileno())

    def close(self):
        if self.file:
            try:
                self.file.close()
            except Exception:
                pass
            self.file = self.writer = None


# DHT11 and ADC sensor manager with smoothing
class SensorManager:
    """DHT11 and ADC sensor manager with smoothing."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.ldr_buf = deque(maxlen=4)
        if HW and ADC:
            try:
                ADC.setup()
            except Exception:
                pass

    def _adc(self, pin):
        if HW and ADC:
            try:
                return ADC.read(pin)
            except Exception:
                pass
        return 0.0

    def _smooth(self, buf, val):
        buf.append(val)
        return sum(buf) / len(buf)

    def read_all(self):
        h, t = None, None
        if HW and Adafruit_DHT:
            try:
                h, t = Adafruit_DHT.read_retry(
                    self.cfg["dht_type"], self.cfg["dht_pin"], retries=3
                )
            except Exception:
                pass
        else:
            h, t = 50.0, 25.0  # stub

        return {
            "t": t,
            "h": h,
            "ldr": round(self._smooth(self.ldr_buf, self._adc(self.cfg["ldr_ain"])), 4),
        }


# LCD, buzzer, LED controller
class UI:
    """LCD, buzzer, LED controller."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.lcd = None
        if HW and GPIO:
            try:
                for pin in [cfg["buzzer"], cfg["led"]]:
                    GPIO.setup(pin, GPIO.OUT)
                    GPIO.output(pin, GPIO.LOW)
            except Exception:
                pass
        try:
            self.lcd = LCD16x2(cfg["i2c_bus"], cfg["lcd_addr"])
        except Exception:
            pass

    def status(self, l1, l2=""):
        if self.lcd:
            self.lcd.render(l1, l2)
        else:
            print(f"[STATUS] {l1}\n[STATUS] {l2}")

    def beep(self, ok=True):
        dur = 0.05 if ok else 0.2
        if HW and GPIO:
            try:
                GPIO.output(self.cfg["buzzer"], GPIO.HIGH)
                time.sleep(dur)
                GPIO.output(self.cfg["buzzer"], GPIO.LOW)
            except Exception:
                pass
        else:
            print(f"[BEEP] {'OK' if ok else 'ERROR'}")

    def heartbeat(self, on):
        if HW and GPIO:
            try:
                GPIO.output(self.cfg["led"], GPIO.HIGH if on else GPIO.LOW)
            except Exception:
                pass

    def close(self):
        if self.lcd:
            self.lcd.close()
        if HW and GPIO:
            try:
                GPIO.output(self.cfg["buzzer"], GPIO.LOW)
                GPIO.output(self.cfg["led"], GPIO.LOW)
            except Exception:
                pass


# Main application
class PocketLabApp:
    """Main application."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.ui = self.sensors = self.logger = None
        self.next_sample = self.last_click = 0.0
        self.hb = self.running = False

    def init(self):
        self.ui = UI(self.cfg)
        self.ui.status("Booting", "Loading cfg")
        time.sleep(0.5)

        if HW and GPIO:
            try:
                GPIO.setup(self.cfg["button"], GPIO.IN, pull_up_down=GPIO.PUD_UP)
            except Exception:
                pass

        self.sensors = SensorManager(self.cfg)
        try:
            self.logger = Logger(self.cfg["log_dir"])
        except Exception as e:
            self.ui.status("Log Error", str(e)[:16])
            self.ui.beep(False)
            time.sleep(1)

        self.ui.status("Sensors OK", "Ready")
        self.ui.beep(True)
        time.sleep(0.5)
        self.next_sample = time.time() + self.cfg["sample_interval_s"]

    def _button_pressed(self):
        if HW and GPIO:
            try:
                if GPIO.input(self.cfg["button"]) == GPIO.LOW:
                    now = time.time()
                    if now - self.last_click >= 0.2:
                        self.last_click = now
                        return True
            except Exception:
                pass
        return False

    def snapshot(self):
        if not all([self.ui, self.sensors, self.logger]):
            return
        self.ui.status("Sampling", "DHT LDR")
        data = self.sensors.read_all()

        if data["t"] is None or data["h"] is None:
            self.ui.status("DHT Timeout", "Retrying")
            self.ui.beep(False)
            time.sleep(0.5)
            data = self.sensors.read_all()

        now = datetime.now()
        row = [
            now.isoformat(timespec="seconds"),
            f"{data['t']:.1f}" if data["t"] is not None else "NaN",
            f"{data['h']:.1f}" if data["h"] is not None else "NaN",
            f"{data['ldr']:.4f}",
        ]
        self.logger.append(row)
        self.ui.beep(True)
        self.ui.status("Saved", now.strftime("%H:%M:%S"))

    def run(self):
        self.running = True
        try:
            while self.running:
                self.hb = not self.hb
                if self.ui:
                    self.ui.heartbeat(self.hb)

                if self._button_pressed():
                    self.snapshot()
                    time.sleep(0.3)

                if time.time() >= self.next_sample:
                    self.ui.status(
                        "AUTO ON", f"Next {self.cfg['sample_interval_s']:02d}s"
                    )
                    time.sleep(0.2)
                    self.snapshot()
                    self.next_sample = time.time() + self.cfg["sample_interval_s"]

                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self._shutdown()

    def _shutdown(self):
        self.running = False
        if self.ui:
            self.ui.status("Shutting down", "Goodbye!")
            self.ui.heartbeat(False)
            time.sleep(0.5)
            self.ui.close()
        if self.logger:
            self.logger.close()
        if HW and GPIO:
            try:
                GPIO.cleanup()
            except Exception:
                pass
        print("Shutdown complete")


if __name__ == "__main__":
    print("=" * 40 + "\nPocketLab Data Logger\n" + "=" * 40)
    if not HW:
        print("WARNING: Using stubs")
    app = PocketLabApp(CFG)
    app.init()
    app.run()

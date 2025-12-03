# PocketLab Data Logger

A data logging application for the PocketBeagle that reads environmental sensors and logs data to CSV files.

## Features

- **DHT11 Sensor**: Reads temperature (°C) and humidity (%)
- **Photocell (LDR)**: Reads ambient light level via ADC
- **Button**: Click for manual snapshots
- **16x2 LCD Display**: Shows status messages via I2C
- **Buzzer**: Audio feedback (short beep for OK, long for error)
- **LED**: Heartbeat indicator
- **CSV Logging**: Timestamped data appended to a single log file

## Hardware Connections

| Component | Pin | Description |
|-----------|-----|-------------|
| LCD SDA | P1_26 | I2C1 data |
| LCD SCL | P1_28 | I2C1 clock |
| LDR | P1_19 | ADC AIN0 |
| Button | P2_02 | GPIO (active-low) |
| DHT11 Data | P2_04 | GPIO |
| Buzzer | P2_06 | GPIO output |
| LED | P2_08 | GPIO output |

## Setup Instructions

### 1. Install Required Libraries

```bash
sudo pip3 install Adafruit_BBIO Adafruit_DHT smbus2
```

### 2. Configure Pins

Run these commands to enable I2C and ADC pins:

```bash
# I2C for LCD
config-pin P1_26 i2c
config-pin P1_28 i2c

# ADC for LDR
config-pin P1_19 ain
```

### 3. Create Log Directory

```bash
sudo mkdir -p /var/lib/pocketlab/logs
sudo chown $USER /var/lib/pocketlab/logs
```

## Running the Application

### Basic Usage

```bash
cd /var/lib/cloud9/EDES301/project_01/code
python3 pocket_lab.py
```

### What to Expect

1. LCD shows "Booting..." then "Sensors OK / Ready"
2. LED blinks every 100ms
3. Press button to take a reading
4. Every 10 seconds (configurable in `CFG`)
5. Press Ctrl+C for graceful exit

## Configuration

Edit the `CFG` dictionary in `pocket_lab.py` to customize:

```python
CFG = {
    "i2c_bus": 1,              # I2C bus number
    "lcd_addr": 0x27,          # LCD I2C address
    "button": "P2_02",         # Button pin
    "dht_pin": "P2_04",        # DHT11 data pin
    "buzzer": "P2_06",         # Buzzer pin
    "led": "P2_08",            # LED pin
    "ldr_ain": "P1_19",        # LDR ADC pin
    "dht_type": 11,            # DHT sensor type (11 or 22)
    "log_dir": "/var/lib/pocketlab/logs",
    "sample_interval_s": 10,   # Auto-sample interval in seconds
}
```

## CSV Output Format

Log file is saved to `/var/lib/pocketlab/logs/pocketlab.csv`.

| Column | Description |
|--------|-------------|
| ts | ISO timestamp (e.g., 2025-12-03T14:30:00) |
| t_C | Temperature in Celsius |
| h_% | Relative humidity percentage |
| ldr | Light level (0.0 to 1.0) |

Example:
```csv
ts,t_C,h_%,ldr
2025-12-03T14:30:00,23.5,45.2,0.7823
2025-12-03T14:30:10,23.6,45.0,0.7801
```

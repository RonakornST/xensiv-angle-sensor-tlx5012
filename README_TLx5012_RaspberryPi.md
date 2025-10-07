# TLx5012 Magnetic Angle Sensor - Raspberry Pi 5 Guide

## Overview

The TLx5012 is a high-precision magnetic angle sensor that can measure absolute angular position with 15-bit resolution. This guide explains how each interface works and provides Python code for Raspberry Pi 5.

## Sensor Interfaces Explained

### 1. SSC/SPI Interface (Primary - Used in Python Code)

**How it works:**
- Uses 3-wire SPI communication (MOSI, SCK, CS)
- MOSI line is bidirectional (switches between input/output)
- 16-bit command/data format
- Command: `0x8000 | register_address | flags`
- Key register `REG_AVAL (0x0020)` contains absolute angle

**Register Reading Process:**
1. Send command word with register address
2. Sensor responds with 16-bit data
3. Optional safety word for error checking
4. Raw data is processed to extract angle

**Angle Calculation:**
```
Raw Data (16-bit) → Remove MSB → Check sign bit → Convert to signed
Angle (degrees) = (360.0 / 32768.0) * signed_raw_value
Range: -180° to +180° with 0.011° resolution
```

### 2. PWM Interface

**How it works:**
- Outputs angle as PWM duty cycle on IFA pin
- Duty cycle proportional to angle position
- Frequency configurable (244Hz to 1953Hz)
- Formula: `Duty Cycle = (Angle + 180°) / 360° * 100%`

**Configuration:**
- Set `REG_MOD_4` interface mode to PWM (1)
- Configure frequency in `IFABRES` field
- Read PWM signal with GPIO input

### 3. IIF (Incremental Interface)

**How it works:**
- Outputs quadrature signals (A/B channels) on IFA/IFB pins
- Resolution configurable: 9-bit to 12-bit (512 to 4096 pulses/revolution)
- Direction determined by phase relationship
- Suitable for motor control applications

**Configuration:**
- Set `REG_MOD_4` interface mode to IIF (0)
- Configure resolution in `IFABRES` field
- Connect to encoder inputs of motor controller

### 4. HSM (Hall Switch Mode)

**How it works:**
- Digital switching output based on angle thresholds
- Configurable switching points
- Hysteresis prevents oscillation
- Useful for position detection applications

### 5. SPC (Short-PWM-Code)

**How it works:**
- Encoded PWM output with angle and temperature data
- Requires trigger pulse on IFA pin to start measurement
- Trigger timing: 12-90 unit times (36-270µs default)
- Data encoded in PWM pulse width

**Trigger Sequence:**
1. Pull IFA low for trigger time
2. Release IFA high
3. Wait for measurement completion
4. Read encoded PWM data

## Hardware Setup - Raspberry Pi 5

### Wiring Connections

```
TLx5012 Sensor    Raspberry Pi 5
--------------    --------------
VCC            -> 3.3V (Pin 1) or 5V (Pin 2)
GND            -> GND (Pin 6)
SCK            -> GPIO 11 (Pin 23) - SPI0_SCLK
MOSI           -> GPIO 10 (Pin 19) - SPI0_MOSI  
CS             -> GPIO 8 (Pin 24) - SPI0_CE0_N
IFA            -> GPIO 9 (Pin 21) - Optional for SPC mode
```

### Enable SPI on Raspberry Pi

```bash
# Enable SPI interface
sudo raspi-config
# Navigate to: Interface Options -> SPI -> Enable

# Or edit config directly
echo 'dtparam=spi=on' | sudo tee -a /boot/config.txt
sudo reboot
```

### Install Dependencies

```bash
# Install Python SPI library
pip install spidev

# Install additional libraries for advanced features
pip install numpy matplotlib  # For data analysis/plotting
```

## Software Usage

### Basic Usage

```python
from tlx5012_raspi import TLx5012

# Initialize sensor
with TLx5012(spi_bus=0, spi_device=0) as sensor:
    # Read absolute angle
    angle = sensor.read_angle_degrees()
    print(f"Angle: {angle:.3f}°")
    
    # Read temperature
    temp = sensor.read_temperature()
    print(f"Temperature: {temp:.1f}°C")
    
    # Read all data
    data = sensor.read_all_data()
    print(data)
```

### Continuous Monitoring

```python
import time
from tlx5012_raspi import TLx5012

with TLx5012() as sensor:
    while True:
        angle = sensor.read_angle_degrees()
        print(f"Angle: {angle:7.3f}°", end='\r')
        time.sleep(0.1)
```

### Error Handling

```python
from tlx5012_raspi import TLx5012, TLx5012Error

try:
    with TLx5012() as sensor:
        # Check sensor status
        status = sensor.read_status()
        if status['voltage_error']:
            print("Warning: Voltage error detected")
            
        angle = sensor.read_angle_degrees()
        
except TLx5012Error as e:
    print(f"Sensor error: {e}")
```

## Register Map (Key Registers)

| Register | Address | Description |
|----------|---------|-------------|
| REG_STAT | 0x0000 | Status flags and error conditions |
| REG_AVAL | 0x0020 | **Absolute angle value (main data)** |
| REG_ASPD | 0x0030 | Angular speed |
| REG_AREV | 0x0040 | Revolution counter |
| REG_FSYNC | 0x0050 | Temperature data |
| REG_MOD_4 | 0x00E0 | Interface mode configuration |

## Troubleshooting

### Common Issues

1. **SPI Communication Fails**
   - Check SPI is enabled: `lsmod | grep spi`
   - Verify wiring connections
   - Check voltage levels (3.3V logic)

2. **Incorrect Angle Readings**
   - Verify magnet placement (centered, correct distance)
   - Check for magnetic interference
   - Calibrate offset if needed

3. **Temperature Readings Incorrect**
   - Allow sensor warm-up time
   - Check ambient temperature range

### Diagnostic Commands

```bash
# Check SPI devices
ls /dev/spi*

# Test SPI communication
python -c "import spidev; spi=spidev.SpiDev(); spi.open(0,0); print('SPI OK')"

# Monitor GPIO states
gpio readall  # If wiringpi installed
```

## Performance Specifications

- **Resolution:** 15-bit (0.011° per LSB)
- **Accuracy:** ±0.1° (typical)
- **Update Rate:** Up to 1 kHz
- **Temperature Range:** -40°C to +150°C
- **Supply Voltage:** 3.0V to 5.5V
- **SPI Clock:** Up to 8 MHz

## Advanced Features

### Multi-Sensor Setup

```python
# Use multiple sensors on different CS pins
sensor1 = TLx5012(spi_bus=0, spi_device=0)  # CS0
sensor2 = TLx5012(spi_bus=0, spi_device=1)  # CS1
```

### Data Logging

```python
import csv
import time
from datetime import datetime

with TLx5012() as sensor, open('angle_log.csv', 'w') as f:
    writer = csv.writer(f)
    writer.writerow(['timestamp', 'angle_deg', 'temperature'])
    
    for _ in range(1000):  # Log 1000 samples
        data = sensor.read_all_data()
        writer.writerow([
            datetime.now().isoformat(),
            data['angle_degrees'],
            data['temperature']
        ])
        time.sleep(0.1)
```

## License

MIT License - See source code for details.

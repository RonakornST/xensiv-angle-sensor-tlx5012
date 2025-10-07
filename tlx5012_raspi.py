#!/usr/bin/env python3
"""
TLx5012 Magnetic Angle Sensor Library for Raspberry Pi 5
Supports reading absolute angle via SPI interface

Author: AI Assistant
License: MIT
"""

import spidev
import time
import struct
from typing import Tuple, Optional
from enum import IntEnum

class TLx5012Error(Exception):
    """Custom exception for TLx5012 sensor errors"""
    pass

class RegisterAddress(IntEnum):
    """TLx5012 Register Addresses"""
    REG_STAT = 0x0000      # Status register
    REG_ACSTAT = 0x0010    # Activation status register
    REG_AVAL = 0x0020      # Angle value register (main angle data)
    REG_ASPD = 0x0030      # Angle speed register
    REG_AREV = 0x0040      # Angle revolution register
    REG_FSYNC = 0x0050     # Frame synchronization register
    REG_MOD_1 = 0x0060     # Interface mode1 register
    REG_SIL = 0x0070       # SIL register
    REG_MOD_2 = 0x0080     # Interface mode2 register
    REG_MOD_3 = 0x0090     # Interface mode3 register
    REG_OFFX = 0x00A0      # Offset X
    REG_OFFY = 0x00B0      # Offset Y
    REG_SYNCH = 0x00C0     # Synchronicity
    REG_IFAB = 0x00D0      # IFAB register
    REG_MOD_4 = 0x00E0     # Interface mode4 register
    REG_TCO_Y = 0x00F0     # Temperature coefficient register
    REG_ADC_X = 0x0100     # ADC X-raw value
    REG_ADC_Y = 0x0110     # ADC Y-raw value
    REG_D_MAG = 0x0140     # Angle vector magnitude
    REG_T_RAW = 0x0150     # Temperature sensor raw-value

class TLx5012:
    """
    TLx5012 Magnetic Angle Sensor Driver for Raspberry Pi
    
    This class provides methods to communicate with the TLx5012 sensor
    via SPI interface and read absolute angle measurements.
    """
    
    # Command constants
    READ_SENSOR = 0x8000
    WRITE_SENSOR = 0x5000
    
    # Bit manipulation constants
    DELETE_BIT_15 = 0x7FFF
    CHECK_BIT_14 = 0x4000
    CHANGE_UINT_TO_INT_15 = 0x8000
    
    # Calculation constants
    ANGLE_360_VAL = 360.0
    POW_2_15 = 32768.0
    
    # Temperature calculation constants
    TEMP_OFFSET = 152.0
    TEMP_DIV = 2.776
    
    def __init__(self, spi_bus: int = 0, spi_device: int = 0, max_speed_hz: int = 1000000):
        """
        Initialize TLx5012 sensor
        
        Args:
            spi_bus: SPI bus number (usually 0 on RPi)
            spi_device: SPI device number (0 or 1)
            max_speed_hz: SPI clock speed in Hz
        """
        self.spi_bus = spi_bus
        self.spi_device = spi_device
        self.max_speed_hz = max_speed_hz
        self.spi = None
        
    def __enter__(self):
        """Context manager entry"""
        self.open()
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.close()
        
    def open(self):
        """Open SPI connection"""
        try:
            self.spi = spidev.SpiDev()
            self.spi.open(self.spi_bus, self.spi_device)
            self.spi.max_speed_hz = self.max_speed_hz
            self.spi.mode = 1  # SPI mode 1 (CPOL=0, CPHA=1)
            self.spi.bits_per_word = 8
            print(f"TLx5012: SPI connection opened on bus {self.spi_bus}, device {self.spi_device}")
        except Exception as e:
            raise TLx5012Error(f"Failed to open SPI connection: {e}")
            
    def close(self):
        """Close SPI connection"""
        if self.spi:
            self.spi.close()
            self.spi = None
            print("TLx5012: SPI connection closed")
            
    def _read_register(self, register_addr: int, safety: bool = True) -> int:
        """
        Read a 16-bit register from the sensor
        
        Args:
            register_addr: Register address to read
            safety: Enable safety word checking
            
        Returns:
            16-bit register value
            
        Raises:
            TLx5012Error: If communication fails
        """
        if not self.spi:
            raise TLx5012Error("SPI connection not open")
            
        # Construct command word
        # Format: READ_SENSOR | register_address | update_flag | safety_flag
        command = self.READ_SENSOR | register_addr
        if safety:
            command |= 0x0001  # Enable safety word
            
        # Convert command to bytes (big-endian)
        cmd_bytes = [(command >> 8) & 0xFF, command & 0xFF]
        
        try:
            # Send command and receive response
            # For safety mode, we expect 4 bytes (2 data + 2 safety)
            # For normal mode, we expect 2 bytes (data only)
            response_length = 4 if safety else 2
            response = self.spi.xfer2(cmd_bytes + [0x00] * response_length)
            
            # Extract data (first 2 bytes after command)
            data = (response[2] << 8) | response[3]
            
            if safety and len(response) >= 6:
                # Extract safety word (last 2 bytes)
                safety_word = (response[4] << 8) | response[5]
                # Basic safety check (simplified)
                if safety_word == 0x0000:
                    raise TLx5012Error("Safety word indicates communication error")
                    
            return data
            
        except Exception as e:
            raise TLx5012Error(f"Failed to read register 0x{register_addr:04X}: {e}")
            
    def read_angle_raw(self) -> int:
        """
        Read raw angle value from REG_AVAL register
        
        Returns:
            Raw 15-bit signed angle value
        """
        raw_data = self._read_register(RegisterAddress.REG_AVAL)
        
        # Remove bit 15 (MSB)
        raw_data = raw_data & self.DELETE_BIT_15
        
        # Check if negative (bit 14 set)
        if raw_data & self.CHECK_BIT_14:
            raw_data = raw_data - self.CHANGE_UINT_TO_INT_15
            
        return raw_data
        
    def read_angle_degrees(self) -> float:
        """
        Read angle value in degrees
        
        Returns:
            Angle in degrees (-180.0 to +180.0)
        """
        raw_angle = self.read_angle_raw()
        angle_degrees = (self.ANGLE_360_VAL / self.POW_2_15) * raw_angle
        return angle_degrees
        
    def read_angle_radians(self) -> float:
        """
        Read angle value in radians
        
        Returns:
            Angle in radians (-π to +π)
        """
        import math
        angle_degrees = self.read_angle_degrees()
        return math.radians(angle_degrees)
        
    def read_temperature(self) -> float:
        """
        Read temperature from sensor
        
        Returns:
            Temperature in Celsius
        """
        # Temperature is in the FSYNC register (bits 9-0)
        fsync_data = self._read_register(RegisterAddress.REG_FSYNC)
        temp_raw = fsync_data & 0x03FF  # Extract lower 10 bits
        
        # Convert to Celsius using sensor formula
        temperature = (temp_raw - self.TEMP_OFFSET) / self.TEMP_DIV
        return temperature
        
    def read_revolution_count(self) -> int:
        """
        Read revolution counter
        
        Returns:
            Number of full revolutions (signed 9-bit value)
        """
        arev_data = self._read_register(RegisterAddress.REG_AREV)
        
        # Extract revolution count (bits 8-0)
        rev_count = arev_data & 0x01FF
        
        # Check if negative (bit 8 set)
        if rev_count & 0x0100:
            rev_count = rev_count - 0x0200
            
        return rev_count
        
    def read_status(self) -> dict:
        """
        Read sensor status register
        
        Returns:
            Dictionary with status flags
        """
        status_data = self._read_register(RegisterAddress.REG_STAT)
        
        return {
            'reset': bool(status_data & 0x0001),
            'watchdog': bool(status_data & 0x0002),
            'voltage_error': bool(status_data & 0x0004),
            'fuse_crc_error': bool(status_data & 0x0008),
            'dspu_error': bool(status_data & 0x0010),
            'overflow': bool(status_data & 0x0020),
            'xy_out_of_limit': bool(status_data & 0x0040),
            'magnitude_out_of_limit': bool(status_data & 0x0080),
            'adc_error': bool(status_data & 0x0100),
            'rom_error': bool(status_data & 0x0200),
            'gmr_xy_error': bool(status_data & 0x0400),
            'gmr_angle_error': bool(status_data & 0x0800),
        }
        
    def read_all_data(self) -> dict:
        """
        Read all sensor data in one call
        
        Returns:
            Dictionary with all sensor readings
        """
        return {
            'angle_degrees': self.read_angle_degrees(),
            'angle_radians': self.read_angle_radians(),
            'angle_raw': self.read_angle_raw(),
            'temperature': self.read_temperature(),
            'revolution_count': self.read_revolution_count(),
            'status': self.read_status()
        }

def main():
    """
    Example usage of TLx5012 sensor
    """
    print("TLx5012 Magnetic Angle Sensor - Raspberry Pi 5 Example")
    print("=" * 50)
    
    try:
        # Initialize sensor (adjust SPI bus/device as needed)
        with TLx5012(spi_bus=0, spi_device=0, max_speed_hz=1000000) as sensor:
            print("Sensor initialized successfully!")
            
            # Read sensor status
            status = sensor.read_status()
            print(f"Sensor Status: {status}")
            
            # Continuous reading loop
            print("\nReading angle data (Ctrl+C to stop):")
            print("Time\t\tAngle (°)\tAngle (rad)\tTemp (°C)\tRevolutions")
            print("-" * 70)
            
            while True:
                try:
                    # Read all data
                    data = sensor.read_all_data()
                    
                    # Format and display
                    timestamp = time.strftime("%H:%M:%S")
                    angle_deg = data['angle_degrees']
                    angle_rad = data['angle_radians']
                    temperature = data['temperature']
                    revolutions = data['revolution_count']
                    
                    print(f"{timestamp}\t{angle_deg:8.3f}\t{angle_rad:8.4f}\t{temperature:7.2f}\t{revolutions:6d}")
                    
                    time.sleep(0.1)  # 10 Hz update rate
                    
                except KeyboardInterrupt:
                    print("\nStopping...")
                    break
                except TLx5012Error as e:
                    print(f"Sensor error: {e}")
                    time.sleep(1)
                    
    except TLx5012Error as e:
        print(f"Failed to initialize sensor: {e}")
        print("\nTroubleshooting:")
        print("1. Check SPI is enabled: sudo raspi-config -> Interface Options -> SPI")
        print("2. Check wiring:")
        print("   - VCC -> 3.3V or 5V")
        print("   - GND -> GND")
        print("   - SCK -> GPIO 11 (SPI0_SCLK)")
        print("   - MOSI -> GPIO 10 (SPI0_MOSI)")
        print("   - CS -> GPIO 8 (SPI0_CE0_N)")
        print("3. Install spidev: pip install spidev")

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
TLx5012 IIF (Incremental Interface) Implementation for Raspberry Pi 5
Reads quadrature encoder signals from IFA/IFB pins

Author: AI Assistant
License: MIT
"""

import time
import threading
from typing import Optional, Callable
import RPi.GPIO as GPIO
from tlx5012_raspi import TLx5012, TLx5012Error, RegisterAddress

class TLx5012IIFError(Exception):
    """Custom exception for IIF interface errors"""
    pass

class TLx5012IIF:
    """
    TLx5012 IIF (Incremental Interface) Reader for Raspberry Pi
    
    This class configures the sensor for IIF output and reads
    quadrature encoder signals from IFA/IFB pins.
    """
    
    # IIF resolution settings (IFABRES field values)
    IIF_RES_12BIT = 0  # 4096 pulses/rev, 0.088° step
    IIF_RES_11BIT = 1  # 2048 pulses/rev, 0.176° step
    IIF_RES_10BIT = 2  # 1024 pulses/rev, 0.352° step
    IIF_RES_9BIT = 3   # 512 pulses/rev, 0.703° step
    
    # IIF mode settings (IIFMOD field values)
    IIF_MODE_DISABLED = 0
    IIF_MODE_AB = 1        # A/B quadrature operation
    IIF_MODE_STEP_DIR = 2  # Step/Direction operation
    
    # Interface mode for IIF
    INTERFACE_IIF = 0
    
    def __init__(self, spi_bus: int = 0, spi_device: int = 0,
                 ifa_pin: int = 17, ifb_pin: int = 18, 
                 resolution: int = IIF_RES_12BIT, mode: int = IIF_MODE_AB):
        """
        Initialize TLx5012 IIF interface
        
        Args:
            spi_bus: SPI bus number for configuration
            spi_device: SPI device number for configuration
            ifa_pin: GPIO pin for IFA signal (Channel A)
            ifb_pin: GPIO pin for IFB signal (Channel B)
            resolution: IIF resolution setting (0-3)
            mode: IIF mode (1=A/B, 2=Step/Dir)
        """
        self.spi_bus = spi_bus
        self.spi_device = spi_device
        self.ifa_pin = ifa_pin
        self.ifb_pin = ifb_pin
        self.resolution = resolution
        self.mode = mode
        
        # Encoder state variables
        self._position = 0
        self._last_a = 0
        self._last_b = 0
        self._direction = 1  # 1 for forward, -1 for reverse
        self._measurement_lock = threading.Lock()
        self._measuring = False
        
        # Resolution mapping
        self._pulses_per_rev = {
            self.IIF_RES_12BIT: 4096,
            self.IIF_RES_11BIT: 2048,
            self.IIF_RES_10BIT: 1024,
            self.IIF_RES_9BIT: 512
        }
        
        self._degrees_per_step = {
            self.IIF_RES_12BIT: 0.087890625,  # 360/4096
            self.IIF_RES_11BIT: 0.17578125,   # 360/2048
            self.IIF_RES_10BIT: 0.3515625,    # 360/1024
            self.IIF_RES_9BIT: 0.703125       # 360/512
        }
        
        self.spi_sensor = None
        
    def __enter__(self):
        """Context manager entry"""
        self.open()
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.close()
        
    def open(self):
        """Initialize IIF interface"""
        try:
            # Initialize SPI sensor for configuration
            self.spi_sensor = TLx5012(self.spi_bus, self.spi_device)
            self.spi_sensor.open()
            
            # Configure sensor for IIF mode
            self._configure_iif_mode()
            
            # Setup GPIO for quadrature input
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.ifa_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.ifb_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            # Read initial states
            self._last_a = GPIO.input(self.ifa_pin)
            self._last_b = GPIO.input(self.ifb_pin)
            
            # Setup interrupts for quadrature decoding
            GPIO.add_event_detect(self.ifa_pin, GPIO.BOTH, 
                                callback=self._quadrature_callback, 
                                bouncetime=1)
            GPIO.add_event_detect(self.ifb_pin, GPIO.BOTH, 
                                callback=self._quadrature_callback, 
                                bouncetime=1)
            
            self._measuring = True
            print(f"TLx5012 IIF: Interface opened on GPIO {self.ifa_pin}/{self.ifb_pin}")
            print(f"Resolution: {self._pulses_per_rev[self.resolution]} pulses/rev")
            
        except Exception as e:
            raise TLx5012IIFError(f"Failed to open IIF interface: {e}")
            
    def close(self):
        """Close IIF interface"""
        self._measuring = False
        
        if GPIO.getmode() is not None:
            GPIO.remove_event_detect(self.ifa_pin)
            GPIO.remove_event_detect(self.ifb_pin)
            GPIO.cleanup([self.ifa_pin, self.ifb_pin])
            
        if self.spi_sensor:
            self.spi_sensor.close()
            self.spi_sensor = None
            
        print("TLx5012 IIF: Interface closed")
        
    def _configure_iif_mode(self):
        """Configure sensor for IIF output mode"""
        try:
            # Configure MOD_1 register for IIF mode
            mod1_data = self.spi_sensor._read_register(RegisterAddress.REG_MOD_1)
            
            # Set IIF mode (bits 1:0)
            mod1_data = (mod1_data & 0xFFFC) | self.mode
            
            # Write back MOD_1 register
            self._write_register(RegisterAddress.REG_MOD_1, mod1_data)
            
            # Configure MOD_4 register
            mod4_data = self.spi_sensor._read_register(RegisterAddress.REG_MOD_4)
            
            # Set interface mode to IIF (bits 1:0 = 00)
            mod4_data = mod4_data & 0xFFFC
            
            # Set IIF resolution (bits 4:3)
            mod4_data = (mod4_data & 0xFFE7) | (self.resolution << 3)
            
            # Write back MOD_4 register
            self._write_register(RegisterAddress.REG_MOD_4, mod4_data)
            
            # Configure IFAB register for push-pull output
            ifab_data = self.spi_sensor._read_register(RegisterAddress.REG_IFAB)
            
            # Enable push-pull output (clear open-drain bit)
            ifab_data = ifab_data & 0xFFFE
            
            # Write back IFAB register
            self._write_register(RegisterAddress.REG_IFAB, ifab_data)
            
            mode_str = "A/B" if self.mode == self.IIF_MODE_AB else "Step/Dir"
            print(f"IIF mode configured: {mode_str}, {self._pulses_per_rev[self.resolution]} pulses/rev")
            
        except Exception as e:
            raise TLx5012IIFError(f"Failed to configure IIF mode: {e}")
            
    def _write_register(self, register_addr: int, data: int):
        """Write to sensor register"""
        if not self.spi_sensor or not self.spi_sensor.spi:
            raise TLx5012IIFError("SPI connection not available")
            
        # Construct write command
        command = 0x5000 | register_addr  # WRITE_SENSOR | address
        
        # Convert to bytes
        cmd_bytes = [(command >> 8) & 0xFF, command & 0xFF,
                     (data >> 8) & 0xFF, data & 0xFF]
        
        try:
            response = self.spi_sensor.spi.xfer2(cmd_bytes)
            time.sleep(0.001)  # Small delay after write
        except Exception as e:
            raise TLx5012IIFError(f"Failed to write register 0x{register_addr:04X}: {e}")
            
    def _quadrature_callback(self, channel):
        """GPIO interrupt callback for quadrature decoding"""
        if not self._measuring:
            return
            
        # Read current states
        a = GPIO.input(self.ifa_pin)
        b = GPIO.input(self.ifb_pin)
        
        with self._measurement_lock:
            # Quadrature decoding state machine
            # Standard quadrature encoding: A leads B for forward rotation
            if self.mode == self.IIF_MODE_AB:
                # A/B quadrature mode
                if self._last_a != a:  # A changed
                    if a == b:
                        self._position += 1
                        self._direction = 1
                    else:
                        self._position -= 1
                        self._direction = -1
                        
                elif self._last_b != b:  # B changed
                    if a != b:
                        self._position += 1
                        self._direction = 1
                    else:
                        self._position -= 1
                        self._direction = -1
                        
            elif self.mode == self.IIF_MODE_STEP_DIR:
                # Step/Direction mode
                # IFA = Step pulse, IFB = Direction
                if channel == self.ifa_pin and a != self._last_a and a == 1:  # Rising edge on step
                    if b == 1:  # Direction high = forward
                        self._position += 1
                        self._direction = 1
                    else:  # Direction low = reverse
                        self._position -= 1
                        self._direction = -1
                        
            # Update last states
            self._last_a = a
            self._last_b = b
            
    def reset_position(self):
        """Reset encoder position to zero"""
        with self._measurement_lock:
            self._position = 0
            
    def read_position_raw(self) -> int:
        """
        Read raw encoder position in pulses
        
        Returns:
            Position in encoder pulses
        """
        with self._measurement_lock:
            return self._position
            
    def read_angle_degrees(self) -> float:
        """
        Read angle in degrees
        
        Returns:
            Angle in degrees (continuous, not limited to ±180°)
        """
        with self._measurement_lock:
            return self._position * self._degrees_per_step[self.resolution]
            
    def read_angle_radians(self) -> float:
        """
        Read angle in radians
        
        Returns:
            Angle in radians (continuous)
        """
        import math
        return math.radians(self.read_angle_degrees())
        
    def read_revolutions(self) -> float:
        """
        Read number of complete revolutions
        
        Returns:
            Number of revolutions (can be fractional)
        """
        with self._measurement_lock:
            return self._position / self._pulses_per_rev[self.resolution]
            
    def read_direction(self) -> int:
        """
        Read current rotation direction
        
        Returns:
            1 for forward, -1 for reverse
        """
        with self._measurement_lock:
            return self._direction
            
    def read_iif_counter(self) -> int:
        """
        Read internal IIF counter from sensor
        
        Returns:
            14-bit IIF counter value
        """
        try:
            counter_data = self.spi_sensor._read_register(0x0200)  # REG_IIF_CNT
            return counter_data & 0x7FFF  # 14-bit counter
        except Exception as e:
            raise TLx5012IIFError(f"Failed to read IIF counter: {e}")
            
    def get_iif_info(self) -> dict:
        """
        Get comprehensive IIF information
        
        Returns:
            Dictionary with IIF measurements
        """
        with self._measurement_lock:
            angle_deg = self._position * self._degrees_per_step[self.resolution]
            revolutions = self._position / self._pulses_per_rev[self.resolution]
            
            return {
                'position_pulses': self._position,
                'angle_degrees': angle_deg,
                'angle_radians': self.read_angle_radians(),
                'revolutions': revolutions,
                'direction': self._direction,
                'resolution_bits': 12 - self.resolution,
                'pulses_per_rev': self._pulses_per_rev[self.resolution],
                'degrees_per_step': self._degrees_per_step[self.resolution],
                'mode': 'A/B' if self.mode == self.IIF_MODE_AB else 'Step/Dir'
            }

def main():
    """
    Example usage of TLx5012 IIF interface
    """
    print("TLx5012 IIF Interface - Raspberry Pi 5 Example")
    print("=" * 50)
    
    try:
        # Initialize IIF interface
        with TLx5012IIF(spi_bus=0, spi_device=0, 
                       ifa_pin=17, ifb_pin=18,
                       resolution=TLx5012IIF.IIF_RES_12BIT,
                       mode=TLx5012IIF.IIF_MODE_AB) as iif_sensor:
            
            print("IIF interface initialized successfully!")
            
            # Wait for signals to stabilize
            print("Waiting for encoder signals to stabilize...")
            time.sleep(1)
            
            # Reset position
            iif_sensor.reset_position()
            print("Position reset to zero")
            
            # Continuous reading
            print("\nReading IIF encoder data (Ctrl+C to stop):")
            print("Time\t\tPosition\tAngle (°)\tRevs\tDir")
            print("-" * 60)
            
            while True:
                try:
                    info = iif_sensor.get_iif_info()
                    
                    timestamp = time.strftime("%H:%M:%S")
                    position = info['position_pulses']
                    angle = info['angle_degrees']
                    revs = info['revolutions']
                    direction = "CW" if info['direction'] > 0 else "CCW"
                    
                    print(f"{timestamp}\t{position:8d}\t{angle:8.3f}\t{revs:6.3f}\t{direction}")
                    
                    time.sleep(0.1)
                    
                except KeyboardInterrupt:
                    print("\nStopping...")
                    break
                except Exception as e:
                    print(f"Measurement error: {e}")
                    time.sleep(1)
                    
    except TLx5012IIFError as e:
        print(f"IIF interface error: {e}")
        print("\nTroubleshooting:")
        print("1. Check SPI connection for sensor configuration")
        print("2. Check IFA/IFB pin connections")
        print("3. Verify sensor is configured for IIF output")
        print("4. Check GPIO permissions: sudo usermod -a -G gpio $USER")

if __name__ == "__main__":
    main()

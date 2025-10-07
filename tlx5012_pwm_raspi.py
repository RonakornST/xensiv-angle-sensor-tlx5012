#!/usr/bin/env python3
"""
TLx5012 PWM Interface Implementation for Raspberry Pi 5
Reads angle data from PWM duty cycle output

Author: AI Assistant
License: MIT
"""

import time
import threading
from typing import Optional, Callable
import RPi.GPIO as GPIO
from tlx5012_raspi import TLx5012, TLx5012Error, RegisterAddress

class TLx5012PWMError(Exception):
    """Custom exception for PWM interface errors"""
    pass

class TLx5012PWM:
    """
    TLx5012 PWM Interface Reader for Raspberry Pi
    
    This class configures the sensor for PWM output and reads
    the angle from PWM duty cycle measurements.
    """
    
    # PWM frequency settings (IFABRES field values)
    PWM_FREQ_244HZ = 0
    PWM_FREQ_488HZ = 1
    PWM_FREQ_977HZ = 2
    PWM_FREQ_1953HZ = 3
    
    # Interface mode for PWM
    INTERFACE_PWM = 1
    
    def __init__(self, spi_bus: int = 0, spi_device: int = 0, 
                 pwm_pin: int = 18, pwm_frequency: int = PWM_FREQ_977HZ):
        """
        Initialize TLx5012 PWM interface
        
        Args:
            spi_bus: SPI bus number for configuration
            spi_device: SPI device number for configuration  
            pwm_pin: GPIO pin number for PWM input (default: 18)
            pwm_frequency: PWM frequency setting (0-3)
        """
        self.spi_bus = spi_bus
        self.spi_device = spi_device
        self.pwm_pin = pwm_pin
        self.pwm_frequency = pwm_frequency
        
        # PWM measurement variables
        self._pulse_start = 0
        self._pulse_width = 0
        self._period = 0
        self._duty_cycle = 0
        self._last_angle = 0
        self._measurement_lock = threading.Lock()
        self._measuring = False
        
        # Expected frequencies in Hz
        self._freq_map = {
            self.PWM_FREQ_244HZ: 244,
            self.PWM_FREQ_488HZ: 488, 
            self.PWM_FREQ_977HZ: 977,
            self.PWM_FREQ_1953HZ: 1953
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
        """Initialize PWM interface"""
        try:
            # Initialize SPI sensor for configuration
            self.spi_sensor = TLx5012(self.spi_bus, self.spi_device)
            self.spi_sensor.open()
            
            # Configure sensor for PWM mode
            self._configure_pwm_mode()
            
            # Setup GPIO for PWM input
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.pwm_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            # Setup interrupt for PWM measurement
            GPIO.add_event_detect(self.pwm_pin, GPIO.BOTH, 
                                callback=self._pwm_edge_callback, 
                                bouncetime=1)
            
            self._measuring = True
            print(f"TLx5012 PWM: Interface opened on GPIO {self.pwm_pin}")
            
        except Exception as e:
            raise TLx5012PWMError(f"Failed to open PWM interface: {e}")
            
    def close(self):
        """Close PWM interface"""
        self._measuring = False
        
        if GPIO.getmode() is not None:
            GPIO.remove_event_detect(self.pwm_pin)
            GPIO.cleanup(self.pwm_pin)
            
        if self.spi_sensor:
            self.spi_sensor.close()
            self.spi_sensor = None
            
        print("TLx5012 PWM: Interface closed")
        
    def _configure_pwm_mode(self):
        """Configure sensor for PWM output mode"""
        try:
            # Read current MOD_4 register
            mod4_data = self.spi_sensor._read_register(RegisterAddress.REG_MOD_4)
            
            # Set interface mode to PWM (bits 1:0 = 01)
            mod4_data = (mod4_data & 0xFFFC) | self.INTERFACE_PWM
            
            # Set PWM frequency (bits 4:3)
            mod4_data = (mod4_data & 0xFFE7) | (self.pwm_frequency << 3)
            
            # Write back to sensor
            self._write_register(RegisterAddress.REG_MOD_4, mod4_data)
            
            # Configure IFAB register for PWM output
            ifab_data = self.spi_sensor._read_register(RegisterAddress.REG_IFAB)
            
            # Enable push-pull output (clear open-drain bit)
            ifab_data = ifab_data & 0xFFFE
            
            # Write back IFAB register
            self._write_register(RegisterAddress.REG_IFAB, ifab_data)
            
            print(f"PWM mode configured: {self._freq_map[self.pwm_frequency]}Hz")
            
        except Exception as e:
            raise TLx5012PWMError(f"Failed to configure PWM mode: {e}")
            
    def _write_register(self, register_addr: int, data: int):
        """Write to sensor register"""
        if not self.spi_sensor or not self.spi_sensor.spi:
            raise TLx5012PWMError("SPI connection not available")
            
        # Construct write command
        command = 0x5000 | register_addr  # WRITE_SENSOR | address
        
        # Convert to bytes
        cmd_bytes = [(command >> 8) & 0xFF, command & 0xFF,
                     (data >> 8) & 0xFF, data & 0xFF]
        
        try:
            response = self.spi_sensor.spi.xfer2(cmd_bytes)
            time.sleep(0.001)  # Small delay after write
        except Exception as e:
            raise TLx5012PWMError(f"Failed to write register 0x{register_addr:04X}: {e}")
            
    def _pwm_edge_callback(self, channel):
        """GPIO interrupt callback for PWM edge detection"""
        if not self._measuring:
            return
            
        current_time = time.time_ns()
        pin_state = GPIO.input(self.pwm_pin)
        
        with self._measurement_lock:
            if pin_state == GPIO.HIGH:  # Rising edge - start of pulse
                if self._pulse_start > 0:
                    # Calculate period from previous rising edge
                    self._period = current_time - self._pulse_start
                self._pulse_start = current_time
                
            else:  # Falling edge - end of pulse
                if self._pulse_start > 0:
                    # Calculate pulse width
                    self._pulse_width = current_time - self._pulse_start
                    
                    # Calculate duty cycle
                    if self._period > 0:
                        self._duty_cycle = (self._pulse_width / self._period) * 100.0
                        
                        # Convert duty cycle to angle
                        # Formula: Angle = (Duty_Cycle * 360 / 100) - 180
                        self._last_angle = (self._duty_cycle * 3.6) - 180.0
                        
    def read_duty_cycle(self) -> float:
        """
        Read current PWM duty cycle
        
        Returns:
            Duty cycle percentage (0-100%)
        """
        with self._measurement_lock:
            return self._duty_cycle
            
    def read_angle_degrees(self) -> float:
        """
        Read angle from PWM duty cycle
        
        Returns:
            Angle in degrees (-180 to +180)
        """
        with self._measurement_lock:
            return self._last_angle
            
    def read_angle_radians(self) -> float:
        """
        Read angle in radians
        
        Returns:
            Angle in radians (-π to +π)
        """
        import math
        return math.radians(self.read_angle_degrees())
        
    def read_frequency(self) -> float:
        """
        Read measured PWM frequency
        
        Returns:
            Frequency in Hz
        """
        with self._measurement_lock:
            if self._period > 0:
                return 1e9 / self._period  # Convert ns to Hz
            return 0.0
            
    def get_pwm_info(self) -> dict:
        """
        Get comprehensive PWM information
        
        Returns:
            Dictionary with PWM measurements
        """
        with self._measurement_lock:
            freq = 1e9 / self._period if self._period > 0 else 0.0
            return {
                'duty_cycle_percent': self._duty_cycle,
                'angle_degrees': self._last_angle,
                'angle_radians': self.read_angle_radians(),
                'frequency_hz': freq,
                'pulse_width_us': self._pulse_width / 1000.0,
                'period_us': self._period / 1000.0,
                'configured_frequency': self._freq_map[self.pwm_frequency]
            }
            
    def calibrate_pwm(self, samples: int = 100) -> dict:
        """
        Calibrate PWM measurements
        
        Args:
            samples: Number of samples for calibration
            
        Returns:
            Calibration statistics
        """
        print(f"Calibrating PWM interface with {samples} samples...")
        
        duty_cycles = []
        frequencies = []
        
        for i in range(samples):
            info = self.get_pwm_info()
            if info['frequency_hz'] > 0:  # Valid measurement
                duty_cycles.append(info['duty_cycle_percent'])
                frequencies.append(info['frequency_hz'])
            time.sleep(0.01)
            
        if not duty_cycles:
            raise TLx5012PWMError("No valid PWM measurements during calibration")
            
        # Calculate statistics
        avg_duty = sum(duty_cycles) / len(duty_cycles)
        avg_freq = sum(frequencies) / len(frequencies)
        
        duty_min = min(duty_cycles)
        duty_max = max(duty_cycles)
        
        freq_min = min(frequencies)
        freq_max = max(frequencies)
        
        calibration_data = {
            'samples': len(duty_cycles),
            'duty_cycle': {
                'average': avg_duty,
                'min': duty_min,
                'max': duty_max,
                'range': duty_max - duty_min
            },
            'frequency': {
                'average': avg_freq,
                'min': freq_min,
                'max': freq_max,
                'expected': self._freq_map[self.pwm_frequency],
                'error_percent': abs(avg_freq - self._freq_map[self.pwm_frequency]) / self._freq_map[self.pwm_frequency] * 100
            }
        }
        
        print("PWM Calibration Results:")
        print(f"  Duty Cycle: {avg_duty:.2f}% (range: {duty_max-duty_min:.2f}%)")
        print(f"  Frequency: {avg_freq:.1f}Hz (expected: {self._freq_map[self.pwm_frequency]}Hz)")
        print(f"  Frequency Error: {calibration_data['frequency']['error_percent']:.2f}%")
        
        return calibration_data

def main():
    """
    Example usage of TLx5012 PWM interface
    """
    print("TLx5012 PWM Interface - Raspberry Pi 5 Example")
    print("=" * 50)
    
    try:
        # Initialize PWM interface
        with TLx5012PWM(spi_bus=0, spi_device=0, pwm_pin=18, 
                       pwm_frequency=TLx5012PWM.PWM_FREQ_977HZ) as pwm_sensor:
            
            print("PWM interface initialized successfully!")
            
            # Wait for PWM signal to stabilize
            print("Waiting for PWM signal to stabilize...")
            time.sleep(2)
            
            # Calibrate PWM measurements
            try:
                calibration = pwm_sensor.calibrate_pwm(50)
            except TLx5012PWMError as e:
                print(f"Calibration failed: {e}")
                print("Continuing with uncalibrated measurements...")
            
            # Continuous reading
            print("\nReading PWM angle data (Ctrl+C to stop):")
            print("Time\t\tAngle (°)\tDuty (%)\tFreq (Hz)")
            print("-" * 60)
            
            while True:
                try:
                    info = pwm_sensor.get_pwm_info()
                    
                    timestamp = time.strftime("%H:%M:%S")
                    angle = info['angle_degrees']
                    duty = info['duty_cycle_percent']
                    freq = info['frequency_hz']
                    
                    print(f"{timestamp}\t{angle:8.3f}\t{duty:7.2f}\t{freq:8.1f}")
                    
                    time.sleep(0.1)
                    
                except KeyboardInterrupt:
                    print("\nStopping...")
                    break
                except Exception as e:
                    print(f"Measurement error: {e}")
                    time.sleep(1)
                    
    except TLx5012PWMError as e:
        print(f"PWM interface error: {e}")
        print("\nTroubleshooting:")
        print("1. Check SPI connection for sensor configuration")
        print("2. Check PWM pin connection (IFA pin of sensor)")
        print("3. Verify sensor is configured for PWM output")
        print("4. Check GPIO permissions: sudo usermod -a -G gpio $USER")

if __name__ == "__main__":
    main()

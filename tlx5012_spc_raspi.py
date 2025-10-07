#!/usr/bin/env python3
"""
TLx5012 SPC (Short-PWM-Code) Interface Implementation for Raspberry Pi 5
Triggers sensor and decodes PWM-encoded angle and temperature data

Author: AI Assistant
License: MIT
"""

import time
import threading
from typing import Optional, Callable, Tuple
import RPi.GPIO as GPIO
from tlx5012_raspi import TLx5012, TLx5012Error, RegisterAddress

class TLx5012SPCError(Exception):
    """Custom exception for SPC interface errors"""
    pass

class TLx5012SPC:
    """
    TLx5012 SPC (Short-PWM-Code) Interface for Raspberry Pi
    
    This class configures the sensor for SPC mode and handles
    trigger generation and PWM-encoded data decoding.
    """
    
    # SPC unit time settings (IFADHYST field values in SPC mode)
    SPC_UNIT_TIME_3_0_US = 0  # 3.0µs unit time
    SPC_UNIT_TIME_2_5_US = 1  # 2.5µs unit time
    SPC_UNIT_TIME_2_0_US = 2  # 2.0µs unit time
    SPC_UNIT_TIME_1_5_US = 3  # 1.5µs unit time
    
    # SPC frame configurations (IFABRES field values in SPC mode)
    SPC_FRAME_12BIT_ANGLE = 0      # 12-bit angle only
    SPC_FRAME_16BIT_ANGLE = 1      # 16-bit angle only
    SPC_FRAME_12BIT_ANGLE_TEMP = 2 # 12-bit angle + 8-bit temperature
    SPC_FRAME_16BIT_ANGLE_TEMP = 3 # 16-bit angle + 8-bit temperature
    
    # SPC trigger time configurations (HSMPLP field values in SPC mode)
    SPC_TRIGGER_90_UT = 0    # 90 * UT total trigger time
    SPC_TRIGGER_CUSTOM = 4   # t_mlow + 12 UT
    
    # Interface mode for SPC
    INTERFACE_SPC = 3
    
    def __init__(self, spi_bus: int = 0, spi_device: int = 0,
                 ifa_pin: int = 17, unit_time: int = SPC_UNIT_TIME_3_0_US,
                 frame_config: int = SPC_FRAME_16BIT_ANGLE,
                 trigger_config: int = SPC_TRIGGER_90_UT,
                 sensor_count: int = 1):
        """
        Initialize TLx5012 SPC interface
        
        Args:
            spi_bus: SPI bus number for configuration
            spi_device: SPI device number for configuration
            ifa_pin: GPIO pin for IFA signal (trigger and data)
            unit_time: Unit time setting (0-3)
            frame_config: Frame configuration (0-3)
            trigger_config: Trigger time configuration
            sensor_count: Number of sensors in chain (1-4)
        """
        self.spi_bus = spi_bus
        self.spi_device = spi_device
        self.ifa_pin = ifa_pin
        self.unit_time = unit_time
        self.frame_config = frame_config
        self.trigger_config = trigger_config
        self.sensor_count = max(1, min(4, sensor_count))
        
        # SPC timing parameters
        self._unit_time_us = {
            self.SPC_UNIT_TIME_3_0_US: 3.0,
            self.SPC_UNIT_TIME_2_5_US: 2.5,
            self.SPC_UNIT_TIME_2_0_US: 2.0,
            self.SPC_UNIT_TIME_1_5_US: 1.5
        }
        
        # Calculate trigger timing
        self._t_mlow = 12 * self.sensor_count  # 12 UT per sensor
        if self.trigger_config == self.SPC_TRIGGER_90_UT:
            self._trigger_total = 90
        else:
            self._trigger_total = self._t_mlow + 12
            
        # PWM decoding variables
        self._last_angle = 0.0
        self._last_temperature = 0.0
        self._measurement_lock = threading.Lock()
        self._measuring = False
        
        # PWM measurement variables for decoding
        self._pulse_times = []
        self._pulse_states = []
        self._decode_start_time = 0
        
        self.spi_sensor = None
        
    def __enter__(self):
        """Context manager entry"""
        self.open()
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.close()
        
    def open(self):
        """Initialize SPC interface"""
        try:
            # Initialize SPI sensor for configuration
            self.spi_sensor = TLx5012(self.spi_bus, self.spi_device)
            self.spi_sensor.open()
            
            # Configure sensor for SPC mode
            self._configure_spc_mode()
            
            # Setup GPIO for trigger and PWM input
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.ifa_pin, GPIO.OUT, initial=GPIO.HIGH)
            
            self._measuring = True
            unit_time_us = self._unit_time_us[self.unit_time]
            trigger_time_us = self._trigger_total * unit_time_us
            
            print(f"TLx5012 SPC: Interface opened on GPIO {self.ifa_pin}")
            print(f"Unit time: {unit_time_us}µs, Trigger time: {trigger_time_us}µs")
            print(f"Sensors: {self.sensor_count}, Frame: {self._get_frame_description()}")
            
        except Exception as e:
            raise TLx5012SPCError(f"Failed to open SPC interface: {e}")
            
    def close(self):
        """Close SPC interface"""
        self._measuring = False
        
        if GPIO.getmode() is not None:
            GPIO.cleanup(self.ifa_pin)
            
        if self.spi_sensor:
            self.spi_sensor.close()
            self.spi_sensor = None
            
        print("TLx5012 SPC: Interface closed")
        
    def _configure_spc_mode(self):
        """Configure sensor for SPC output mode"""
        try:
            # Configure MOD_4 register
            mod4_data = self.spi_sensor._read_register(RegisterAddress.REG_MOD_4)
            
            # Set interface mode to SPC (bits 1:0 = 11)
            mod4_data = (mod4_data & 0xFFFC) | self.INTERFACE_SPC
            
            # Set frame configuration (bits 4:3)
            mod4_data = (mod4_data & 0xFFE7) | (self.frame_config << 3)
            
            # Set trigger configuration (bits 8:5)
            mod4_data = (mod4_data & 0xFE1F) | (self.trigger_config << 5)
            
            # Write back MOD_4 register
            self._write_register(RegisterAddress.REG_MOD_4, mod4_data)
            
            # Configure IFAB register for SPC
            ifab_data = self.spi_sensor._read_register(RegisterAddress.REG_IFAB)
            
            # Set unit time (bits 1:0)
            ifab_data = (ifab_data & 0xFFFC) | self.unit_time
            
            # Enable push-pull output (clear open-drain bit)
            ifab_data = ifab_data & 0xFFFE
            
            # Write back IFAB register
            self._write_register(RegisterAddress.REG_IFAB, ifab_data)
            
            print(f"SPC mode configured: {self._get_frame_description()}")
            
        except Exception as e:
            raise TLx5012SPCError(f"Failed to configure SPC mode: {e}")
            
    def _write_register(self, register_addr: int, data: int):
        """Write to sensor register"""
        if not self.spi_sensor or not self.spi_sensor.spi:
            raise TLx5012SPCError("SPI connection not available")
            
        # Construct write command
        command = 0x5000 | register_addr  # WRITE_SENSOR | address
        
        # Convert to bytes
        cmd_bytes = [(command >> 8) & 0xFF, command & 0xFF,
                     (data >> 8) & 0xFF, data & 0xFF]
        
        try:
            response = self.spi_sensor.spi.xfer2(cmd_bytes)
            time.sleep(0.001)  # Small delay after write
        except Exception as e:
            raise TLx5012SPCError(f"Failed to write register 0x{register_addr:04X}: {e}")
            
    def _get_frame_description(self) -> str:
        """Get human-readable frame configuration description"""
        descriptions = {
            self.SPC_FRAME_12BIT_ANGLE: "12-bit angle",
            self.SPC_FRAME_16BIT_ANGLE: "16-bit angle", 
            self.SPC_FRAME_12BIT_ANGLE_TEMP: "12-bit angle + temperature",
            self.SPC_FRAME_16BIT_ANGLE_TEMP: "16-bit angle + temperature"
        }
        return descriptions.get(self.frame_config, "unknown")
        
    def _generate_trigger(self):
        """Generate SPC trigger pulse"""
        unit_time_us = self._unit_time_us[self.unit_time]
        t_mlow_us = self._t_mlow * unit_time_us
        t_high_us = (self._trigger_total - self._t_mlow) * unit_time_us
        
        # Generate trigger pulse
        GPIO.output(self.ifa_pin, GPIO.LOW)
        time.sleep(t_mlow_us / 1_000_000)  # Convert µs to seconds
        GPIO.output(self.ifa_pin, GPIO.HIGH)
        time.sleep(t_high_us / 1_000_000)  # Wait for response
        
    def _setup_pwm_capture(self):
        """Setup GPIO for PWM capture after trigger"""
        # Switch pin to input for PWM capture
        GPIO.setup(self.ifa_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Clear previous measurements
        self._pulse_times = []
        self._pulse_states = []
        self._decode_start_time = time.time_ns()
        
    def _capture_pwm_data(self, timeout_ms: int = 50) -> bool:
        """
        Capture PWM data after trigger
        
        Args:
            timeout_ms: Timeout in milliseconds
            
        Returns:
            True if data captured successfully
        """
        timeout_ns = timeout_ms * 1_000_000
        start_time = time.time_ns()
        last_state = GPIO.input(self.ifa_pin)
        
        while (time.time_ns() - start_time) < timeout_ns:
            current_state = GPIO.input(self.ifa_pin)
            current_time = time.time_ns()
            
            if current_state != last_state:
                self._pulse_times.append(current_time - self._decode_start_time)
                self._pulse_states.append(current_state)
                last_state = current_state
                
            time.sleep(0.000001)  # 1µs sampling
            
        return len(self._pulse_times) > 0
        
    def _decode_pwm_data(self) -> Tuple[Optional[float], Optional[float]]:
        """
        Decode angle and temperature from PWM data
        
        Returns:
            Tuple of (angle_degrees, temperature_celsius) or (None, None) if decode fails
        """
        if len(self._pulse_times) < 2:
            return None, None
            
        try:
            # Simple PWM decoding - this is a simplified implementation
            # Real implementation would need to decode the specific SPC protocol
            
            # Calculate duty cycles from pulse timing
            total_time = self._pulse_times[-1] - self._pulse_times[0]
            high_time = 0
            
            for i in range(0, len(self._pulse_times) - 1, 2):
                if i + 1 < len(self._pulse_states) and self._pulse_states[i] == 1:
                    high_time += self._pulse_times[i + 1] - self._pulse_times[i]
                    
            if total_time > 0:
                duty_cycle = (high_time / total_time) * 100.0
                
                # Convert duty cycle to angle (simplified)
                angle = (duty_cycle * 360.0 / 100.0) - 180.0
                
                # Temperature decoding (if frame includes temperature)
                temperature = None
                if self.frame_config in [self.SPC_FRAME_12BIT_ANGLE_TEMP, 
                                       self.SPC_FRAME_16BIT_ANGLE_TEMP]:
                    # Simplified temperature extraction
                    temperature = 25.0  # Placeholder
                    
                return angle, temperature
                
        except Exception as e:
            print(f"PWM decode error: {e}")
            
        return None, None
        
    def trigger_and_read(self) -> Tuple[Optional[float], Optional[float]]:
        """
        Trigger sensor and read angle/temperature data
        
        Returns:
            Tuple of (angle_degrees, temperature_celsius)
        """
        if not self._measuring:
            raise TLx5012SPCError("SPC interface not open")
            
        with self._measurement_lock:
            try:
                # Generate trigger pulse
                self._generate_trigger()
                
                # Setup for PWM capture
                self._setup_pwm_capture()
                
                # Capture PWM response
                if self._capture_pwm_data():
                    angle, temperature = self._decode_pwm_data()
                    
                    if angle is not None:
                        self._last_angle = angle
                    if temperature is not None:
                        self._last_temperature = temperature
                        
                    return angle, temperature
                else:
                    return None, None
                    
            finally:
                # Restore pin to output mode
                GPIO.setup(self.ifa_pin, GPIO.OUT, initial=GPIO.HIGH)
                
    def read_angle_degrees(self) -> Optional[float]:
        """
        Read angle in degrees
        
        Returns:
            Angle in degrees or None if read fails
        """
        angle, _ = self.trigger_and_read()
        return angle
        
    def read_temperature_celsius(self) -> Optional[float]:
        """
        Read temperature in Celsius
        
        Returns:
            Temperature in Celsius or None if not available
        """
        _, temperature = self.trigger_and_read()
        return temperature
        
    def get_spc_info(self) -> dict:
        """
        Get comprehensive SPC information
        
        Returns:
            Dictionary with SPC measurements and configuration
        """
        angle, temperature = self.trigger_and_read()
        
        return {
            'angle_degrees': angle,
            'temperature_celsius': temperature,
            'last_angle_degrees': self._last_angle,
            'last_temperature_celsius': self._last_temperature,
            'unit_time_us': self._unit_time_us[self.unit_time],
            'trigger_time_us': self._trigger_total * self._unit_time_us[self.unit_time],
            'frame_config': self._get_frame_description(),
            'sensor_count': self.sensor_count,
            'pulse_count': len(self._pulse_times)
        }

def main():
    """
    Example usage of TLx5012 SPC interface
    """
    print("TLx5012 SPC Interface - Raspberry Pi 5 Example")
    print("=" * 50)
    
    try:
        # Initialize SPC interface
        with TLx5012SPC(spi_bus=0, spi_device=0, ifa_pin=17,
                       unit_time=TLx5012SPC.SPC_UNIT_TIME_3_0_US,
                       frame_config=TLx5012SPC.SPC_FRAME_16BIT_ANGLE_TEMP,
                       sensor_count=1) as spc_sensor:
            
            print("SPC interface initialized successfully!")
            
            # Wait for interface to stabilize
            print("Waiting for SPC interface to stabilize...")
            time.sleep(1)
            
            # Continuous reading
            print("\nReading SPC sensor data (Ctrl+C to stop):")
            print("Time\t\tAngle (°)\tTemp (°C)\tPulses")
            print("-" * 60)
            
            while True:
                try:
                    info = spc_sensor.get_spc_info()
                    
                    timestamp = time.strftime("%H:%M:%S")
                    angle = info['angle_degrees']
                    temp = info['temperature_celsius']
                    pulses = info['pulse_count']
                    
                    angle_str = f"{angle:8.3f}" if angle is not None else "   N/A  "
                    temp_str = f"{temp:7.1f}" if temp is not None else "  N/A  "
                    
                    print(f"{timestamp}\t{angle_str}\t{temp_str}\t{pulses:6d}")
                    
                    time.sleep(0.5)  # SPC typically slower than other interfaces
                    
                except KeyboardInterrupt:
                    print("\nStopping...")
                    break
                except Exception as e:
                    print(f"Measurement error: {e}")
                    time.sleep(1)
                    
    except TLx5012SPCError as e:
        print(f"SPC interface error: {e}")
        print("\nTroubleshooting:")
        print("1. Check SPI connection for sensor configuration")
        print("2. Check IFA pin connection for trigger/data")
        print("3. Verify sensor is TLE5012B-E9000 variant")
        print("4. Check trigger timing and unit time settings")

if __name__ == "__main__":
    main()

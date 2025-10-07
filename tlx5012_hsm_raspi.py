#!/usr/bin/env python3
"""
TLx5012 HSM (Hall Switch Mode) Interface Implementation for Raspberry Pi 5
Reads digital switching signals based on angle thresholds

Author: AI Assistant
License: MIT
"""

import time
import threading
from typing import Optional, Callable, List
import RPi.GPIO as GPIO
from tlx5012_raspi import TLx5012, TLx5012Error, RegisterAddress

class TLx5012HSMError(Exception):
    """Custom exception for HSM interface errors"""
    pass

class TLx5012HSM:
    """
    TLx5012 HSM (Hall Switch Mode) Interface Reader for Raspberry Pi
    
    This class configures the sensor for HSM output and reads
    digital switching signals based on configurable angle thresholds.
    """
    
    # HSM hysteresis settings (IFADHYST field values)
    HSM_HYSTERESIS_0_DEG = 0      # 0° hysteresis
    HSM_HYSTERESIS_0_175_DEG = 1  # 0.175° hysteresis
    HSM_HYSTERESIS_0_35_DEG = 2   # 0.35° hysteresis
    HSM_HYSTERESIS_0_70_DEG = 3   # 0.70° hysteresis
    
    # HSM pole pair settings (1-16 pole pairs)
    HSM_POLE_PAIRS_MIN = 1
    HSM_POLE_PAIRS_MAX = 16
    
    # Interface mode for HSM
    INTERFACE_HSM = 2
    
    def __init__(self, spi_bus: int = 0, spi_device: int = 0,
                 ifa_pin: int = 17, ifb_pin: int = 18, ifc_pin: int = 19,
                 pole_pairs: int = 1, hysteresis: int = HSM_HYSTERESIS_0_35_DEG):
        """
        Initialize TLx5012 HSM interface
        
        Args:
            spi_bus: SPI bus number for configuration
            spi_device: SPI device number for configuration
            ifa_pin: GPIO pin for IFA signal (Hall A)
            ifb_pin: GPIO pin for IFB signal (Hall B)
            ifc_pin: GPIO pin for IFC signal (Hall C/Index)
            pole_pairs: Number of pole pairs (1-16)
            hysteresis: Hysteresis setting (0-3)
        """
        self.spi_bus = spi_bus
        self.spi_device = spi_device
        self.ifa_pin = ifa_pin
        self.ifb_pin = ifb_pin
        self.ifc_pin = ifc_pin
        self.pole_pairs = max(1, min(16, pole_pairs))
        self.hysteresis = hysteresis
        
        # HSM state variables
        self._hall_states = {'A': 0, 'B': 0, 'C': 0}
        self._last_hall_states = {'A': 0, 'B': 0, 'C': 0}
        self._sector = 0
        self._position_electrical = 0
        self._position_mechanical = 0
        self._direction = 1  # 1 for forward, -1 for reverse
        self._measurement_lock = threading.Lock()
        self._measuring = False
        
        # Hall state transition table for 3-phase motor (6 sectors)
        # Standard sequence: 001 -> 011 -> 010 -> 110 -> 100 -> 101 -> 001
        self._hall_sequence_forward = [
            0b001,  # Sector 0
            0b011,  # Sector 1
            0b010,  # Sector 2
            0b110,  # Sector 3
            0b100,  # Sector 4
            0b101   # Sector 5
        ]
        
        # Hysteresis values in degrees
        self._hysteresis_values = {
            self.HSM_HYSTERESIS_0_DEG: 0.0,
            self.HSM_HYSTERESIS_0_175_DEG: 0.175,
            self.HSM_HYSTERESIS_0_35_DEG: 0.35,
            self.HSM_HYSTERESIS_0_70_DEG: 0.70
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
        """Initialize HSM interface"""
        try:
            # Initialize SPI sensor for configuration
            self.spi_sensor = TLx5012(self.spi_bus, self.spi_device)
            self.spi_sensor.open()
            
            # Configure sensor for HSM mode
            self._configure_hsm_mode()
            
            # Setup GPIO for Hall sensor inputs
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.ifa_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.ifb_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.ifc_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            # Read initial Hall states
            self._update_hall_states()
            self._last_hall_states = self._hall_states.copy()
            
            # Setup interrupts for Hall state changes
            GPIO.add_event_detect(self.ifa_pin, GPIO.BOTH, 
                                callback=self._hall_callback, 
                                bouncetime=1)
            GPIO.add_event_detect(self.ifb_pin, GPIO.BOTH, 
                                callback=self._hall_callback, 
                                bouncetime=1)
            GPIO.add_event_detect(self.ifc_pin, GPIO.BOTH, 
                                callback=self._hall_callback, 
                                bouncetime=1)
            
            self._measuring = True
            print(f"TLx5012 HSM: Interface opened on GPIO {self.ifa_pin}/{self.ifb_pin}/{self.ifc_pin}")
            print(f"Pole pairs: {self.pole_pairs}, Hysteresis: {self._hysteresis_values[self.hysteresis]}°")
            
        except Exception as e:
            raise TLx5012HSMError(f"Failed to open HSM interface: {e}")
            
    def close(self):
        """Close HSM interface"""
        self._measuring = False
        
        if GPIO.getmode() is not None:
            GPIO.remove_event_detect(self.ifa_pin)
            GPIO.remove_event_detect(self.ifb_pin)
            GPIO.remove_event_detect(self.ifc_pin)
            GPIO.cleanup([self.ifa_pin, self.ifb_pin, self.ifc_pin])
            
        if self.spi_sensor:
            self.spi_sensor.close()
            self.spi_sensor = None
            
        print("TLx5012 HSM: Interface closed")
        
    def _configure_hsm_mode(self):
        """Configure sensor for HSM output mode"""
        try:
            # Configure MOD_4 register
            mod4_data = self.spi_sensor._read_register(RegisterAddress.REG_MOD_4)
            
            # Set interface mode to HSM (bits 1:0 = 10)
            mod4_data = (mod4_data & 0xFFFC) | self.INTERFACE_HSM
            
            # Set pole pair configuration (bits 8:5)
            # HSMPLP field: 0000b=1 pole pair, 1111b=16 pole pairs
            pole_pair_code = (self.pole_pairs - 1) & 0x0F
            mod4_data = (mod4_data & 0xFE1F) | (pole_pair_code << 5)
            
            # Write back MOD_4 register
            self._write_register(RegisterAddress.REG_MOD_4, mod4_data)
            
            # Configure IFAB register for HSM
            ifab_data = self.spi_sensor._read_register(RegisterAddress.REG_IFAB)
            
            # Set hysteresis mode (bits 1:0)
            ifab_data = (ifab_data & 0xFFFC) | self.hysteresis
            
            # Enable push-pull output (clear open-drain bit if needed)
            ifab_data = ifab_data & 0xFFFE
            
            # Write back IFAB register
            self._write_register(RegisterAddress.REG_IFAB, ifab_data)
            
            print(f"HSM mode configured: {self.pole_pairs} pole pairs, {self._hysteresis_values[self.hysteresis]}° hysteresis")
            
        except Exception as e:
            raise TLx5012HSMError(f"Failed to configure HSM mode: {e}")
            
    def _write_register(self, register_addr: int, data: int):
        """Write to sensor register"""
        if not self.spi_sensor or not self.spi_sensor.spi:
            raise TLx5012HSMError("SPI connection not available")
            
        # Construct write command
        command = 0x5000 | register_addr  # WRITE_SENSOR | address
        
        # Convert to bytes
        cmd_bytes = [(command >> 8) & 0xFF, command & 0xFF,
                     (data >> 8) & 0xFF, data & 0xFF]
        
        try:
            response = self.spi_sensor.spi.xfer2(cmd_bytes)
            time.sleep(0.001)  # Small delay after write
        except Exception as e:
            raise TLx5012HSMError(f"Failed to write register 0x{register_addr:04X}: {e}")
            
    def _update_hall_states(self):
        """Update Hall sensor states from GPIO"""
        self._hall_states['A'] = GPIO.input(self.ifa_pin)
        self._hall_states['B'] = GPIO.input(self.ifb_pin)
        self._hall_states['C'] = GPIO.input(self.ifc_pin)
        
    def _hall_callback(self, channel):
        """GPIO interrupt callback for Hall state changes"""
        if not self._measuring:
            return
            
        with self._measurement_lock:
            # Update Hall states
            self._update_hall_states()
            
            # Calculate current Hall state as 3-bit value
            current_state = (self._hall_states['C'] << 2) | (self._hall_states['B'] << 1) | self._hall_states['A']
            last_state = (self._last_hall_states['C'] << 2) | (self._last_hall_states['B'] << 1) | self._last_hall_states['A']
            
            if current_state != last_state:
                # Determine sector from Hall state
                try:
                    new_sector = self._hall_sequence_forward.index(current_state)
                    
                    # Calculate direction based on sector transition
                    if self._sector != new_sector:
                        sector_diff = (new_sector - self._sector) % 6
                        if sector_diff == 1:
                            self._direction = 1  # Forward
                        elif sector_diff == 5:  # -1 mod 6
                            self._direction = -1  # Reverse
                            
                        self._sector = new_sector
                        
                        # Update electrical position (0-5 for 6 sectors)
                        self._position_electrical = self._sector
                        
                        # Update mechanical position considering pole pairs
                        # Each electrical revolution = 360° / pole_pairs mechanical degrees
                        electrical_angle = (self._sector * 60.0)  # 60° per sector
                        self._position_mechanical = electrical_angle / self.pole_pairs
                        
                except ValueError:
                    # Invalid Hall state - might be during transition
                    pass
                    
            # Update last states
            self._last_hall_states = self._hall_states.copy()
            
    def read_hall_states(self) -> dict:
        """
        Read current Hall sensor states
        
        Returns:
            Dictionary with Hall A, B, C states
        """
        with self._measurement_lock:
            return self._hall_states.copy()
            
    def read_sector(self) -> int:
        """
        Read current electrical sector (0-5)
        
        Returns:
            Current sector number
        """
        with self._measurement_lock:
            return self._sector
            
    def read_electrical_angle(self) -> float:
        """
        Read electrical angle in degrees
        
        Returns:
            Electrical angle (0-360°)
        """
        with self._measurement_lock:
            return self._sector * 60.0  # 60° per sector
            
    def read_mechanical_angle(self) -> float:
        """
        Read mechanical angle in degrees
        
        Returns:
            Mechanical angle considering pole pairs
        """
        with self._measurement_lock:
            return self._position_mechanical
            
    def read_direction(self) -> int:
        """
        Read rotation direction
        
        Returns:
            1 for forward, -1 for reverse
        """
        with self._measurement_lock:
            return self._direction
            
    def get_hsm_info(self) -> dict:
        """
        Get comprehensive HSM information
        
        Returns:
            Dictionary with HSM measurements
        """
        with self._measurement_lock:
            hall_state_value = (self._hall_states['C'] << 2) | (self._hall_states['B'] << 1) | self._hall_states['A']
            
            return {
                'hall_states': self._hall_states.copy(),
                'hall_state_binary': f"{hall_state_value:03b}",
                'hall_state_value': hall_state_value,
                'sector': self._sector,
                'electrical_angle_degrees': self._sector * 60.0,
                'mechanical_angle_degrees': self._position_mechanical,
                'direction': self._direction,
                'direction_text': "CW" if self._direction > 0 else "CCW",
                'pole_pairs': self.pole_pairs,
                'hysteresis_degrees': self._hysteresis_values[self.hysteresis]
            }
            
    def calibrate_hall_sequence(self, samples: int = 60) -> dict:
        """
        Calibrate Hall sensor sequence by observing transitions
        
        Args:
            samples: Number of samples to collect
            
        Returns:
            Calibration data with observed sequence
        """
        print(f"Calibrating HSM Hall sequence with {samples} samples...")
        print("Please rotate the magnet slowly during calibration...")
        
        observed_states = []
        transitions = []
        
        last_state = None
        
        for i in range(samples):
            info = self.get_hsm_info()
            current_state = info['hall_state_value']
            
            if current_state != last_state and last_state is not None:
                transitions.append((last_state, current_state))
                
            if current_state not in observed_states:
                observed_states.append(current_state)
                
            last_state = current_state
            time.sleep(0.1)
            
        # Analyze transitions
        unique_states = sorted(set(observed_states))
        
        calibration_data = {
            'observed_states': observed_states,
            'unique_states': unique_states,
            'unique_states_binary': [f"{state:03b}" for state in unique_states],
            'transitions': transitions,
            'expected_states': self._hall_sequence_forward,
            'expected_states_binary': [f"{state:03b}" for state in self._hall_sequence_forward],
            'valid_sequence': len(unique_states) == 6
        }
        
        print("HSM Calibration Results:")
        print(f"  Observed states: {unique_states}")
        print(f"  Expected states: {self._hall_sequence_forward}")
        print(f"  Valid 6-state sequence: {calibration_data['valid_sequence']}")
        
        return calibration_data

def main():
    """
    Example usage of TLx5012 HSM interface
    """
    print("TLx5012 HSM Interface - Raspberry Pi 5 Example")
    print("=" * 50)
    
    try:
        # Initialize HSM interface
        with TLx5012HSM(spi_bus=0, spi_device=0,
                       ifa_pin=17, ifb_pin=18, ifc_pin=19,
                       pole_pairs=1, 
                       hysteresis=TLx5012HSM.HSM_HYSTERESIS_0_35_DEG) as hsm_sensor:
            
            print("HSM interface initialized successfully!")
            
            # Wait for signals to stabilize
            print("Waiting for Hall signals to stabilize...")
            time.sleep(1)
            
            # Calibrate Hall sequence
            try:
                calibration = hsm_sensor.calibrate_hall_sequence(30)
            except TLx5012HSMError as e:
                print(f"Calibration failed: {e}")
                print("Continuing with uncalibrated measurements...")
            
            # Continuous reading
            print("\nReading HSM Hall sensor data (Ctrl+C to stop):")
            print("Time\t\tHall(ABC)\tSector\tElec(°)\tMech(°)\tDir")
            print("-" * 70)
            
            while True:
                try:
                    info = hsm_sensor.get_hsm_info()
                    
                    timestamp = time.strftime("%H:%M:%S")
                    hall_binary = info['hall_state_binary']
                    sector = info['sector']
                    elec_angle = info['electrical_angle_degrees']
                    mech_angle = info['mechanical_angle_degrees']
                    direction = info['direction_text']
                    
                    print(f"{timestamp}\t{hall_binary}\t\t{sector}\t{elec_angle:6.1f}\t{mech_angle:6.1f}\t{direction}")
                    
                    time.sleep(0.1)
                    
                except KeyboardInterrupt:
                    print("\nStopping...")
                    break
                except Exception as e:
                    print(f"Measurement error: {e}")
                    time.sleep(1)
                    
    except TLx5012HSMError as e:
        print(f"HSM interface error: {e}")
        print("\nTroubleshooting:")
        print("1. Check SPI connection for sensor configuration")
        print("2. Check IFA/IFB/IFC pin connections")
        print("3. Verify sensor is configured for HSM output")
        print("4. Check magnet placement and pole pair setting")

if __name__ == "__main__":
    main()

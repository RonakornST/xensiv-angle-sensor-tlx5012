#!/usr/bin/env python3
"""
TLx5012 All Interfaces Test Script for Raspberry Pi 5
Tests and demonstrates all available interfaces

Author: AI Assistant
License: MIT
"""

import time
import sys
from typing import Dict, Any, Optional

# Import all interface implementations
from tlx5012_multi_interface import TLx5012MultiInterface, InterfaceType, TLx5012MultiInterfaceError
from tlx5012_raspi import TLx5012, TLx5012Error
from tlx5012_pwm_raspi import TLx5012PWM, TLx5012PWMError
from tlx5012_iif_raspi import TLx5012IIF, TLx5012IIFError
from tlx5012_hsm_raspi import TLx5012HSM, TLx5012HSMError
from tlx5012_spc_raspi import TLx5012SPC, TLx5012SPCError

class TLx5012InterfaceTester:
    """Comprehensive tester for all TLx5012 interfaces"""
    
    def __init__(self, spi_bus: int = 0, spi_device: int = 0):
        self.spi_bus = spi_bus
        self.spi_device = spi_device
        self.test_results = {}
        
    def print_header(self, title: str):
        """Print formatted test section header"""
        print("\n" + "=" * 60)
        print(f" {title}")
        print("=" * 60)
        
    def print_subheader(self, title: str):
        """Print formatted test subsection header"""
        print(f"\n--- {title} ---")
        
    def test_spi_interface(self) -> Dict[str, Any]:
        """Test SPI interface functionality"""
        self.print_subheader("Testing SPI Interface")
        
        result = {
            'interface': 'SPI',
            'connection': False,
            'angle_reading': False,
            'temperature_reading': False,
            'revolution_reading': False,
            'error': None
        }
        
        try:
            with TLx5012(self.spi_bus, self.spi_device) as sensor:
                print("✓ SPI connection established")
                result['connection'] = True
                
                # Test angle reading
                angle = sensor.read_angle_degrees()
                if angle is not None:
                    print(f"✓ Angle reading: {angle:.3f}°")
                    result['angle_reading'] = True
                else:
                    print("✗ Angle reading failed")
                    
                # Test temperature reading
                temp = sensor.read_temperature()
                if temp is not None:
                    print(f"✓ Temperature reading: {temp:.1f}°C")
                    result['temperature_reading'] = True
                else:
                    print("✗ Temperature reading failed")
                    
                # Test revolution reading
                rev = sensor.read_revolutions()
                if rev is not None:
                    print(f"✓ Revolution reading: {rev}")
                    result['revolution_reading'] = True
                else:
                    print("✗ Revolution reading failed")
                    
        except TLx5012Error as e:
            print(f"✗ SPI interface error: {e}")
            result['error'] = str(e)
        except Exception as e:
            print(f"✗ Unexpected error: {e}")
            result['error'] = str(e)
            
        return result
        
    def test_pwm_interface(self) -> Dict[str, Any]:
        """Test PWM interface functionality"""
        self.print_subheader("Testing PWM Interface")
        
        result = {
            'interface': 'PWM',
            'connection': False,
            'configuration': False,
            'pwm_measurement': False,
            'angle_reading': False,
            'error': None
        }
        
        try:
            with TLx5012PWM(self.spi_bus, self.spi_device, pwm_pin=18) as sensor:
                print("✓ PWM interface opened")
                result['connection'] = True
                result['configuration'] = True
                
                # Wait for PWM signal to stabilize
                time.sleep(1)
                
                # Test PWM measurement
                duty_cycle = sensor.read_duty_cycle()
                if duty_cycle > 0:
                    print(f"✓ PWM duty cycle: {duty_cycle:.2f}%")
                    result['pwm_measurement'] = True
                else:
                    print("✗ No PWM signal detected")
                    
                # Test angle reading
                angle = sensor.read_angle_degrees()
                if angle is not None:
                    print(f"✓ PWM angle reading: {angle:.3f}°")
                    result['angle_reading'] = True
                else:
                    print("✗ PWM angle reading failed")
                    
        except TLx5012PWMError as e:
            print(f"✗ PWM interface error: {e}")
            result['error'] = str(e)
        except Exception as e:
            print(f"✗ Unexpected error: {e}")
            result['error'] = str(e)
            
        return result
        
    def test_iif_interface(self) -> Dict[str, Any]:
        """Test IIF interface functionality"""
        self.print_subheader("Testing IIF Interface")
        
        result = {
            'interface': 'IIF',
            'connection': False,
            'configuration': False,
            'encoder_signals': False,
            'position_tracking': False,
            'error': None
        }
        
        try:
            with TLx5012IIF(self.spi_bus, self.spi_device, ifa_pin=17, ifb_pin=18) as sensor:
                print("✓ IIF interface opened")
                result['connection'] = True
                result['configuration'] = True
                
                # Wait for encoder signals
                time.sleep(1)
                
                # Test encoder position
                initial_position = sensor.read_position_raw()
                print(f"✓ Initial encoder position: {initial_position} pulses")
                result['encoder_signals'] = True
                
                # Test angle reading
                angle = sensor.read_angle_degrees()
                print(f"✓ IIF angle reading: {angle:.3f}°")
                result['position_tracking'] = True
                
        except TLx5012IIFError as e:
            print(f"✗ IIF interface error: {e}")
            result['error'] = str(e)
        except Exception as e:
            print(f"✗ Unexpected error: {e}")
            result['error'] = str(e)
            
        return result
        
    def test_hsm_interface(self) -> Dict[str, Any]:
        """Test HSM interface functionality"""
        self.print_subheader("Testing HSM Interface")
        
        result = {
            'interface': 'HSM',
            'connection': False,
            'configuration': False,
            'hall_signals': False,
            'sector_detection': False,
            'error': None
        }
        
        try:
            with TLx5012HSM(self.spi_bus, self.spi_device, 
                           ifa_pin=17, ifb_pin=18, ifc_pin=19) as sensor:
                print("✓ HSM interface opened")
                result['connection'] = True
                result['configuration'] = True
                
                # Wait for Hall signals
                time.sleep(1)
                
                # Test Hall states
                hall_states = sensor.read_hall_states()
                print(f"✓ Hall states - A:{hall_states['A']} B:{hall_states['B']} C:{hall_states['C']}")
                result['hall_signals'] = True
                
                # Test sector detection
                sector = sensor.read_sector()
                angle = sensor.read_mechanical_angle()
                print(f"✓ HSM sector: {sector}, angle: {angle:.1f}°")
                result['sector_detection'] = True
                
        except TLx5012HSMError as e:
            print(f"✗ HSM interface error: {e}")
            result['error'] = str(e)
        except Exception as e:
            print(f"✗ Unexpected error: {e}")
            result['error'] = str(e)
            
        return result
        
    def test_spc_interface(self) -> Dict[str, Any]:
        """Test SPC interface functionality"""
        self.print_subheader("Testing SPC Interface")
        
        result = {
            'interface': 'SPC',
            'connection': False,
            'configuration': False,
            'trigger_generation': False,
            'data_decoding': False,
            'error': None
        }
        
        try:
            with TLx5012SPC(self.spi_bus, self.spi_device, ifa_pin=17) as sensor:
                print("✓ SPC interface opened")
                result['connection'] = True
                result['configuration'] = True
                
                # Test trigger and data reading
                angle, temperature = sensor.trigger_and_read()
                result['trigger_generation'] = True
                
                if angle is not None:
                    print(f"✓ SPC angle reading: {angle:.3f}°")
                    result['data_decoding'] = True
                else:
                    print("✗ SPC data decoding failed")
                    
                if temperature is not None:
                    print(f"✓ SPC temperature reading: {temperature:.1f}°C")
                    
        except TLx5012SPCError as e:
            print(f"✗ SPC interface error: {e}")
            result['error'] = str(e)
        except Exception as e:
            print(f"✗ Unexpected error: {e}")
            result['error'] = str(e)
            
        return result
        
    def test_multi_interface(self) -> Dict[str, Any]:
        """Test multi-interface functionality"""
        self.print_subheader("Testing Multi-Interface Library")
        
        result = {
            'interface': 'Multi',
            'detection': False,
            'auto_open': False,
            'unified_reading': False,
            'interface_switching': False,
            'detected_interface': None,
            'error': None
        }
        
        try:
            with TLx5012MultiInterface(self.spi_bus, self.spi_device) as multi_sensor:
                
                # Test interface detection
                detected = multi_sensor.detect_interface()
                if detected:
                    print(f"✓ Interface detection: {detected.value}")
                    result['detection'] = True
                    result['detected_interface'] = detected.value
                else:
                    print("✗ Interface detection failed")
                    
                # Test auto-open
                interface = multi_sensor.auto_open_interface()
                if interface:
                    print("✓ Auto-open interface successful")
                    result['auto_open'] = True
                    
                    # Test unified reading
                    angle = multi_sensor.read_angle_degrees()
                    if angle is not None:
                        print(f"✓ Unified angle reading: {angle:.3f}°")
                        result['unified_reading'] = True
                    else:
                        print("✗ Unified angle reading failed")
                        
        except TLx5012MultiInterfaceError as e:
            print(f"✗ Multi-interface error: {e}")
            result['error'] = str(e)
        except Exception as e:
            print(f"✗ Unexpected error: {e}")
            result['error'] = str(e)
            
        return result
        
    def run_all_tests(self) -> Dict[str, Dict[str, Any]]:
        """Run all interface tests"""
        self.print_header("TLx5012 Interface Comprehensive Test Suite")
        
        print("Testing all available interfaces for TLx5012 sensor...")
        print("Note: Some tests may fail if hardware is not properly configured")
        
        # Run individual interface tests
        self.test_results['spi'] = self.test_spi_interface()
        self.test_results['pwm'] = self.test_pwm_interface()
        self.test_results['iif'] = self.test_iif_interface()
        self.test_results['hsm'] = self.test_hsm_interface()
        self.test_results['spc'] = self.test_spc_interface()
        self.test_results['multi'] = self.test_multi_interface()
        
        return self.test_results
        
    def print_summary(self):
        """Print test results summary"""
        self.print_header("Test Results Summary")
        
        total_tests = 0
        passed_tests = 0
        
        for interface, results in self.test_results.items():
            print(f"\n{interface.upper()} Interface:")
            
            for test_name, test_result in results.items():
                if test_name in ['interface', 'detected_interface', 'error']:
                    continue
                    
                total_tests += 1
                status = "✓ PASS" if test_result else "✗ FAIL"
                if test_result:
                    passed_tests += 1
                    
                print(f"  {test_name}: {status}")
                
            if results.get('error'):
                print(f"  Error: {results['error']}")
                
        print(f"\nOverall Results: {passed_tests}/{total_tests} tests passed")
        
        # Recommendations
        print("\nRecommendations:")
        if self.test_results['spi']['connection']:
            print("• SPI interface is working - use for basic sensor access")
        if self.test_results['multi']['detection']:
            detected = self.test_results['multi']['detected_interface']
            print(f"• Sensor is configured for {detected} interface")
        if not any(r['connection'] for r in self.test_results.values() if 'connection' in r):
            print("• Check hardware connections and power supply")
            print("• Verify SPI is enabled: sudo raspi-config -> Interface Options -> SPI")
            print("• Check GPIO permissions: sudo usermod -a -G gpio $USER")

def main():
    """Main test execution"""
    if len(sys.argv) > 1 and sys.argv[1] == '--help':
        print("TLx5012 Interface Test Suite")
        print("Usage: python3 test_all_interfaces.py")
        print("\nThis script tests all available TLx5012 interfaces:")
        print("- SPI: Direct register access")
        print("- PWM: Pulse width modulation output")
        print("- IIF: Incremental interface (quadrature)")
        print("- HSM: Hall switch mode")
        print("- SPC: Short-PWM-Code (trigger-based)")
        print("- Multi: Unified interface library")
        return
        
    try:
        tester = TLx5012InterfaceTester(spi_bus=0, spi_device=0)
        tester.run_all_tests()
        tester.print_summary()
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Test suite error: {e}")

if __name__ == "__main__":
    main()

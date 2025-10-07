#!/usr/bin/env python3
"""
TLx5012 Sensor Test Script for Raspberry Pi 5
Tests basic connectivity and functionality
"""

import time
import sys
from tlx5012_raspi import TLx5012, TLx5012Error

def test_spi_connection():
    """Test basic SPI connectivity"""
    print("Testing SPI connection...")
    try:
        import spidev
        spi = spidev.SpiDev()
        spi.open(0, 0)
        spi.close()
        print("✓ SPI interface available")
        return True
    except Exception as e:
        print(f"✗ SPI interface error: {e}")
        return False

def test_sensor_communication():
    """Test sensor communication"""
    print("\nTesting sensor communication...")
    try:
        with TLx5012(spi_bus=0, spi_device=0, max_speed_hz=500000) as sensor:
            # Try to read status register
            status = sensor.read_status()
            print("✓ Sensor communication successful")
            print(f"  Status: {status}")
            return True, sensor
    except TLx5012Error as e:
        print(f"✗ Sensor communication failed: {e}")
        return False, None
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        return False, None

def test_angle_reading(sensor):
    """Test angle reading functionality"""
    print("\nTesting angle reading...")
    try:
        # Read angle multiple times to check consistency
        angles = []
        for i in range(5):
            angle = sensor.read_angle_degrees()
            angles.append(angle)
            print(f"  Reading {i+1}: {angle:.3f}°")
            time.sleep(0.1)
        
        # Check if readings are reasonable
        angle_range = max(angles) - min(angles)
        if angle_range < 360:  # Reasonable range
            print("✓ Angle readings appear valid")
            return True
        else:
            print("⚠ Angle readings may be unstable")
            return False
            
    except Exception as e:
        print(f"✗ Angle reading failed: {e}")
        return False

def test_temperature_reading(sensor):
    """Test temperature reading"""
    print("\nTesting temperature reading...")
    try:
        temp = sensor.read_temperature()
        print(f"  Temperature: {temp:.2f}°C")
        
        # Check if temperature is reasonable (room temperature range)
        if -10 <= temp <= 60:
            print("✓ Temperature reading appears valid")
            return True
        else:
            print("⚠ Temperature reading may be incorrect")
            return False
            
    except Exception as e:
        print(f"✗ Temperature reading failed: {e}")
        return False

def test_revolution_counter(sensor):
    """Test revolution counter"""
    print("\nTesting revolution counter...")
    try:
        rev_count = sensor.read_revolution_count()
        print(f"  Revolution count: {rev_count}")
        print("✓ Revolution counter reading successful")
        return True
    except Exception as e:
        print(f"✗ Revolution counter reading failed: {e}")
        return False

def interactive_test(sensor):
    """Interactive test mode"""
    print("\n" + "="*50)
    print("INTERACTIVE TEST MODE")
    print("Move the magnet and observe angle changes")
    print("Press Ctrl+C to stop")
    print("="*50)
    
    try:
        last_angle = None
        movement_detected = False
        
        while True:
            # Read all sensor data
            data = sensor.read_all_data()
            
            angle = data['angle_degrees']
            temp = data['temperature']
            revs = data['revolution_count']
            
            # Detect movement
            if last_angle is not None:
                angle_diff = abs(angle - last_angle)
                if angle_diff > 1.0:  # More than 1 degree change
                    movement_detected = True
                    
            # Display data
            status_char = "🔄" if movement_detected else "📍"
            print(f"\r{status_char} Angle: {angle:7.3f}° | Temp: {temp:5.1f}°C | Rev: {revs:3d} ", end="")
            
            last_angle = angle
            movement_detected = False
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n\nInteractive test completed.")

def main():
    """Main test function"""
    print("TLx5012 Magnetic Angle Sensor Test")
    print("=" * 40)
    
    # Test 1: SPI Connection
    if not test_spi_connection():
        print("\nFailed: SPI interface not available")
        print("Solution: Enable SPI with 'sudo raspi-config'")
        sys.exit(1)
    
    # Test 2: Sensor Communication
    comm_ok, sensor = test_sensor_communication()
    if not comm_ok:
        print("\nFailed: Cannot communicate with sensor")
        print("Check wiring:")
        print("  VCC -> 3.3V")
        print("  GND -> GND") 
        print("  SCK -> GPIO 11")
        print("  MOSI -> GPIO 10")
        print("  CS -> GPIO 8")
        sys.exit(1)
    
    # Test 3: Angle Reading
    with sensor:
        angle_ok = test_angle_reading(sensor)
        
        # Test 4: Temperature Reading
        temp_ok = test_temperature_reading(sensor)
        
        # Test 5: Revolution Counter
        rev_ok = test_revolution_counter(sensor)
        
        # Summary
        print("\n" + "="*40)
        print("TEST SUMMARY")
        print("="*40)
        print(f"SPI Connection:      ✓")
        print(f"Sensor Communication: ✓")
        print(f"Angle Reading:       {'✓' if angle_ok else '⚠'}")
        print(f"Temperature Reading: {'✓' if temp_ok else '⚠'}")
        print(f"Revolution Counter:  {'✓' if rev_ok else '⚠'}")
        
        if angle_ok and temp_ok and rev_ok:
            print("\n🎉 All tests passed! Sensor is working correctly.")
            
            # Ask for interactive test
            try:
                response = input("\nRun interactive test? (y/n): ").lower()
                if response in ['y', 'yes']:
                    interactive_test(sensor)
            except KeyboardInterrupt:
                pass
        else:
            print("\n⚠ Some tests failed. Check connections and magnet placement.")

if __name__ == "__main__":
    main()

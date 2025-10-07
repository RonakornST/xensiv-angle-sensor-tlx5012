#!/usr/bin/env python3
"""
TLx5012 Multi-Interface Library for Raspberry Pi 5
Unified interface supporting SPI, PWM, IIF, HSM, and SPC modes

Author: AI Assistant
License: MIT
"""

import time
from typing import Optional, Dict, Any, Union
from enum import Enum

# Import all interface implementations
from tlx5012_raspi import TLx5012, TLx5012Error, RegisterAddress
from tlx5012_pwm_raspi import TLx5012PWM, TLx5012PWMError
from tlx5012_iif_raspi import TLx5012IIF, TLx5012IIFError
from tlx5012_hsm_raspi import TLx5012HSM, TLx5012HSMError
from tlx5012_spc_raspi import TLx5012SPC, TLx5012SPCError

class InterfaceType(Enum):
    """Enumeration of available interface types"""
    SPI = "spi"
    PWM = "pwm"
    IIF = "iif"
    HSM = "hsm"
    SPC = "spc"

class TLx5012MultiInterfaceError(Exception):
    """Custom exception for multi-interface errors"""
    pass

class TLx5012MultiInterface:
    """
    TLx5012 Multi-Interface Controller for Raspberry Pi
    
    This class provides a unified interface to access the TLx5012 sensor
    using any of the available communication interfaces.
    """
    
    def __init__(self, spi_bus: int = 0, spi_device: int = 0):
        """
        Initialize multi-interface controller
        
        Args:
            spi_bus: SPI bus number
            spi_device: SPI device number
        """
        self.spi_bus = spi_bus
        self.spi_device = spi_device
        
        # Interface instances
        self._interfaces = {}
        self._current_interface = None
        self._current_interface_type = None
        
        # Default GPIO pin assignments
        self._default_pins = {
            'pwm_pin': 18,
            'ifa_pin': 17,
            'ifb_pin': 18,
            'ifc_pin': 19
        }
        
    def __enter__(self):
        """Context manager entry"""
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.close_all_interfaces()
        
    def detect_interface(self) -> Optional[InterfaceType]:
        """
        Detect the currently configured interface type
        
        Returns:
            Detected interface type or None if detection fails
        """
        try:
            # Use SPI to read interface configuration
            spi_sensor = TLx5012(self.spi_bus, self.spi_device)
            spi_sensor.open()
            
            try:
                # Read MOD_4 register to get interface mode
                mod4_data = spi_sensor._read_register(RegisterAddress.REG_MOD_4)
                interface_mode = mod4_data & 0x0003  # Bits 1:0
                
                # Map interface mode to interface type
                interface_map = {
                    0: InterfaceType.IIF,
                    1: InterfaceType.PWM,
                    2: InterfaceType.HSM,
                    3: InterfaceType.SPC
                }
                
                detected_type = interface_map.get(interface_mode)
                print(f"Detected interface: {detected_type.value if detected_type else 'unknown'} (mode: {interface_mode})")
                
                return detected_type
                
            finally:
                spi_sensor.close()
                
        except Exception as e:
            print(f"Interface detection failed: {e}")
            return None
            
    def open_spi_interface(self) -> TLx5012:
        """
        Open SPI interface
        
        Returns:
            SPI interface instance
        """
        if InterfaceType.SPI in self._interfaces:
            return self._interfaces[InterfaceType.SPI]
            
        try:
            spi_interface = TLx5012(self.spi_bus, self.spi_device)
            spi_interface.open()
            
            self._interfaces[InterfaceType.SPI] = spi_interface
            self._current_interface = spi_interface
            self._current_interface_type = InterfaceType.SPI
            
            print("SPI interface opened successfully")
            return spi_interface
            
        except TLx5012Error as e:
            raise TLx5012MultiInterfaceError(f"Failed to open SPI interface: {e}")
            
    def open_pwm_interface(self, pwm_pin: Optional[int] = None, 
                          frequency: int = TLx5012PWM.PWM_FREQ_977HZ) -> TLx5012PWM:
        """
        Open PWM interface
        
        Args:
            pwm_pin: GPIO pin for PWM input
            frequency: PWM frequency setting
            
        Returns:
            PWM interface instance
        """
        if InterfaceType.PWM in self._interfaces:
            return self._interfaces[InterfaceType.PWM]
            
        pin = pwm_pin or self._default_pins['pwm_pin']
        
        try:
            pwm_interface = TLx5012PWM(self.spi_bus, self.spi_device, pin, frequency)
            pwm_interface.open()
            
            self._interfaces[InterfaceType.PWM] = pwm_interface
            self._current_interface = pwm_interface
            self._current_interface_type = InterfaceType.PWM
            
            print("PWM interface opened successfully")
            return pwm_interface
            
        except TLx5012PWMError as e:
            raise TLx5012MultiInterfaceError(f"Failed to open PWM interface: {e}")
            
    def open_iif_interface(self, ifa_pin: Optional[int] = None, 
                          ifb_pin: Optional[int] = None,
                          resolution: int = TLx5012IIF.IIF_RES_12BIT) -> TLx5012IIF:
        """
        Open IIF interface
        
        Args:
            ifa_pin: GPIO pin for IFA signal
            ifb_pin: GPIO pin for IFB signal
            resolution: IIF resolution setting
            
        Returns:
            IIF interface instance
        """
        if InterfaceType.IIF in self._interfaces:
            return self._interfaces[InterfaceType.IIF]
            
        ifa = ifa_pin or self._default_pins['ifa_pin']
        ifb = ifb_pin or self._default_pins['ifb_pin']
        
        try:
            iif_interface = TLx5012IIF(self.spi_bus, self.spi_device, ifa, ifb, resolution)
            iif_interface.open()
            
            self._interfaces[InterfaceType.IIF] = iif_interface
            self._current_interface = iif_interface
            self._current_interface_type = InterfaceType.IIF
            
            print("IIF interface opened successfully")
            return iif_interface
            
        except TLx5012IIFError as e:
            raise TLx5012MultiInterfaceError(f"Failed to open IIF interface: {e}")
            
    def open_hsm_interface(self, ifa_pin: Optional[int] = None,
                          ifb_pin: Optional[int] = None,
                          ifc_pin: Optional[int] = None,
                          pole_pairs: int = 1) -> TLx5012HSM:
        """
        Open HSM interface
        
        Args:
            ifa_pin: GPIO pin for IFA signal
            ifb_pin: GPIO pin for IFB signal
            ifc_pin: GPIO pin for IFC signal
            pole_pairs: Number of pole pairs
            
        Returns:
            HSM interface instance
        """
        if InterfaceType.HSM in self._interfaces:
            return self._interfaces[InterfaceType.HSM]
            
        ifa = ifa_pin or self._default_pins['ifa_pin']
        ifb = ifb_pin or self._default_pins['ifb_pin']
        ifc = ifc_pin or self._default_pins['ifc_pin']
        
        try:
            hsm_interface = TLx5012HSM(self.spi_bus, self.spi_device, ifa, ifb, ifc, pole_pairs)
            hsm_interface.open()
            
            self._interfaces[InterfaceType.HSM] = hsm_interface
            self._current_interface = hsm_interface
            self._current_interface_type = InterfaceType.HSM
            
            print("HSM interface opened successfully")
            return hsm_interface
            
        except TLx5012HSMError as e:
            raise TLx5012MultiInterfaceError(f"Failed to open HSM interface: {e}")
            
    def open_spc_interface(self, ifa_pin: Optional[int] = None,
                          frame_config: int = TLx5012SPC.SPC_FRAME_16BIT_ANGLE) -> TLx5012SPC:
        """
        Open SPC interface
        
        Args:
            ifa_pin: GPIO pin for IFA signal
            frame_config: SPC frame configuration
            
        Returns:
            SPC interface instance
        """
        if InterfaceType.SPC in self._interfaces:
            return self._interfaces[InterfaceType.SPC]
            
        ifa = ifa_pin or self._default_pins['ifa_pin']
        
        try:
            spc_interface = TLx5012SPC(self.spi_bus, self.spi_device, ifa, 
                                     frame_config=frame_config)
            spc_interface.open()
            
            self._interfaces[InterfaceType.SPC] = spc_interface
            self._current_interface = spc_interface
            self._current_interface_type = InterfaceType.SPC
            
            print("SPC interface opened successfully")
            return spc_interface
            
        except TLx5012SPCError as e:
            raise TLx5012MultiInterfaceError(f"Failed to open SPC interface: {e}")
            
    def auto_open_interface(self, **kwargs) -> Any:
        """
        Automatically detect and open the appropriate interface
        
        Returns:
            Opened interface instance
        """
        detected_type = self.detect_interface()
        
        if detected_type is None:
            # Default to SPI if detection fails
            print("Interface detection failed, defaulting to SPI")
            return self.open_spi_interface()
            
        # Open the detected interface
        if detected_type == InterfaceType.SPI:
            return self.open_spi_interface()
        elif detected_type == InterfaceType.PWM:
            return self.open_pwm_interface(**kwargs)
        elif detected_type == InterfaceType.IIF:
            return self.open_iif_interface(**kwargs)
        elif detected_type == InterfaceType.HSM:
            return self.open_hsm_interface(**kwargs)
        elif detected_type == InterfaceType.SPC:
            return self.open_spc_interface(**kwargs)
        else:
            raise TLx5012MultiInterfaceError(f"Unsupported interface type: {detected_type}")
            
    def read_angle_degrees(self) -> Optional[float]:
        """
        Read angle in degrees using current interface
        
        Returns:
            Angle in degrees or None if no interface is open
        """
        if self._current_interface is None:
            raise TLx5012MultiInterfaceError("No interface is currently open")
            
        try:
            return self._current_interface.read_angle_degrees()
        except Exception as e:
            raise TLx5012MultiInterfaceError(f"Failed to read angle: {e}")
            
    def read_angle_radians(self) -> Optional[float]:
        """
        Read angle in radians using current interface
        
        Returns:
            Angle in radians or None if no interface is open
        """
        if self._current_interface is None:
            raise TLx5012MultiInterfaceError("No interface is currently open")
            
        try:
            if hasattr(self._current_interface, 'read_angle_radians'):
                return self._current_interface.read_angle_radians()
            else:
                # Convert from degrees if radians method not available
                degrees = self._current_interface.read_angle_degrees()
                if degrees is not None:
                    import math
                    return math.radians(degrees)
                return None
        except Exception as e:
            raise TLx5012MultiInterfaceError(f"Failed to read angle: {e}")
            
    def get_interface_info(self) -> Dict[str, Any]:
        """
        Get information about current interface
        
        Returns:
            Dictionary with interface information
        """
        if self._current_interface is None:
            return {'interface_type': None, 'interface_open': False}
            
        info = {
            'interface_type': self._current_interface_type.value,
            'interface_open': True,
            'available_interfaces': [itype.value for itype in self._interfaces.keys()]
        }
        
        # Get interface-specific information
        try:
            if hasattr(self._current_interface, 'get_pwm_info'):
                info.update(self._current_interface.get_pwm_info())
            elif hasattr(self._current_interface, 'get_iif_info'):
                info.update(self._current_interface.get_iif_info())
            elif hasattr(self._current_interface, 'get_hsm_info'):
                info.update(self._current_interface.get_hsm_info())
            elif hasattr(self._current_interface, 'get_spc_info'):
                info.update(self._current_interface.get_spc_info())
        except Exception as e:
            info['info_error'] = str(e)
            
        return info
        
    def close_interface(self, interface_type: InterfaceType):
        """
        Close specific interface
        
        Args:
            interface_type: Type of interface to close
        """
        if interface_type in self._interfaces:
            try:
                self._interfaces[interface_type].close()
                del self._interfaces[interface_type]
                
                if self._current_interface_type == interface_type:
                    self._current_interface = None
                    self._current_interface_type = None
                    
                print(f"{interface_type.value.upper()} interface closed")
            except Exception as e:
                print(f"Error closing {interface_type.value} interface: {e}")
                
    def close_all_interfaces(self):
        """Close all open interfaces"""
        for interface_type in list(self._interfaces.keys()):
            self.close_interface(interface_type)
            
    def switch_interface(self, interface_type: InterfaceType, **kwargs) -> Any:
        """
        Switch to a different interface
        
        Args:
            interface_type: Target interface type
            **kwargs: Interface-specific parameters
            
        Returns:
            New interface instance
        """
        # Close current interface if different
        if (self._current_interface_type is not None and 
            self._current_interface_type != interface_type):
            self.close_interface(self._current_interface_type)
            
        # Open new interface
        if interface_type == InterfaceType.SPI:
            return self.open_spi_interface()
        elif interface_type == InterfaceType.PWM:
            return self.open_pwm_interface(**kwargs)
        elif interface_type == InterfaceType.IIF:
            return self.open_iif_interface(**kwargs)
        elif interface_type == InterfaceType.HSM:
            return self.open_hsm_interface(**kwargs)
        elif interface_type == InterfaceType.SPC:
            return self.open_spc_interface(**kwargs)
        else:
            raise TLx5012MultiInterfaceError(f"Unsupported interface type: {interface_type}")

def main():
    """
    Example usage of TLx5012 multi-interface library
    """
    print("TLx5012 Multi-Interface Library - Raspberry Pi 5 Example")
    print("=" * 60)
    
    try:
        with TLx5012MultiInterface(spi_bus=0, spi_device=0) as multi_sensor:
            
            # Auto-detect and open interface
            print("Auto-detecting interface...")
            interface = multi_sensor.auto_open_interface()
            
            # Get interface information
            info = multi_sensor.get_interface_info()
            print(f"Active interface: {info['interface_type']}")
            
            # Continuous reading
            print("\nReading sensor data (Ctrl+C to stop):")
            print("Time\t\tAngle (°)\tInterface")
            print("-" * 50)
            
            while True:
                try:
                    angle = multi_sensor.read_angle_degrees()
                    timestamp = time.strftime("%H:%M:%S")
                    interface_type = info['interface_type']
                    
                    angle_str = f"{angle:8.3f}" if angle is not None else "   N/A  "
                    print(f"{timestamp}\t{angle_str}\t{interface_type}")
                    
                    time.sleep(0.1)
                    
                except KeyboardInterrupt:
                    print("\nStopping...")
                    break
                except Exception as e:
                    print(f"Measurement error: {e}")
                    time.sleep(1)
                    
    except TLx5012MultiInterfaceError as e:
        print(f"Multi-interface error: {e}")
        print("\nTroubleshooting:")
        print("1. Check all connections (SPI + GPIO pins)")
        print("2. Verify sensor power and ground connections")
        print("3. Check interface configuration in sensor")

if __name__ == "__main__":
    main()

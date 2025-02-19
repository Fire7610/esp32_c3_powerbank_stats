# esp32_c3_powerbank_stats
Personal project using esp32_c3 to make an information display that would monitor stats like those expensive ones wahahahahaha


# ESP32-C3 Power Bank Monitor

This project is a smart power bank monitoring system built using an ESP32-C3, an INA226 power sensor (via the GyverINA library), and a 128×64 OLED display. The system continuously measures battery voltage, current, and power and displays these parameters in real-time. Additionally, it supports calibration by storing calibration data in EEPROM and allowing interactive adjustments via Serial commands, ensuring accurate measurements over time.

## Features

- **Real-Time Monitoring:**  
  Measures and displays battery voltage, current, and power consumption in real time.

- **Calibration & EEPROM Storage:**  
  - Uses a 0.005 Ω shunt resistor with the INA226 sensor for accurate current measurement.  
  - Calibration values for both current and voltage can be adjusted via serial commands and are stored in EEPROM to persist across power cycles.  
  - A linear interpolation function is used to correct raw voltage readings based on calibration points.

- **User Interface:**  
  - A 128×64 OLED (SH1106 based) displays key metrics in a two-column layout:
    - **Row 1:** Voltage (left) and Current (right)
    - **Row 2:** Power (left) and Status (e.g., "Discharge", "Charging", "Idle") (right)
  - A push-button (with internal pull-up on GPIO 4) toggles the display on and off to conserve power.

- **Interactive Serial Interface:**  
  - Serial commands such as `+a`, `-a`, `addv`, `resetv`, `save`, and `showv` allow interactive calibration adjustments and data viewing.
  - This makes it easy to fine-tune sensor performance without needing to reprogram the board.

- **Learning & Version Control:**  
  - This project provides an excellent opportunity to practice Git version control, repository management, and collaborative coding practices.

## Hardware Components

- **ESP32-C3 Super Mini**
- **INA226 Power Sensor** (using the GyverINA library)
- **128×64 OLED Display** (SH1106 based)
- Calibration shunt resistor (0.005 Ω)
- Push-button for toggling the display
- Battery pack and associated power circuitry

## Software Components

- **GyverINA Library** for INA226 sensor communication.
- **U8g2 Library** for OLED display control.
- **EEPROM Library** for storing calibration data.
- Custom calibration routines and serial command handling.

## Getting Started

1. **Hardware Setup:**  
   - Wire the INA226 sensor to the ESP32-C3:
     - **I2C:** SDA on GPIO 5, SCL on GPIO 6.
   - Connect the OLED display on the shared I2C bus.
   - Connect a push-button between GPIO 4 and GND (using the internal pull-up).
   - Ensure the shunt resistor is properly placed in the current path for accurate measurements.

2. **Software Setup:**  
   - Install the required Arduino libraries:
     - GyverINA (for INA226 sensor)
     - U8g2 (for OLED display)
     - EEPROM (usually included with Arduino)
   - Open the project in your Arduino IDE.
   - Compile and upload the code to your ESP32-C3.

3. **Calibration & Use:**  
   - Use the Serial Monitor to interact with the system.
   - Adjust calibration parameters via commands (e.g., `+a10`, `addv [voltage]`, etc.).
   - View real-time battery statistics on the OLED display.

## Future Enhancements

- Add more features for calibrating the voltage, amp, and wattage.
- Implement more advanced energy logging and battery health monitoring.
- Enhance the user interface with additional display pages or menus.
- Optimize power consumption further by implementing low-power modes when idle.

## Acknowledgments

- **GyverLibs** for the INA226 library.
- The Arduino community for continuous inspiration and support.

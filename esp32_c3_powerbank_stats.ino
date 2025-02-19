#include <Wire.h>
#include <GyverINA.h>
#include <U8g2lib.h>
#include <EEPROM.h>
#include <math.h>

// I2C pins for ESP32-C3
#define I2C_SDA 5
#define I2C_SCL 6

// Button pin (active LOW with internal pull-up)
#define BUTTON_PIN 4

// INA226 settings
const float SHUNT_OHMS = 0.005;
const float MAX_EXPECTED_AMPS = 7.0;

// EEPROM Addresses
#define EEPROM_ADDRESS_CURRENT 0
#define EEPROM_ADDRESS_VOLTAGE_POINTS 10  

// Max calibration points
#define MAX_CALIBRATION_POINTS 50  

// Calibration point struct
struct CalibrationPoint {
    float measuredVoltage;  
    float rawVoltage;       
};

// Create INA226 object
INA226 ina226(SHUNT_OHMS, MAX_EXPECTED_AMPS, 0x40);

// Create OLED display object
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

bool displayEnabled = true;
unsigned long lastButtonPress = 0;
const unsigned long debounceDelay = 200;

uint16_t currentCalibrationValue;
CalibrationPoint voltageCalibration[MAX_CALIBRATION_POINTS];  
int calibrationPointCount = 0;  

void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    Wire.begin(I2C_SDA, I2C_SCL);
    EEPROM.begin(1024);  // Increased storage size for large calibration points

    if (!ina226.begin()) {
        Serial.println("ERROR: INA226 not detected!");
        while (1);
    }

    // Load saved current calibration from EEPROM
    EEPROM.get(EEPROM_ADDRESS_CURRENT, currentCalibrationValue);
    if (currentCalibrationValue == 0xFFFF || currentCalibrationValue == 0) {
        currentCalibrationValue = ina226.getCalibration();
        EEPROM.put(EEPROM_ADDRESS_CURRENT, currentCalibrationValue);
        EEPROM.commit();
    } else {
        ina226.setCalibration(currentCalibrationValue);
    }

    // Load voltage correction points
    EEPROM.get(EEPROM_ADDRESS_VOLTAGE_POINTS, voltageCalibration);
    EEPROM.get(EEPROM_ADDRESS_VOLTAGE_POINTS + sizeof(voltageCalibration), calibrationPointCount);

    // Prevent invalid EEPROM data corruption
    if (calibrationPointCount > MAX_CALIBRATION_POINTS || calibrationPointCount < 0) {
        calibrationPointCount = 0;
    }

    Serial.printf("Loaded Calibration - Current: %d, Voltage Points: %d\n",
                  currentCalibrationValue, calibrationPointCount);

    ina226.setAveraging(INA226_AVG_X4);
    ina226.setSampleTime(INA226_VBUS, INA226_CONV_140US);
    ina226.setSampleTime(INA226_VSHUNT, INA226_CONV_140US);

    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_7x14_tf);
    u8g2.drawStr(0, 14, "Powerbank Ready");
    u8g2.sendBuffer();
    delay(2000);
}

void loop() {
    static bool buttonState = HIGH;
    bool currentState = digitalRead(BUTTON_PIN);

    if (currentState == LOW && buttonState == HIGH) {
        if (millis() - lastButtonPress > debounceDelay) {
            displayEnabled = !displayEnabled;
            lastButtonPress = millis();
            Serial.print("Display toggled: ");
            Serial.println(displayEnabled ? "ON" : "OFF");
        }
    }
    buttonState = currentState;

    float rawBusVoltage = ina226.getVoltage();
    float busVoltage = applyVoltageCorrection(rawBusVoltage);
    float current = ina226.getCurrent();
    float power = busVoltage * current;

    const char* status = "Idle";
    if (fabs(current) > 0.02) {
        status = (current > 0) ? "Discharge" : "Charging";
    }

    //Serial.printf("Voltage: %.2f V | Current: %.3f A | Power: %.2f W | Status: %s | Calib: %d\n",
    //              busVoltage, fabs(current), power, status, currentCalibrationValue);

    handleSerialInput();

    if (displayEnabled) {
        u8g2.setPowerSave(0);
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_7x14_tf);

        char buf[32];
        snprintf(buf, sizeof(buf), "V: %.2fV", busVoltage);
        u8g2.drawStr(0, 14, buf);

        snprintf(buf, sizeof(buf), "I: %.3fA", fabs(current));
        u8g2.drawStr(64, 14, buf);

        snprintf(buf, sizeof(buf), "P: %.2fW", power);
        u8g2.drawStr(0, 28, buf);

        u8g2.drawStr(64, 28, status);
        u8g2.sendBuffer();
    } else {
        u8g2.setPowerSave(1);
    }

    delay(1000);
}

// Apply correction using linear interpolation with extrapolation
float applyVoltageCorrection(float rawVoltage) {
    if (calibrationPointCount < 2) return rawVoltage; // Not enough points

    // 1️⃣ Extrapolate for values BELOW the first point
    if (rawVoltage < voltageCalibration[0].rawVoltage) {
        float x0 = voltageCalibration[0].rawVoltage;
        float y0 = voltageCalibration[0].measuredVoltage;
        float x1 = voltageCalibration[1].rawVoltage;
        float y1 = voltageCalibration[1].measuredVoltage;
        
        return y0 + ((rawVoltage - x0) * (y1 - y0) / (x1 - x0));  // Extrapolation
    }

    // 2️⃣ Extrapolate for values ABOVE the last point
    if (rawVoltage > voltageCalibration[calibrationPointCount - 1].rawVoltage) {
        float x0 = voltageCalibration[calibrationPointCount - 2].rawVoltage;
        float y0 = voltageCalibration[calibrationPointCount - 2].measuredVoltage;
        float x1 = voltageCalibration[calibrationPointCount - 1].rawVoltage;
        float y1 = voltageCalibration[calibrationPointCount - 1].measuredVoltage;

        return y0 + ((rawVoltage - x0) * (y1 - y0) / (x1 - x0));  // Extrapolation
    }

    // 3️⃣ Standard Linear Interpolation for in-range values
    for (int i = 0; i < calibrationPointCount - 1; i++) {
        if (rawVoltage >= voltageCalibration[i].rawVoltage && rawVoltage <= voltageCalibration[i + 1].rawVoltage) {
            float x0 = voltageCalibration[i].rawVoltage;
            float y0 = voltageCalibration[i].measuredVoltage;
            float x1 = voltageCalibration[i + 1].rawVoltage;
            float y1 = voltageCalibration[i + 1].measuredVoltage;

            return y0 + ((rawVoltage - x0) * (y1 - y0) / (x1 - x0));  // Linear interpolation
        }
    }

    return rawVoltage;
}

// Reset Voltage Calibration Points
void resetVoltageCalibration() {
    calibrationPointCount = 0;
    memset(voltageCalibration, 0, sizeof(voltageCalibration));  
    EEPROM.put(EEPROM_ADDRESS_VOLTAGE_POINTS, voltageCalibration);
    EEPROM.put(EEPROM_ADDRESS_VOLTAGE_POINTS + sizeof(voltageCalibration), calibrationPointCount);
    EEPROM.commit();
    Serial.println("All voltage calibration points cleared!");
}

// Function to print all calibration points to the Serial Monitor
void showCalibrationPoints() {
  Serial.println("Calibration Points:");
  if (calibrationPointCount <= 0) {
    Serial.println("No calibration points available.");
    return;
  }
  for (int i = 0; i < calibrationPointCount; i++) {
    Serial.print("Point ");
    Serial.print(i);
    Serial.print(": Raw Voltage = ");
    Serial.print(voltageCalibration[i].rawVoltage, 2);
    Serial.print(" V, Calibrated Voltage = ");
    Serial.print(voltageCalibration[i].measuredVoltage, 2);
    Serial.println(" V");
  }
}

// Handle Serial Commands
void handleSerialInput() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.equalsIgnoreCase("stats")) {
            Serial.printf("Number of calibration points: %d\n", calibrationPointCount);
        }
        else if (input.startsWith("+a") || input.startsWith("-a")) {
            int adjustment = input.substring(2).toInt();
            ina226.adjCalibration(adjustment);
            currentCalibrationValue = ina226.getCalibration();
            Serial.printf("Adjusted Current Calibration: %d\n", currentCalibrationValue);
        } 
        else if (input.startsWith("addv")) {  
            if (calibrationPointCount < MAX_CALIBRATION_POINTS) {
                float rawVoltage = ina226.getVoltage();
                float measuredVoltage = input.substring(5).toFloat();
                voltageCalibration[calibrationPointCount++] = {measuredVoltage, rawVoltage};
                Serial.printf("Added Voltage Calibration Point: %.2fV (INA226) → %.2fV (Multimeter)\n", rawVoltage, measuredVoltage);
            } else {
                Serial.println("Max calibration points reached!");
            }
        }
        else if (input.equalsIgnoreCase("resetv")) {  
            resetVoltageCalibration();
        }
        else if (input.equalsIgnoreCase("save")) {
            EEPROM.put(EEPROM_ADDRESS_VOLTAGE_POINTS, voltageCalibration);
            EEPROM.put(EEPROM_ADDRESS_VOLTAGE_POINTS + sizeof(voltageCalibration), calibrationPointCount);
            EEPROM.commit();
            Serial.println("Voltage Calibration Saved!");
        }
        else if(input.equalsIgnoreCase("showv")){
            showCalibrationPoints();
        }
        else {
          Serial.println("Invalid command! Use +a10 / -a10 / addv[voltage] / resetv / save / showv");
        }
    }
}

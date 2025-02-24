#include <Wire.h>
#include <GyverINA.h>
#include <U8g2lib.h>
#include <EEPROM.h>
#include <cmath>

#define I2C_SDA 5
#define I2C_SCL 6

#define BUTTON_PIN 4

const float SHUNT_OHMS = 0.005;
const float MAX_EXPECTED_AMPS = 7.0;

#define EEPROM_ADDRESS_CURRENT 0

#define EEPROM_ADDRESS_VOLTAGE_POINTS 10
#define EEPROM_ADDRESS_VOLTAGE_COUNT (EEPROM_ADDRESS_VOLTAGE_POINTS + (sizeof(CalibrationPoint) * MAX_CALIBRATION_POINTS))
#define EEPROM_ADDRESS_WATTAGE_POINTS (EEPROM_ADDRESS_VOLTAGE_COUNT + sizeof(voltageCalibrationPointCount))
#define EEPROM_ADDRESS_WATTAGE_COUNT (EEPROM_ADDRESS_WATTAGE_POINTS + (sizeof(CalibrationPoint) * MAX_CALIBRATION_POINTS))


#define MAX_CALIBRATION_POINTS 50  
struct CalibrationPoint {
    float measuredValue;  
    float rawValue;       
};

INA226 ina226(SHUNT_OHMS, MAX_EXPECTED_AMPS, 0x40);

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0,U8X8_PIN_NONE);

bool displayEnabled = true;
unsigned long lastButtonPress = 0;
const unsigned long debounceDelay = 200;

uint16_t currentCalibrationValue;
CalibrationPoint voltageCalibration[MAX_CALIBRATION_POINTS];  
CalibrationPoint wattageCalibration[MAX_CALIBRATION_POINTS];  
int voltageCalibrationPointCount = 0;  
int wattageCalibrationPointCount = 0;  

void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    Wire.begin(I2C_SDA, I2C_SCL);
    EEPROM.begin(2048);

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

    EEPROM.get(EEPROM_ADDRESS_VOLTAGE_COUNT, voltageCalibrationPointCount);
    EEPROM.get(EEPROM_ADDRESS_VOLTAGE_POINTS, voltageCalibration);
    EEPROM.get(EEPROM_ADDRESS_VOLTAGE_POINTS + sizeof(voltageCalibration), voltageCalibrationPointCount);

    EEPROM.get(EEPROM_ADDRESS_WATTAGE_COUNT, wattageCalibrationPointCount);
    EEPROM.get(EEPROM_ADDRESS_WATTAGE_POINTS, wattageCalibration);
    EEPROM.get(EEPROM_ADDRESS_WATTAGE_POINTS + sizeof(wattageCalibration), wattageCalibrationPointCount);

    if (voltageCalibrationPointCount > MAX_CALIBRATION_POINTS || voltageCalibrationPointCount < 0) {
        voltageCalibrationPointCount = 0;
    }
    if (wattageCalibrationPointCount > MAX_CALIBRATION_POINTS || wattageCalibrationPointCount < 0) {
        wattageCalibrationPointCount = 0;
    }

    Serial.printf("Loaded Calibration - Current: %d, Voltage Points: %d, Wattage Points: %d\n",
                  currentCalibrationValue, voltageCalibrationPointCount, wattageCalibrationPointCount);

    ina226.setAveraging(INA226_AVG_X4);
    ina226.setSampleTime(INA226_VBUS, INA226_CONV_140US);
    ina226.setSampleTime(INA226_VSHUNT, INA226_CONV_140US);

    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_7x14_tf);
    u8g2.drawStr(0, 14, "Powerbank Ready");
    u8g2.sendBuffer();
    delay(500);
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
    float rawPower = fabs(busVoltage * ina226.getCurrent());
    float power = applyWattageCorrection(rawPower);
    float current = power / busVoltage;

    const char* status = "Idle";
    if (fabs(current) > 0.03) {
        status = (ina226.getCurrent() > 0) ? "Discharge" : "Charging";
    }

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
        
        u8g2.drawStr((128 - u8g2.getStrWidth(buf)) / 2, 28, buf);

        snprintf(buf, sizeof(buf), "%s", status);
        u8g2.drawStr((128 - u8g2.getStrWidth(buf)) / 2, 42, buf);

        u8g2.sendBuffer();
    } else {
        u8g2.setPowerSave(1);
    }

    delay(1000);
}

//linear interpolation with extrapolation
float applyVoltageCorrection(float rawVoltage) {
    if (voltageCalibrationPointCount < 2) return rawVoltage;

    //values BELOW the first point
    if (rawVoltage < voltageCalibration[0].rawValue) {
        float x0 = voltageCalibration[0].rawValue;
        float y0 = voltageCalibration[0].measuredValue;
        float x1 = voltageCalibration[1].rawValue;
        float y1 = voltageCalibration[1].measuredValue;
        
        return y0 + ((rawVoltage - x0) * (y1 - y0) / (x1 - x0));  // Extrapolation
    }

    //values ABOVE the last point
    if (rawVoltage > voltageCalibration[voltageCalibrationPointCount - 1].rawValue) {
        float x0 = voltageCalibration[voltageCalibrationPointCount - 2].rawValue;
        float y0 = voltageCalibration[voltageCalibrationPointCount - 2].measuredValue;
        float x1 = voltageCalibration[voltageCalibrationPointCount - 1].rawValue;
        float y1 = voltageCalibration[voltageCalibrationPointCount - 1].measuredValue;

        return y0 + ((rawVoltage - x0) * (y1 - y0) / (x1 - x0));  // Extrapolation
    }

    //in-range values
    for (int i = 0; i < voltageCalibrationPointCount - 1; i++) {
        if (rawVoltage >= voltageCalibration[i].rawValue && rawVoltage <= voltageCalibration[i + 1].rawValue) {
            float x0 = voltageCalibration[i].rawValue;
            float y0 = voltageCalibration[i].measuredValue;
            float x1 = voltageCalibration[i + 1].rawValue;
            float y1 = voltageCalibration[i + 1].measuredValue;

            return y0 + ((rawVoltage - x0) * (y1 - y0) / (x1 - x0));  // Linear interpolation
        }
    }

    return rawVoltage;
}

//=linear interpolation with extrapolation
float applyWattageCorrection(float rawWattage) {
    if (wattageCalibrationPointCount < 2) return rawWattage; // Not enough points

    //values BELOW the first point
    if (rawWattage < wattageCalibration[0].rawValue) {
        float x0 = wattageCalibration[0].rawValue;
        float y0 = wattageCalibration[0].measuredValue;
        float x1 = wattageCalibration[1].rawValue;
        float y1 = wattageCalibration[1].measuredValue;
        
        return y0 + ((rawWattage - x0) * (y1 - y0) / (x1 - x0));  // Extrapolation
    }

    //values ABOVE the last point
    if (rawWattage > wattageCalibration[wattageCalibrationPointCount - 1].rawValue) {
        float x0 = wattageCalibration[wattageCalibrationPointCount - 2].rawValue;
        float y0 = wattageCalibration[wattageCalibrationPointCount - 2].measuredValue;
        float x1 = wattageCalibration[wattageCalibrationPointCount - 1].rawValue;
        float y1 = wattageCalibration[wattageCalibrationPointCount - 1].measuredValue;

        return y0 + ((rawWattage - x0) * (y1 - y0) / (x1 - x0));  // Extrapolation
    }

    //in-range values
    for (int i = 0; i < wattageCalibrationPointCount - 1; i++) {
        if (rawWattage >= wattageCalibration[i].rawValue && rawWattage <= wattageCalibration[i + 1].rawValue) {
            float x0 = wattageCalibration[i].rawValue;
            float y0 = wattageCalibration[i].measuredValue;
            float x1 = wattageCalibration[i + 1].rawValue;
            float y1 = wattageCalibration[i + 1].measuredValue;

            return y0 + ((rawWattage - x0) * (y1 - y0) / (x1 - x0));  // Linear interpolation
        }
    }

    return rawWattage;
}

// Reset Voltage Calibration Points
void resetVoltageCalibration() {
    voltageCalibrationPointCount = 0;
    EEPROM.put(EEPROM_ADDRESS_VOLTAGE_COUNT, voltageCalibrationPointCount);
    EEPROM.put(EEPROM_ADDRESS_VOLTAGE_POINTS, voltageCalibration);
    EEPROM.put(EEPROM_ADDRESS_VOLTAGE_POINTS + sizeof(voltageCalibration), voltageCalibrationPointCount);
    EEPROM.commit();
    Serial.println("All voltage calibration points cleared!");
}

// Reset Wattage Calibration Points
void resetWattageCalibration() {
    wattageCalibrationPointCount = 0;
    EEPROM.put(EEPROM_ADDRESS_WATTAGE_COUNT, wattageCalibrationPointCount);
    EEPROM.put(EEPROM_ADDRESS_WATTAGE_POINTS, wattageCalibration);
    EEPROM.put(EEPROM_ADDRESS_WATTAGE_POINTS + sizeof(wattageCalibration), wattageCalibrationPointCount);
    EEPROM.commit();
    Serial.println("All wattage calibration points cleared!");
}

// Function to print all calibration points to the Serial Monitor
void showCalibrationPoints() {
  Serial.println("Voltage Calibration Points:");
  if (voltageCalibrationPointCount <= 0) {
    Serial.println("No voltage calibration points available.");
  } else {
    for (int i = 0; i < voltageCalibrationPointCount; i++) {
      Serial.print("Point ");
      Serial.print(i);
      Serial.print(": Raw Voltage = ");
      Serial.print(voltageCalibration[i].rawValue, 2);
      Serial.print(" V, Calibrated Voltage = ");
      Serial.print(voltageCalibration[i].measuredValue, 2);
      Serial.println(" V");
    }
  }

  Serial.println("Wattage Calibration Points:");
  if (wattageCalibrationPointCount <= 0) {
    Serial.println("No wattage calibration points available.");
  } else {
    for (int i = 0; i < wattageCalibrationPointCount; i++) {
      Serial.print("Point ");
      Serial.print(i);
      Serial.print(": Raw Wattage = ");
      Serial.print(wattageCalibration[i].rawValue, 2);
      Serial.print(" W, Calibrated Wattage = ");
      Serial.print(wattageCalibration[i].measuredValue, 2);
      Serial.println(" W");
    }
  }
}

// Handle Serial Commands
void handleSerialInput() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.equalsIgnoreCase("stats")) {
            Serial.printf("Number of voltage calibration points: %d\n", voltageCalibrationPointCount);
            Serial.printf("Number of wattage calibration points: %d\n", wattageCalibrationPointCount);
        }
        else if (input.startsWith("addv")) {  
            if (voltageCalibrationPointCount < MAX_CALIBRATION_POINTS) {
                float rawVoltage = ina226.getVoltage();
                float measuredVoltage = input.substring(5).toFloat();
                voltageCalibration[voltageCalibrationPointCount++] = {measuredVoltage, rawVoltage};
                sortCalibrationPoints(voltageCalibration, voltageCalibrationPointCount);
                Serial.printf("Added Voltage Calibration Point: %.2fV (INA226) → %.2fV (Multimeter)\n", rawVoltage, measuredVoltage);
            } else {
                Serial.println("Max voltage calibration points reached!");
            }
        }
        else if (input.startsWith("addw")) {  
            if (wattageCalibrationPointCount < MAX_CALIBRATION_POINTS) {
                float rawWattage = fabs(ina226.getVoltage() * ina226.getCurrent());  // Ensure rawWattage is always positive
                float measuredWattage = fabs(input.substring(5).toFloat());  // Ensure measuredWattage is always positive
                wattageCalibration[wattageCalibrationPointCount++] = {measuredWattage, rawWattage};
                sortCalibrationPoints(wattageCalibration, wattageCalibrationPointCount);
                Serial.printf("Added Wattage Calibration Point: %.2fW (INA226) → %.2fW (Multimeter)\n", rawWattage, measuredWattage);
            } else {
                Serial.println("Max wattage calibration points reached!");
            }
        }
        else if (input.equalsIgnoreCase("resetv")) {  
            resetVoltageCalibration();
        }
        else if (input.equalsIgnoreCase("resetw")) {  
            resetWattageCalibration();
        }
        else if (input.equalsIgnoreCase("save")) {
            EEPROM.put(EEPROM_ADDRESS_VOLTAGE_POINTS, voltageCalibration);
            EEPROM.put(EEPROM_ADDRESS_VOLTAGE_POINTS + sizeof(voltageCalibration), voltageCalibrationPointCount);
            EEPROM.put(EEPROM_ADDRESS_WATTAGE_POINTS, wattageCalibration);
            EEPROM.put(EEPROM_ADDRESS_WATTAGE_POINTS + sizeof(wattageCalibration), wattageCalibrationPointCount);
            EEPROM.commit();
            Serial.println("Calibration Saved!");
        }
        else if(input.equalsIgnoreCase("show")){
            showCalibrationPoints();
        }
        else if (input.startsWith("removev")) {  
            int index = input.substring(8).toInt();
            if (index >= 0 && index < voltageCalibrationPointCount) {
                for (int i = index; i < voltageCalibrationPointCount - 1; i++) {
                    voltageCalibration[i] = voltageCalibration[i + 1];
                }
                voltageCalibrationPointCount--;
                Serial.printf("Removed Voltage Calibration Point at index %d\n", index);
            } else {
                Serial.println("Invalid index for voltage calibration point!");
            }
        }
        else if (input.startsWith("removew")) {  
            int index = input.substring(8).toInt();
            if (index >= 0 && index < wattageCalibrationPointCount) {
                for (int i = index; i < wattageCalibrationPointCount - 1; i++) {
                    wattageCalibration[i] = wattageCalibration[i + 1];
                }
                wattageCalibrationPointCount--;
                Serial.printf("Removed Wattage Calibration Point at index %d\n", index);
            } else {
                Serial.println("Invalid index for wattage calibration point!");
            }
        }
        else {
          Serial.println("Invalid command!\n addv[voltage] / addw[wattage]\n resetv / resetw\n removew[index] removev[index] \nsave / show");
        }
    }
}

// Sort calibration points by rawValue
void sortCalibrationPoints(CalibrationPoint* points, int count) {
    for (int i = 0; i < count - 1; i++) {
        for (int j = 0; j < count - i - 1; j++) {
            if (points[j].rawValue > points[j + 1].rawValue) {
                CalibrationPoint temp = points[j];
                points[j] = points[j + 1];
                points[j + 1] = temp;
            }
        }
    }
}

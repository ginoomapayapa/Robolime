#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Calibration offset: Actual (240mm) - Measured (300mm) = -60mm
const int16_t OFFSET_MM = -60;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }
  
  Serial.println(F("Adafruit VL53L0X Calibrated Test"));
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
  Serial.println(F("VL53L0X ready!\n")); 
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {
    // Apply calibration offset and prevent negative numbers
    int16_t calibratedDistance = (int16_t)measure.RangeMilliMeter + OFFSET_MM;
    if (calibratedDistance < 0) calibratedDistance = 0;

    Serial.print(F("Raw: "));
    Serial.print(measure.RangeMilliMeter);
    Serial.print(F(" mm | Calibrated: "));
    Serial.print(calibratedDistance);
    Serial.println(F(" mm"));
  } else {
    Serial.println(F("Out of range"));
  }
    
  delay(100);
}

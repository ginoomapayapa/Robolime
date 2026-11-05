#include <HardwareSerial.h>

const int RX_PIN = 15;     // RO from RS485 module -> ESP32 GPIO 15
const int TX_PIN = 2;      // DI from RS485 module -> ESP32 GPIO 2
const int RE_DE_PIN = 4;   // RE and DE tied together -> ESP32 GPIO 4

HardwareSerial rs485(2);

// Helper function to calculate Modbus RTU CRC-16
uint16_t calculateCRC(const byte *buffer, byte length) {
  uint16_t crc = 0xFFFF;
  for (byte pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)buffer[pos];
    for (byte i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  pinMode(RE_DE_PIN, OUTPUT);
  digitalWrite(RE_DE_PIN, LOW); // Set to receive mode initially

  rs485.begin(4800, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.println("RoboLime ESP32 Sensor Initialized (Core + Salinity/TDS).");
  delay(1500);
}

void loop() {
  // ==========================================
  // REQUEST 1: Core Parameters (Moisture, Temp, EC, pH)
  // Registers: 0x0000 to 0x0003 (4 registers)
  // =================0=========================
  byte coreQuery[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00};
  uint16_t coreCrc = calculateCRC(coreQuery, 6);
  coreQuery[6] = lowByte(coreCrc);
  coreQuery[7] = highByte(coreCrc);

  while(rs485.available()) rs485.read(); // Clear buffer

  digitalWrite(RE_DE_PIN, HIGH);
  delay(2);
  rs485.write(coreQuery, sizeof(coreQuery));
  rs485.flush();
  digitalWrite(RE_DE_PIN, LOW);
  delay(2);

  byte coreResp[13];
  int coreBytesRead = 0;
  unsigned long start = millis();
  while (millis() - start < 300) {
    while (rs485.available() && coreBytesRead < 13) {
      coreResp[coreBytesRead++] = rs485.read();
    }
    if (coreBytesRead >= 13) break;
  }

  // ==========================================
  // REQUEST 2: Salinity & TDS
  // Registers: 0x0007 to 0x0008 (2 registers)
  // ==========================================
  byte extraQuery[8] = {0x01, 0x03, 0x00, 0x07, 0x00, 0x02, 0x00, 0x00};
  uint16_t extraCrc = calculateCRC(extraQuery, 6);
  extraQuery[6] = lowByte(extraCrc);
  extraQuery[7] = highByte(extraCrc);

  delay(50); // Small pause between requests
  while(rs485.available()) rs485.read(); // Clear buffer

  digitalWrite(RE_DE_PIN, HIGH);
  delay(2);
  rs485.write(extraQuery, sizeof(extraQuery));
  rs485.flush();
  digitalWrite(RE_DE_PIN, LOW);
  delay(2);

  byte extraResp[9];
  int extraBytesRead = 0;
  start = millis();
  while (millis() - start < 300) {
    while (rs485.available() && extraBytesRead < 9) {
      extraResp[extraBytesRead++] = rs485.read();
    }
    if (extraBytesRead >= 9) break;
  }

  // ==========================================
  // PROCESS AND DISPLAY DATA
  // ==========================================
  if (coreBytesRead == 13) {
    float moisture      = ((coreResp[3] << 8) | coreResp[4]) / 10.0;
    float temperature = ((coreResp[5] << 8) | coreResp[6]) / 10.0;
    uint16_t ec       = (coreResp[7] << 8) | coreResp[8];
    float ph          = ((coreResp[9] << 8) | coreResp[10]) / 10.0;

    Serial.println("--- Comprehensive Soil Telemetry ---");
    Serial.print("Moisture:    "); Serial.print(moisture, 1);    Serial.println(" %");
    Serial.print("Temperature: "); Serial.print(temperature, 1); Serial.println(" °C");
    Serial.print("EC:          "); Serial.print(ec);            Serial.println(" uS/cm");
    Serial.print("pH:          "); Serial.print(ph, 1);           Serial.println(" pH");

    if (extraBytesRead == 9) {
      uint16_t salinity = (extraResp[3] << 8) | extraResp[4];
      uint16_t tds      = (extraResp[5] << 8) | extraResp[6];
      Serial.print("Salinity:    "); Serial.print(salinity);        Serial.println(" mg/L");
      Serial.print("TDS:         "); Serial.print(tds);           Serial.println(" mg/L");
    } else {
      Serial.println("Salinity/TDS: [Read Timeout]");
    }
    Serial.println("------------------------------------");
  } else {
    Serial.println("[ERROR: Failed to read core sensor registers]");
  }

  delay(3000); // Refresh data every 3 seconds
}
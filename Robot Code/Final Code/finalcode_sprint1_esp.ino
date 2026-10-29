#include <SPI.h>
#include <LoRa.h>          
#include <TinyGPSPlus.h>

// --- UART Configuration (ESP32 <-> Nano for Actuator & Pump) ---
#define RXD2_NANO 4  
#define TXD2_NANO 15 

// --- GPS Module Configuration (UART2, RX=16, TX=17, 9600 Baud) ---
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD 9600

HardwareSerial gpsSerial(2); // Use UART2 for GPS

// --- LoRa Pins & 433MHz Configuration (Sandeep Mistry) ---
const int LORA_SCK  = 18;
const int LORA_MISO = 19;
const int LORA_MOSI = 23;
const int LORA_SS   = 5;     
const int LORA_RST  = 14;    
const int LORA_DIO0 = 2;     
const long LORA_FREQUENCY = 433E6; 

// --- Battery Voltage Monitoring Pin & Fine-Tuned Divider ---
const int BATTERY_PIN = 35;           
const float R1_RESISTOR = 56.0; // kOhms
const float R2_RESISTOR = 10.0; // kOhms
// Fine-tuned multiplier adjusted to match 12.36V multimeter reading from 12.41V ESP32 output
const float VOLTAGE_MULTIPLIER = (((R1_RESISTOR + R2_RESISTOR) / R2_RESISTOR) * (12.36 / 11.1)) * (12.36 / 12.41); // ~7.32

// --- Dual BTS7960 Pin Configuration (Left Enable on D33) ---
const int LEFT_EN   = 33;  
const int LEFT_RPWM = 13;
const int LEFT_LPWM = 12;

// Right Driver Module
const int RIGHT_EN  = 25;
const int RIGHT_RPWM = 27;
const int RIGHT_LPWM = 26;

TinyGPSPlus gps;

unsigned long gpsByteCount = 0;
bool uartHandshakeSuccess = false;

enum MenuState { 
  STATE_MAIN_MENU, 
  STATE_GET_PWM, 
  STATE_GET_DIRECTION, 
  STATE_ACTIVE_MOTION, 
  STATE_GET_CUSTOM_UART,
  STATE_LORA_MENU,
  STATE_GET_LORA_MESSAGE
};

MenuState currentMenuState = STATE_MAIN_MENU;
int selectedPwm = 127; 
String currentMotionState = "STOPPED";

void handleMenuInput(String input);
void executeMovement(int directionOption, int pwm);
void stopMotors();
void printStatusReport();
void printMainMenu();
void printDirectionMenu();
void printActiveMotionMenu();
void printLoraMenu();

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println("  ESP32 MASTER SYSTEM INITIALIZATION    ");
  Serial.println("========================================");

  // 1. UART for Nano on Serial1
  Serial1.begin(9600, SERIAL_8N1, RXD2_NANO, TXD2_NANO);
  Serial.println("=== UART Ready (ESP32 to Nano) on Serial1 ===");

  // 2. GPS Serial Initialization (UART2)
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("=== GPS Serial started (9600 Baud) ===");

  // 3. LoRa Init (433MHz)
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa (433MHz) Initialization Failed!");
  } else {
    Serial.println("LoRa (433MHz) Initialized Successfully.");
    LoRa.receive(); 
  }

  // 4. Pins Init
  pinMode(BATTERY_PIN, INPUT);
  pinMode(LEFT_EN, OUTPUT);
  pinMode(LEFT_RPWM, OUTPUT);
  pinMode(LEFT_LPWM, OUTPUT);
  pinMode(RIGHT_EN, OUTPUT);
  pinMode(RIGHT_RPWM, OUTPUT);
  pinMode(RIGHT_LPWM, OUTPUT);

  digitalWrite(LEFT_EN, HIGH);
  digitalWrite(RIGHT_EN, HIGH);
  stopMotors();

  printMainMenu();
}

void loop() {
  // =========================================================================
  // 1. Interactive Serial Menu Parser
  // =========================================================================
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      handleMenuInput(input);
    }
  }

  // =========================================================================
  // 2. UART Communication Check (Incoming from Nano on Serial1)
  // =========================================================================
  if (Serial1.available()) {
    String incoming = Serial1.readStringUntil('\n');
    incoming.trim();
    if (incoming.length() > 0) {
      uartHandshakeSuccess = true;
      Serial.print("[NANO] Says: ");
      Serial.println(incoming);
      Serial1.print("ACK:");
      Serial1.println(incoming);
    }
  }

  // =========================================================================
  // 3. GPS Data Stream Processing via gpsSerial
  // =========================================================================
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gpsByteCount++;
    gps.encode(c);
  }

  // =========================================================================
  // 4. LoRa Packet Listener & Gateway Command Router
  // =========================================================================
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String loraPacket = "";
    while (LoRa.available()) {
      loraPacket += (char)LoRa.read();
    }
    loraPacket.trim();
    
    Serial.print("\n[LoRa 433MHz RX] Content: ");
    Serial.println(loraPacket);

    if (!loraPacket.startsWith("ACK:") && loraPacket != "GET_ROBOT_STATUS") {
      String upperCmd = loraPacket;
      upperCmd.toUpperCase();

      if (upperCmd == "FORWARD" || upperCmd == "MOTOR_FORWARD") {
        executeMovement(1, selectedPwm);
      } 
      else if (upperCmd == "REVERSE" || upperCmd == "MOTOR_REVERSE") {
        executeMovement(2, selectedPwm);
      } 
      else if (upperCmd == "TURN_LEFT" || upperCmd == "MOTOR_LEFT") {
        executeMovement(3, selectedPwm);
      } 
      else if (upperCmd == "TURN_RIGHT" || upperCmd == "MOTOR_RIGHT") {
        executeMovement(4, selectedPwm);
      } 
      else if (upperCmd == "MOTOR_STOP") {
        stopMotors();
        Serial.println("[LoRa Action] BTS7960 Motors Stopped.");
      } 
      else if (upperCmd == "STOP_ALL") {
        stopMotors();
        Serial1.println(loraPacket); 
        Serial.println("[LoRa Action] Stopped local motors & forwarded STOP_ALL to Nano.");
      } 
      else if (upperCmd.startsWith("PWM:")) {
        int pwmVal = upperCmd.substring(4).toInt();
        if (pwmVal >= 0 && pwmVal <= 255) {
          selectedPwm = pwmVal;
          Serial.print("[LoRa Action] PWM updated to: ");
          Serial.println(selectedPwm);
        }
      } 
      else {
        Serial1.println(loraPacket);
        Serial.print("[UART TX] Forwarded to Nano: ");
        Serial.println(loraPacket);
      }
    }

    if (!loraPacket.startsWith("ACK:")) {
      LoRa.beginPacket();
      LoRa.print("ACK:");
      LoRa.print(loraPacket);
      LoRa.endPacket();
      LoRa.receive(); 
      Serial.print("[LoRa 433MHz TX] Sent ACK for: ");
      Serial.println(loraPacket);
    }
  }

  // =========================================================================
  // 5. Periodic Status Broadcast & GPS Check (Every 3 seconds)
  // =========================================================================
  static unsigned long lastStatusCheck = 0;
  const unsigned long statusInterval = 3000;

  if (millis() - lastStatusCheck >= statusInterval) {
    lastStatusCheck = millis();

    // Read Battery Voltage via D35 using fine-tuned calibration multiplier
    int adcValue = analogRead(BATTERY_PIN);
    float pinVoltage = (adcValue / 4095.0) * 3.3; 
    float batteryVoltage = pinVoltage * VOLTAGE_MULTIPLIER; 

    bool gpsFixValid = gps.location.isValid();
    float lat = gpsFixValid ? gps.location.lat() : 0.0;
    float lng = gpsFixValid ? gps.location.lng() : 0.0;

    // --- Format GPS Time to Philippine Time Zone (PHT / UTC+8) Military Time ---
    String phtTimeStr = "";
    if (gps.time.isValid()) {
      int phtHour = gps.time.hour() + 8; // Convert UTC to PHT (UTC+8)
      if (phtHour >= 24) {
        phtHour -= 24; // Handle day wrap-around
      }
      int phtMinute = gps.time.minute();
      int phtSecond = gps.time.second();

      char timeBuffer[10];
      sprintf(timeBuffer, "%02d%02d%02d", phtHour, phtMinute, phtSecond);
      phtTimeStr = String(timeBuffer);
    } else {
      phtTimeStr = String(millis() / 1000); // Fallback until GPS sync
    }

    // Combine Error status and GPS Fix into the ERR: field
    String errCombined = "0_GPS" + String(gpsFixValid ? "1" : "0");

    // Construct status message with BATT formatted to exactly 1 decimal place (e.g. 12.4)
    String robotStatusMsg = "MSG:ROBOT_STATUS,STATE:" + currentMotionState + 
                            ",ERR:" + errCombined + 
                            ",BATT:" + String(batteryVoltage, 1) + 
                            ",LAT:" + String(lat, 6) + ",LNG:" + String(lng, 6) +
                            ",TS:" + phtTimeStr;

    LoRa.beginPacket();
    LoRa.print(robotStatusMsg);
    LoRa.endPacket();
    LoRa.receive(); 
  }
}

void handleMenuInput(String input) {
  switch (currentMenuState) {
    case STATE_MAIN_MENU:
      if (input == "1") {
        currentMenuState = STATE_GET_PWM;
        Serial.println("\n--- STEP 1: SET BTS7960 PWM ---");
        Serial.print("Enter desired PWM value (0 to 255): ");
      } else if (input == "2") {
        stopMotors();
        Serial.println("\n[ACTION] BTS7960 Motors Stopped.");
        printMainMenu();
      } else if (input == "3") {
        currentMenuState = STATE_GET_CUSTOM_UART;
        Serial.println("\n--- SEND CUSTOM UART STRING TO NANO ---");
        Serial.print("Enter custom string/command to send: ");
      } else if (input == "4") {
        currentMenuState = STATE_LORA_MENU;
        printLoraMenu();
      } else {
        printMainMenu();
      }
      break;
    case STATE_GET_PWM:
      {
        int pwmVal = input.toInt();
        if (pwmVal >= 0 && pwmVal <= 255) {
          selectedPwm = pwmVal;
          currentMenuState = STATE_GET_DIRECTION;
          printDirectionMenu();
        }
      }
      break;
    case STATE_GET_DIRECTION:
      executeMovement(input.toInt(), selectedPwm);
      break;
    case STATE_ACTIVE_MOTION:
      if (input == "1") {
        stopMotors();
        currentMenuState = STATE_MAIN_MENU;
        printMainMenu();
      }
      break;
    case STATE_GET_CUSTOM_UART:
      Serial1.println(input);
      currentMenuState = STATE_MAIN_MENU;
      printMainMenu();
      break;
    case STATE_LORA_MENU:
      if (input == "1") {
        currentMenuState = STATE_GET_LORA_MESSAGE;
      } else {
        currentMenuState = STATE_MAIN_MENU;
        printMainMenu();
      }
      break;
    case STATE_GET_LORA_MESSAGE:
      LoRa.beginPacket();
      LoRa.print(input);
      LoRa.endPacket();
      LoRa.receive(); 
      currentMenuState = STATE_MAIN_MENU;
      printMainMenu();
      break;
  }
}

void executeMovement(int directionOption, int pwm) {
  digitalWrite(LEFT_EN, HIGH);
  digitalWrite(RIGHT_EN, HIGH);

  switch (directionOption) {
    case 1: 
      currentMotionState = "FORWARD";
      analogWrite(LEFT_RPWM, pwm);  analogWrite(LEFT_LPWM, 0);
      analogWrite(RIGHT_RPWM, pwm); analogWrite(RIGHT_LPWM, 0);
      break;
    case 2: 
      currentMotionState = "REVERSING";
      analogWrite(LEFT_RPWM, 0);  analogWrite(LEFT_LPWM, pwm);
      analogWrite(RIGHT_RPWM, 0); analogWrite(RIGHT_LPWM, pwm);
      break;
    case 3: 
      currentMotionState = "TURNING LEFT";
      analogWrite(LEFT_RPWM, 0);    analogWrite(LEFT_LPWM, 0);
      analogWrite(RIGHT_RPWM, pwm); analogWrite(RIGHT_LPWM, 0);
      break;
    case 4: 
      currentMotionState = "TURNING RIGHT";
      analogWrite(LEFT_RPWM, pwm);  analogWrite(LEFT_LPWM, 0);
      analogWrite(RIGHT_RPWM, 0);   analogWrite(RIGHT_LPWM, 0);
      break;
    default:
      stopMotors();
      break;
  }
  printStatusReport();
  currentMenuState = STATE_ACTIVE_MOTION;
  printActiveMotionMenu();
}

void stopMotors() {
  analogWrite(LEFT_RPWM, 0);
  analogWrite(LEFT_LPWM, 0);
  analogWrite(RIGHT_RPWM, 0);
  analogWrite(RIGHT_LPWM, 0);
  currentMotionState = "STOPPED";
}

void printStatusReport() {
  Serial.println("\n========================================");
  Serial.println("           ROBOT STATUS REPORT          ");
  Serial.println("========================================");
  Serial.print  (" -> Motion State : "); Serial.println(currentMotionState);
  Serial.print  (" -> Active PWM   : "); Serial.println(selectedPwm);
  Serial.println("========================================");
}

void printMainMenu() {
  Serial.println("\n[1] Move/PWM [2] Stop Motors [3] UART to Nano [4] LoRa Menu");
  Serial.print("Select option (1-4): ");
}

void printDirectionMenu() {
  Serial.println("\n[1] Forward [2] Reverse [3] Turn Left [4] Turn Right");
  Serial.print("Select option (1-4): ");
}

void printActiveMotionMenu() {
  Serial.println("\n[1] Stop Motors [2] New Movement");
  Serial.print("Select option (1-2): ");
}

void printLoraMenu() {
  Serial.println("\n[1] Send LoRa Packet [2] Back");
  Serial.print("Select option (1-2): ");
}

#include <SoftwareSerial.h>
#include <AltSoftSerial.h>

SoftwareSerial nanoSerial(5, 6); // RX = D5, TX = D6
AltSoftSerial rs485Serial;       // RO = D8, DI = D9
const int RS485_RE_DE = 4;       

const int ULTRASONIC_TRIG = 11;
const int ULTRASONIC_ECHO = 12;

const int ACTUATOR_IN3 = 13;  
const int ACTUATOR_IN4 = 10;  

const int PUMP_PIN = 7; 

String actuatorState = "STOPPED"; 
String pumpState = "OFF";         

float currentPh = 7.00;
float currentMoisture = 0.00;
float currentTemp = 25.00;
float currentEc = 0.00;
float currentDistance = 0.00;

bool espAcknowledged = false;
unsigned long lastPingTime = 0;
const unsigned long pingInterval = 2000; 

unsigned long lastStatusTime = 0;
const unsigned long statusInterval = 3000;

void setup() {
  Serial.begin(9600);
  while (!Serial);                    

  nanoSerial.begin(9600);
  rs485Serial.begin(9600); 

  pinMode(RS485_RE_DE, OUTPUT);
  digitalWrite(RS485_RE_DE, LOW); 

  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);

  pinMode(ACTUATOR_IN3, OUTPUT);
  pinMode(ACTUATOR_IN4, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  
  stopAll();
  Serial.println("=== Nano Initialized ===");
}

void loop() {
  if (!espAcknowledged) {
    if (millis() - lastPingTime >= pingInterval) {
      lastPingTime = millis();
      nanoSerial.println("NANO_READY");
      Serial.println("[UART] Sent initialization string to ESP32: NANO_READY");
    }
  }

  if (nanoSerial.available()) {
    String incoming = nanoSerial.readStringUntil('\n');
    incoming.trim();
    
    if (incoming.equalsIgnoreCase("ACK:NANO_READY")) {
      if (!espAcknowledged) {
        espAcknowledged = true;
        Serial.print("[UART HANDSHAKE SUCCESS] Acknowledgment received: ");
        Serial.println(incoming);
      }
    } 
    else if (incoming.startsWith("ACK:")) {
      // Ignore generic ACKs
    } 
    else {
      String upperCmd = incoming;
      upperCmd.toUpperCase();
      if (upperCmd.length() > 0) {
        Serial.print("[ESP32] Says: ");
        Serial.println(incoming);
        
        executeCommand(upperCmd);

        nanoSerial.print("ACK:");
        nanoSerial.println(incoming);
        Serial.print("[UART TX] Sent ACK back to ESP32 for: ");
        Serial.println(incoming);
      }
    }
  }

  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();
    if (msg.length() > 0) {
      nanoSerial.println(msg);
      Serial.print("[NANO] Sent: ");
      Serial.println(msg);
    }
  }

  if (millis() - lastStatusTime >= statusInterval) {
    lastStatusTime = millis();
    runStatusChecker();
  }
}

void runStatusChecker() {
  currentDistance = readUltrasonicDistance();
  bool ultrasonicError = (currentDistance < 0);
  bool phSensorError = !readSoilSensor();

  Serial.println("\n----------------------------------------");
  Serial.println("          NANO STATUS REPORT            ");
  Serial.println("----------------------------------------");
  
  Serial.print("Status Error : ");
  if (!ultrasonicError && !phSensorError) {
    Serial.println("None [All Systems OK]");
  } else if (ultrasonicError && phSensorError) {
    Serial.println("Both (Ultrasonic Sensor & pH/Soil Sensor Fault)");
  } else if (ultrasonicError) {
    Serial.println("Ultrasonic Sensor Fault (Out of range / Timeout)");
  } else if (phSensorError) {
    Serial.println("pH / Soil Sensor Fault (RS485 Communication Error)");
  }

  Serial.print("pH Level     : "); Serial.println(currentPh, 2);
  Serial.print("Moisture     : "); Serial.print(currentMoisture, 1); Serial.println(" %");
  Serial.print("Temperature  : "); Serial.print(currentTemp, 1); Serial.println(" °C");
  Serial.print("EC           : "); Serial.print(currentEc, 2); Serial.println(" mS/cm");
  Serial.print("Distance     : "); Serial.print(currentDistance, 1); Serial.println(" cm");
  
  Serial.print("Actuator     : "); Serial.println(actuatorState);
  Serial.print("Pump         : "); Serial.println(pumpState);
  Serial.println("----------------------------------------");
}

float readUltrasonicDistance() {
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);

  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000); 
  if (duration == 0) return -1.0; 
  
  return (duration * 0.034 / 2.0); 
}

bool readSoilSensor() {
  digitalWrite(RS485_RE_DE, HIGH); 
  delay(2);
  rs485Serial.println("GET_SOIL_DATA"); 

  digitalWrite(RS485_RE_DE, LOW);  
  delay(15);

  bool success = false;
  if (rs485Serial.available()) {
    String response = rs485Serial.readStringUntil('\n');
    response.trim();
    if (response.length() > 0) {
      success = true; 
    }
  }
  return success;
}

void executeCommand(String cmd) {
  if (cmd == "ACTUATOR_EXTEND") {
    digitalWrite(ACTUATOR_IN3, HIGH);
    digitalWrite(ACTUATOR_IN4, LOW);
    actuatorState = "EXTENDING";
    Serial.println("[ACTION] Linear Actuator Extending...");
  } 
  else if (cmd == "ACTUATOR_RETRACT") {
    digitalWrite(ACTUATOR_IN3, LOW);
    digitalWrite(ACTUATOR_IN4, HIGH);
    actuatorState = "RETRACTING";
    Serial.println("[ACTION] Linear Actuator Retracting...");
  } 
  else if (cmd == "ACTUATOR_STOP") {
    digitalWrite(ACTUATOR_IN3, LOW);
    digitalWrite(ACTUATOR_IN4, LOW);
    actuatorState = "STOPPED";
    Serial.println("[ACTION] Linear Actuator Stopped.");
  }
  else if (cmd == "PUMP_ON") {
    digitalWrite(PUMP_PIN, HIGH);
    pumpState = "ON";
    Serial.println("[ACTION] Diaphragm Pump ON.");
  } 
  else if (cmd == "PUMP_OFF" || cmd == "PUMP_STOP") {
    digitalWrite(PUMP_PIN, LOW);
    pumpState = "OFF";
    Serial.println("[ACTION] Diaphragm Pump Stopped/OFF.");
  } 
  else if (cmd == "STOP_ALL") {
    stopAll();
    Serial.println("[ACTION] All Nano actuators and pumps stopped.");
  } 
  else {
    Serial.println("[WARNING] Unknown command received.");
  }
}

void stopAll() {
  digitalWrite(ACTUATOR_IN3, LOW);
  digitalWrite(ACTUATOR_IN4, LOW);
  digitalWrite(PUMP_PIN, LOW);
  actuatorState = "STOPPED";
  pumpState = "OFF";
}

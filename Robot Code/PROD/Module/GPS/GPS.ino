#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

// Define ESP32 Hardware Serial 2 pins for GPS
const int RXD2 = 17;
const int TXD2 = 16;

// Initialize HardwareSerial and TinyGPS++ objects
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // NEO-6M default baud rate is typically 9600
  gpsSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);
  
  Serial.println("NEO-6M GPS Test Initialized.");
  Serial.println("Waiting for satellite lock (this may take a few minutes outdoors)...");
}

void loop() {
  // Read incoming characters from the GPS module
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    // Feed the data into the TinyGPS++ parser
    gps.encode(c);
  }

  // Print data updates once per second if new valid info is found
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("WARNING: No GPS data received. Check wiring (TX->16, RX->17) and baud rate!");
    delay(5000);
  }

  if (gps.location.isUpdated()) {
    Serial.print("Latitude: "); 
    Serial.print(gps.location.lat(), 6);
    Serial.print(" | Longitude: "); 
    Serial.print(gps.location.lng(), 6);
    
    Serial.print(" | Satellites: "); 
    Serial.print(gps.satellites.value());
    
    Serial.print(" | Altitude: "); 
    Serial.print(gps.altitude.meters());
    Serial.println("m");
  }
  
  delay(100);
}
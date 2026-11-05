#include <Wire.h>
#include <Adafruit_QMC5883P.h>

const int sdaPin = 21;
const int sclPin = 22;

Adafruit_QMC5883P qmc;

// Structure to hold the reference raw values for each cardinal direction
struct CardinalReference {
  int16_t x;
  int16_t y;
  const char* name;
};

// Your captured raw averages for North, South, East, and West
CardinalReference targets[] = {
  { -205, -1155, "North" },
  { 1005,  1040, "South" },
  { -620,  -130, "East"  },
  { 1480,  -100, "West"  }
};

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Wire.begin(sdaPin, sclPin);

  if (!qmc.begin(0x2C, &Wire)) {
    Serial.println("Failed to find QMC5883P chip!");
    while (1) delay(10);
  }

  qmc.setMode(QMC5883P_MODE_NORMAL);
  qmc.setODR(QMC5883P_ODR_50HZ);
  qmc.setRange(QMC5883P_RANGE_8G);
  
  Serial.println("Raw Coordinate Direction Matcher Initialized.");
}

void loop() {
  if (qmc.isDataReady()) {
    int16_t rawX, rawY, rawZ;

    if (qmc.getRawMagnetic(&rawX, &rawY, &rawZ)) {
      
      // Find the closest matching cardinal direction based on raw X and Y distance
      const char* closestDirection = "Unknown";
      float minDistance = 999999.0;

      for (int i = 0; i < 4; i++) {
        float dx = (float)rawX - targets[i].x;
        float dy = (float)rawY - targets[i].y;
        float distance = sqrt((dx * dx) + (dy * dy));

        if (distance < minDistance) {
          minDistance = distance;
          closestDirection = targets[i].name;
        }
      }

      // Print raw data and the matched direction
      Serial.print("Raw X: ");
      Serial.print(rawX);
      Serial.print(" | Raw Y: ");
      Serial.print(rawY);
      Serial.print(" | Direction: ");
      Serial.println(closestDirection);
    }
  }
  
  delay(200);
}
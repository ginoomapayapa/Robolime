#include <SPI.h>
#include <LoRa.h>

// --- LOCKED PIN CONFIGURATION ---
const int csPin = 5;           // LoRa NSS (GPIO 5)
const int resetPin = 4;        // LoRa RST (GPIO 4 - RC Reset Circuit)
const int dio0Pin = 35;        // LoRa DIO0 (GPIO 35)
const int sckPin = 18;         // SCK (GPIO 18)
const int misoPin = 19;        // MISO (GPIO 19)
const int mosiPin = 23;        // MOSI (GPIO 23)

const long frequency = 433E6;  // 433 MHz for Ra-02

// Menu States
enum MenuState {
  MAIN_MENU,
  LORA_CUSTOM_MESSAGE
};

MenuState currentMenuState = MAIN_MENU;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(1000);

  Serial.println("\n=== Robot Master Node Initializing ===");

  // Initialize SPI bus with your exact pin map
  SPI.begin(sckPin, misoPin, mosiPin, csPin);
  LoRa.setPins(csPin, resetPin, dio0Pin);

  if (!LoRa.begin(frequency)) {
    Serial.println("Starting LoRa failed! Check connections.");
    while (1);
  }

  Serial.println("LoRa Initialized Successfully!");
  printMainMenu();
}

void loop() {
  // 1. Handle Serial Menu & User Input
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      handleMenuInput(input);
    }
  }

  // 2. Continuously listen for incoming LoRa packets from the gateway/other nodes
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String incomingCommand = "";
    while (LoRa.available()) {
      incomingCommand += (char)LoRa.read();
    }
    incomingCommand.trim();

    // If it's an ACK meant for us, handle it or let the retry loop catch it.
    // Otherwise, treat it as a command to execute.
    if (!incomingCommand.startsWith("ACK:")) {
      Serial.println("\n--- [LoRa RX Command] ---");
      Serial.print("Received: '");
      Serial.print(incomingCommand);
      Serial.print("' | RSSI: ");
      Serial.println(LoRa.packetRssi());

      executeRobotCommand(incomingCommand);
      sendAcknowledgment(incomingCommand);
      Serial.println("-------------------------");
    }
  }
}

// --- MENU SYSTEM ---

void printMainMenu() {
  Serial.println("\n==============================");
  Serial.println("       ROBOT MAIN MENU        ");
  Serial.println("==============================");
  Serial.println("1. LoRa (Send Custom Message/Command with Retry)");
  // Future options can be added here easily:
  // Serial.println("2. Run Motor Test");
  Serial.println("------------------------------");
  Serial.print("Enter your choice: ");
  currentMenuState = MAIN_MENU;
}

void handleMenuInput(String input) {
  switch (currentMenuState) {
    case MAIN_MENU:
      if (input == "1") {
        currentMenuState = LORA_CUSTOM_MESSAGE;
        Serial.println("\n[LoRa Transmission Mode Selected]");
        Serial.print("Type the custom message/command to send: ");
      } 
      else {
        Serial.println("Invalid choice. Please try again.");
        printMainMenu();
      }
      break;

    case LORA_CUSTOM_MESSAGE:
      {
        Serial.println("\nSending message with ACK retry logic...");
        
        // Call the retry function
        bool success = sendLoRaWithRetry(input, 3, 2000); // 3 retries, 2000ms timeout per try
        
        if (success) {
          Serial.println("-> Transmission confirmed via ACK!");
        } else {
          Serial.println("-> Transmission FAILED: No ACK received after max retries.");
        }
        
        // Return to main menu after transaction attempt
        printMainMenu();
      }
      break;
  }
}

// --- RETRY & ACK TRANSMISSION LOGIC ---

bool sendLoRaWithRetry(String message, int maxRetries, unsigned long timeoutMs) {
  for (int attempt = 1; attempt <= maxRetries; attempt++) {
    Serial.print("[Attempt ");
    Serial.print(attempt);
    Serial.print("/");
    Serial.print(maxRetries);
    Serial.print("] Transmitting: ");
    Serial.println(message);

    // 1. Send Packet
    LoRa.beginPacket();
    LoRa.print(message);
    LoRa.endPacket();

    // 2. Wait and listen for matching ACK
    unsigned long startTime = millis();
    bool ackReceived = false;

    while (millis() - startTime < timeoutMs) {
      int packetSize = LoRa.parsePacket();
      if (packetSize) {
        String response = "";
        while (LoRa.available()) {
          response += (char)LoRa.read();
        }
        response.trim();

        // Check if the response is an ACK matching what we sent
        if (response == "ACK:" + message) {
          Serial.print("Received valid ACK: '");
          Serial.print(response);
          Serial.print("' | RSSI: ");
          Serial.println(LoRa.packetRssi());
          ackReceived = true;
          break;
        }
      }
    }

    if (ackReceived) {
      return true; // Exit loop successfully
    } else {
      Serial.println("-> Timeout waiting for ACK. Retrying...");
      delay(500); // Short delay before next retry
    }
  }

  return false; // Exhausted all retries without an ACK
}

// --- ROBOT COMMAND EXECUTION ---

void executeRobotCommand(String cmd) {
  if (cmd == "FORWARD") {
    Serial.println("-> Action: Moving Robot Forward");
  } 
  else if (cmd == "BACKWARD") {
    Serial.println("-> Action: Moving Robot Backward");
  } 
  else if (cmd == "STOP") {
    Serial.println("-> Action: Stopping Motors");
  } 
  else {
    Serial.println("-> Action: Custom command executed.");
  }
}

void sendAcknowledgment(String originalCmd) {
  String ackMessage = "ACK:" + originalCmd;
  LoRa.beginPacket();
  LoRa.print(ackMessage);
  LoRa.endPacket();
  Serial.print("Sent Response: ");
  Serial.println(ackMessage);
}
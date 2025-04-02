#include <SoftwareSerial.h>
#include <Wire.h> 

// --- Pin Definitions ---
// Relays for Motors
const int RELAY_PIN_1 = 12; // Pin for the first motor's relay
const int RELAY_PIN_2 = 11; // Pin for the second motor's relay

// RS485 Module Control Pins (for NPK Sensor)
#define RE 8 // Receiver Enable Pin
#define DE 7 // Driver Enable Pin

// SoftwareSerial Pins for RS485 Module (RX, TX)
// Use pins 2, 3 for Uno/Nano etc.
SoftwareSerial mod(2, 3);
// If using Mega, you might use HardwareSerial or different pins:
// SoftwareSerial mod(10, 11); // Conflicts with RELAY_PIN_2 if set to 11!

// --- NPK Sensor Settings ---
// Modbus RTU requests for reading NPK values
const byte nitro[] = {0x01, 0x03, 0x00, 0x1e, 0x00, 0x01, 0xe4, 0x0c}; // Request Nitrogen
const byte phos[]  = {0x01, 0x03, 0x00, 0x1f, 0x00, 0x01, 0xb5, 0xcc}; // Request Phosphorus
const byte pota[]  = {0x01, 0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xc0}; // Request Potassium

// Buffer to store Modbus response data
byte values[11]; // Needs to be large enough for expected response

// --- Control Logic Settings ---
const int NITROGEN_THRESHOLD = 1.0; 

const unsigned long MOTOR_ON_DURATION = 10000; // 10 seconds in milliseconds
const unsigned long WAIT_BETWEEN_MOTORS = 60000; // 1 minute in milliseconds
const unsigned long MONITORING_DELAY = 5000;   // Check sensor every 5 seconds when idle

// ================= SETUP =================
void setup() {
  // Initialize Serial Monitor for debugging
  Serial.begin(9600);
  Serial.println("Starting Integrated Control System...");

  // Initialize SoftwareSerial for NPK Sensor
  mod.begin(9600);

  // Initialize Relay Pins
  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);
  // Ensure motors start OFF (assuming Active HIGH relays)
  digitalWrite(RELAY_PIN_1, LOW);
  digitalWrite(RELAY_PIN_2, LOW);
  Serial.println("Relay pins initialized.");

  // Initialize RS485 Control Pins
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);
  // Set RS485 initially to receiving mode
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);
  Serial.println("RS485 pins initialized.");

  delay(500); // Allow time for components to stabilize
  Serial.println("Setup complete. Entering main loop.");
}

// ================= LOOP =================
void loop() {
  // 1. Read Nitrogen Value
  int currentNitrogen = nitrogen(); // Read from sensor
  delay(250); // Short delay between Modbus commands as in original code

  // Optional: Read P and K if needed for logging, but they don't affect motor control here
  // int currentPhos = phosphorous();
  // delay(250);
  // int currentPota = potassium();
  // delay(250);

  // 2. Print current Nitrogen value
  Serial.print("Current Nitrogen: ");
  Serial.print(currentNitrogen);
  Serial.println(" mg/kg");

  // 3. Check Nitrogen Level and Control Motors
  if (currentNitrogen > NITROGEN_THRESHOLD) {
    Serial.println("Nitrogen level HIGH. Activating motor sequence.");

    // --- Motor 1 Sequence ---
    Serial.println("Turning Motor 1 ON");
    digitalWrite(RELAY_PIN_1, HIGH); // Turn Motor 1 ON (use LOW if Active LOW relay)
    delay(MOTOR_ON_DURATION);        // Keep it ON for 10 seconds
    Serial.println("Turning Motor 1 OFF");
    digitalWrite(RELAY_PIN_1, LOW);  // Turn Motor 1 OFF (use HIGH if Active LOW relay)

    // --- Wait between Motors ---
    Serial.println("Waiting for 1 minute...");
    delay(WAIT_BETWEEN_MOTORS);     // Wait for 1 minute

    // --- Motor 2 Sequence ---
    Serial.println("Turning Motor 2 ON");
    digitalWrite(RELAY_PIN_2, HIGH); // Turn Motor 2 ON (use LOW if Active LOW relay)
    delay(MOTOR_ON_DURATION);        // Keep it ON for 10 seconds
    Serial.println("Turning Motor 2 OFF");
    digitalWrite(RELAY_PIN_2, LOW);  // Turn Motor 2 OFF (use HIGH if Active LOW relay)

    Serial.println("Motor sequence complete. Re-checking Nitrogen level soon.");
    // Loop will automatically restart and check Nitrogen again immediately

  } else {
    // Nitrogen level is OK
    Serial.println("Nitrogen level OK. Monitoring...");
    // Ensure motors are definitely off (safety check)
    digitalWrite(RELAY_PIN_1, LOW);
    digitalWrite(RELAY_PIN_2, LOW);

    // Wait before checking the sensor again
    delay(MONITORING_DELAY);
  }
}

// ================= NPK Sensor Reading Functions =================

// Function to read Nitrogen value
int nitrogen() {
  // Set RS485 to transmit mode
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(10); // Allow time for mode change

  // Send Modbus request for Nitrogen
  mod.write(nitro, sizeof(nitro));

  // Set RS485 back to receive mode
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);

  // Wait for and read the response (expecting 7 bytes for standard 1-register read response)
  byte byteCount = 0;
  unsigned long startTime = millis();
  while (mod.available() < 7 && (millis() - startTime < 1000)) {
      // Wait for 7 bytes or timeout after 1 second
      delay(1); // Small delay to prevent busy-waiting
  }

  if (mod.available() >= 7) {
      Serial.print("N Response HEX: ");
      for (byte i = 0; i < 7; i++) {
          values[i] = mod.read();
          // Serial.print(values[i], HEX); // Uncomment for raw HEX debugging
          // Serial.print(" ");
      }
      // Serial.println(); // Uncomment for raw HEX debugging

      // Combine High Byte (values[3]) and Low Byte (values[4]) for the result
      // Assuming standard Modbus response format: Addr Func ByteCount DataHi DataLo CRCHi CRCLo
      int result = (values[3] << 8) | values[4];
      // Basic CRC check placeholder (optional but recommended for robustness)
      // You would calculate CRC on first 5 bytes and compare with values[5] & values[6]
      return result;
  } else {
      Serial.println("Nitrogen read timeout or insufficient data!");
      // Flush buffer if partial data received
      while(mod.available()) { mod.read(); }
      return -1; // Return -1 or some error code
  }
}


// Function to read Phosphorus value (similar structure to nitrogen)
int phosphorous() {
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(10);
  mod.write(phos, sizeof(phos));
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);

  byte byteCount = 0;
  unsigned long startTime = millis();
   while (mod.available() < 7 && (millis() - startTime < 1000)) { delay(1); }

  if (mod.available() >= 7) {
      Serial.print("P Response HEX: ");
      for (byte i = 0; i < 7; i++) {
          values[i] = mod.read();
          // Serial.print(values[i], HEX); // Uncomment for raw HEX debugging
          // Serial.print(" ");
      }
      // Serial.println(); // Uncomment for raw HEX debugging
      int result = (values[3] << 8) | values[4];
      return result;
  } else {
       Serial.println("Phosphorus read timeout or insufficient data!");
       while(mod.available()) { mod.read(); }
       return -1;
  }
}

// Function to read Potassium value (similar structure to nitrogen)
int potassium() {
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(10);
  mod.write(pota, sizeof(pota));
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);

  byte byteCount = 0;
  unsigned long startTime = millis();
  while (mod.available() < 7 && (millis() - startTime < 1000)) { delay(1); }

  if (mod.available() >= 7) {
      Serial.print("K Response HEX: ");
      for (byte i = 0; i < 7; i++) {
          values[i] = mod.read();
          // Serial.print(values[i], HEX); // Uncomment for raw HEX debugging
          // Serial.print(" ");
      }
      // Serial.println(); // Uncomment for raw HEX debugging
      int result = (values[3] << 8) | values[4];
      return result;
  } else {
      Serial.println("Potassium read timeout or insufficient data!");
      while(mod.available()) { mod.read(); }
      return -1;
  }
}

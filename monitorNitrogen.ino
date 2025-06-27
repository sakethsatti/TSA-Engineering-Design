// =================================================================
// ==      ESP32 AUTOMATIC WATER PURITY CONTROLLER (DUAL SENSOR)  ==
// =================================================================
// This script uses Sensor 1 as the primary control and Sensor 2 for monitoring.
//
// LOGIC (based on Sensor 1):
// - If TDS > 300 PPM (Impure):         Activate Motor 1.
// - If 31 <= TDS <= 300 PPM (Pure):    Activate Motor 2.
// - If TDS <= 30 PPM (No Water/Empty): Do nothing.
//

// --- Hardware Pin Definitions ---
const int TDS_PIN_1   = 34; // GPIO34 - PRIMARY CONTROL SENSOR for decision making
const int TDS_PIN_2   = 35; // GPIO35 - MONITORING SENSOR for additional data
const int RELAY_PIN_1 = 25; // Relay for Motor 1 (Impure Water Pump)
const int RELAY_PIN_2 = 26; // Relay for Motor 2 (Pure Water Pump)

// --- Logic & Control Constants ---
const int IMPURE_THRESHOLD   = 300;  // PPM value above which water is "impure"
const int NO_WATER_THRESHOLD = 30;   // PPM value below which we assume no water / tank empty
const int MOTOR_RUN_TIME     = 5000; // How long to run the motor (in milliseconds)
const int CHECK_INTERVAL     = 10000; // How often to evaluate water purity (in milliseconds)

// --- TDS Measurement Constants ---
#define VREF           3.3      // ESP32 ADC reference voltage is 3.3V
#define ADC_RESOLUTION 4095.0   // ESP32 ADC is 12-bit (0-4095)
#define SCOUNT         30       // Number of samples to take for a stable reading (median filter)

// --- Data Structure for a Sensor ---
struct TdsSensor {
  const byte pin;               // The GPIO pin the sensor is connected to
  int analogBuffer[SCOUNT];     // Array to store raw sensor readings for filtering
  int analogBufferIndex;        // Current position/index in the analog buffer
};

// --- Global Variables ---
TdsSensor sensor1 = {TDS_PIN_1};
TdsSensor sensor2 = {TDS_PIN_2};

// Assumed water temperature for TDS compensation.
float temperature = 25.0;

// =================================================================
// ==                          SETUP                              ==
// =================================================================
void setup() {
  // Initialize serial communication to send data to the Serial Monitor.
  Serial.begin(115200);
  Serial.println("\n--- Dual Sensor Water Purity Controller Initialized ---");

  // Set the relay pins as outputs to control the motors.
  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);

  // Ensure both motors are OFF at startup for safety.
  digitalWrite(RELAY_PIN_1, LOW);
  digitalWrite(RELAY_PIN_2, LOW); 
}


// =================================================================
// ==                           LOOP                              ==
// =================================================================
void loop() {
  // from BOTH sensors every 40 milliseconds for real-time data collection.
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) {
    analogSampleTimepoint = millis();

    // Sample Sensor 1 and store the reading in its buffer.
    sensor1.analogBuffer[sensor1.analogBufferIndex++] = analogRead(sensor1.pin);
    if (sensor1.analogBufferIndex == SCOUNT) {
      sensor1.analogBufferIndex = 0; 
    }

    // Sample Sensor 2 and store its reading.
    sensor2.analogBuffer[sensor2.analogBufferIndex++] = analogRead(sensor2.pin);
    if (sensor2.analogBufferIndex == SCOUNT) {
      sensor2.analogBufferIndex = 0;
    }
  }

  // This is the main control timer. It triggers the purity check and motor logic
  // at the defined CHECK_INTERVAL (e.g., every 10 seconds).
  static unsigned long lastCheckTime = 0;
  if (millis() - lastCheckTime > CHECK_INTERVAL) {
    lastCheckTime = millis(); // Reset the timer to start the next interval

    // --- Step 1: Calculate the current TDS values for both sensors ---
    float controlTdsValue = getCurrentTdsValue(sensor1);    // Value from the primary sensor
    float monitoringTdsValue = getCurrentTdsValue(sensor2); // Value from the secondary sensor

    Serial.println("\n-------------------------------------------------");
    Serial.print("Checking water... (Cycle every ");
    Serial.print(CHECK_INTERVAL / 1000);
    Serial.println(" seconds)");

    // Print the TDS value from the primary control sensor.
    Serial.print("  - Control Sensor (Pin ");
    Serial.print(sensor1.pin);
    Serial.print(") TDS: ");
    Serial.print(controlTdsValue, 0); 
    Serial.println(" ppm");

    // Print the TDS value from the secondary monitoring sensor.
    Serial.print("  - Monitoring Sensor (Pin ");
    Serial.print(sensor2.pin);
    Serial.print(") TDS: ");
    Serial.print(monitoringTdsValue, 0);
    Serial.println(" ppm");

    // --- Step 2: Decide what to do based on the CONTROL sensor's value ---
    if (controlTdsValue > IMPURE_THRESHOLD) {
      Serial.println("  - Result: Water is IMPURE.");
      runMotor(1, MOTOR_RUN_TIME); // Run Motor 1 (for impure water)

    } else if (controlTdsValue > NO_WATER_THRESHOLD) {
      Serial.println("  - Result: Water is PURE.");
      runMotor(2, MOTOR_RUN_TIME); // Run Motor 2 (for pure water)

    } else {
      Serial.println("  - Result: No water detected or very low TDS.");
      Serial.println("  - Action: Doing nothing."); // Stay idle if no water
    }
  }
}


// =================================================================
// ==                      HELPER FUNCTIONS                       ==
// =================================================================

/**
 * @brief Activates a specified motor's relay for a given duration.
 * Prints action messages to the Serial Monitor.
 *
 * @param motorNumber The motor to operate (1 for Motor 1, 2 for Motor 2).
 * @param duration The time in milliseconds the motor should run.
 */
void runMotor(int motorNumber, int duration) {
  // Determine which relay pin corresponds to the requested motor.
  int relayPin = (motorNumber == 1) ? RELAY_PIN_1 : RELAY_PIN_2;

  Serial.print("  - Action: Running Motor ");
  Serial.print(motorNumber);
  Serial.print(" for ");
  Serial.print(duration / 1000); // Convert milliseconds to seconds for display
  Serial.println(" seconds...");

  // Activate the relay to turn the motor ON.
  digitalWrite(relayPin, HIGH);
  delay(duration); 

  // Deactivate the relay to turn the motor OFF.
  digitalWrite(relayPin, LOW);

  Serial.print("  - Action complete. Motor ");
  Serial.print(motorNumber);
  Serial.println(" OFF.");
}

/**
 * @brief Calculates the TDS value (in PPM) for a given sensor object.
 * This function applies median filtering, voltage conversion, and temperature compensation.
 *
 * @param sensor A reference to the TdsSensor object to process.
 * @return The calculated TDS value in PPM (float).
 */
float getCurrentTdsValue(TdsSensor &sensor) {
  // Get the median value from the sensor's buffer to filter out noise.
  // We pass a copy of the buffer to avoid modifying the original during sorting.
  int medianValue = getMedianNum(sensor.analogBuffer, SCOUNT);

  // Convert the raw ADC median value to a voltage.
  float voltage = medianValue * VREF / ADC_RESOLUTION;

  // Calculate the temperature compensation coefficient.
  float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);

  // Apply temperature compensation to the voltage.
  float compensationVoltage = voltage / compensationCoefficient;

  // Convert the compensated voltage into a TDS value (PPM) using the sensor's formula.
  float currentTdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage -
                           255.86 * compensationVoltage * compensationVoltage +
                           857.39 * compensationVoltage) * 0.5;

  return currentTdsValue;
}

/**
 * @brief Implements a median filtering algorithm.
 * Sorts an array of integers and returns the median value. This is highly effective
 * for reducing random noise in sensor readings.
 *
 * @param bArray The array of integer readings to filter.
 * @param iFilterLen The number of elements in the array.
 * @return The median value of the array.
 */
int getMedianNum(int bArray[], int iFilterLen) {
  // Create a temporary array to store a copy of the input array.
  // This ensures the original sensor buffer is not sorted.
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++) {
    bTab[i] = bArray[i];
  }

  // Bubble sort algorithm to sort the temporary array.
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        // Swap elements if they are in the wrong order.
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }

  // Determine the median:
  // If the array length is odd, the median is the middle element.
  if ((iFilterLen & 1) > 0) { // Equivalent to iFilterLen % 2 != 0
    return bTab[(iFilterLen - 1) / 2];
  }
  // If the array length is even, the median is the average of the two middle elements.
  else {
    return (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
}

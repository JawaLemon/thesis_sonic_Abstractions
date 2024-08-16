#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

// Button pins
const int buttonPin1 = 4;
const int buttonPin2 = 5;
const int buttonPin3 = 6;
const int buttonPin4 = 7;

// Debounce variables for buttons
int lastButtonState1 = HIGH;
int lastButtonState2 = HIGH;
int lastButtonState3 = HIGH;
int lastButtonState4 = HIGH;
unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;
unsigned long lastDebounceTime3 = 0;
unsigned long lastDebounceTime4 = 0;
const unsigned long debounceDelay = 50; // Debounce delay time in milliseconds

// Smoothing filter variables
const int numReadings = 10; // Number of readings to average
int readingsX[numReadings];
int readingsY[numReadings];
int readingsZ[numReadings];
int readIndex = 0;
int totalX = 0, totalY = 0, totalZ = 0;
int averageX = 0, averageY = 0, averageZ = 0;

Adafruit_MMA8451 mma = Adafruit_MMA8451();

bool handshakeComplete = false;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Setup started");

  // Enable internal pull-up resistors
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(buttonPin3, INPUT_PULLUP);
  pinMode(buttonPin4, INPUT_PULLUP);

  Wire.begin();
  Serial.println("Initializing MMA8451 accelerometer...");
  if (!mma.begin()) {
    Serial.println("Failed to detect and initialize MMA8451!");
    while (1);
  }
  mma.setRange(MMA8451_RANGE_2_G);
  Serial.println("MMA8451 accelerometer initialized.");

  // Initialize all readings to 0
  for (int i = 0; i < numReadings; i++) {
    readingsX[i] = 0;
    readingsY[i] = 0;
    readingsZ[i] = 0;
  }

  Serial.println("Setup complete");
  
  establishContact(); // Perform handshake
  handshakeComplete = true;
}

void loop() {
  if (handshakeComplete) {
    normalOperation();
  }
}

void normalOperation() {
  int value1 = debounceButton(buttonPin1, lastButtonState1, lastDebounceTime1);
  int value2 = debounceButton(buttonPin2, lastButtonState2, lastDebounceTime2);
  int value3 = debounceButton(buttonPin3, lastButtonState3, lastDebounceTime3);
  int value4 = debounceButton(buttonPin4, lastButtonState4, lastDebounceTime4);

  // Read accelerometer data
  sensors_event_t event;
  mma.getEvent(&event);

  // Update the total to subtract the last reading
  totalX = totalX - readingsX[readIndex];
  totalY = totalY - readingsY[readIndex];
  totalZ = totalZ - readingsZ[readIndex];

  // Read the current accelerometer values
  readingsX[readIndex] = event.acceleration.x;
  readingsY[readIndex] = event.acceleration.y;
  readingsZ[readIndex] = event.acceleration.z;

  // Add the reading to the total
  totalX = totalX + readingsX[readIndex];
  totalY = totalY + readingsY[readIndex];
  totalZ = totalZ + readingsZ[readIndex];

  // Advance to the next position in the array
  readIndex = readIndex + 1;

  // If we're at the end of the array, wrap around to the beginning
  if (readIndex >= numReadings) {
    readIndex = 0;
  }

  // Calculate the average
  averageX = totalX / numReadings;
  averageY = totalY / numReadings;
  averageZ = totalZ / numReadings;

  // Print button states and smoothed accelerometer data as integers
  Serial.print(value1);
  Serial.print(",");
  Serial.print(value2);
  Serial.print(",");
  Serial.print(value3);
  Serial.print(",");
  Serial.print(value4);
  Serial.print(",");
  Serial.print(averageY); // Swapped X and Y
  Serial.print(",");
  Serial.print(averageX); // Swapped X and Y
  Serial.print(",");
  Serial.println(averageZ);

  delay(100);
}

int debounceButton(int buttonPin, int &lastButtonState, unsigned long &lastDebounceTime) {
  int reading = digitalRead(buttonPin);
  int buttonValue = lastButtonState;
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading == LOW) {
      buttonValue = 1; // Use 1 for pressed
    } else {
      buttonValue = 0; // Use 0 for released
    }
    lastButtonState = reading; // Only update the last button state after debounce time
  }
  return buttonValue;
}

void establishContact() {
  while (Serial.available() <= 0) {
    Serial.print('A'); // Send a capital A
    delay(300);
  }
}
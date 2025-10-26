/*
  Wearable Music Glove Firmware
  ---------------------------------
  This Arduino Nano 33 BLE firmware acquires data from
  five flex sensors and an onboard IMU to capture finger
  bending and wrist motion for musical interaction.

  Main functions:
   - Sample five flex sensors via analogue inputs
   - Read IMU accelerometer/gyroscope data 
   - Apply preprocessing: normalisation, filtering, rate calculation
   - Perform basic gesture recognition
   - Drive five LEDs on the glove for visual feedback
   - Package processed data and gesture results into a serial message

  Notes:
   - Serial communication: 9600 baud
   - Flex sensor calibration range: ~145–430 ADC units
   - Filtering: exponential moving average (α configurable)
   - Required library: Arduino_LSM9DS1 (install via Arduino Library Manager)

  Author: Jayson Chen
  Date: 16.8.2025
*/

#include <Arduino_LSM9DS1.h> // Required library: Arduino_LSM9DS1 (**IMPORTANT**)

// 5 Flex sensor pins and variables
int flexPins[5] = {A7, A6, A4, A2, A0};
int flexValues[5];
float flexNormalized[5];
float prevFlexNormalized[5] = {0, 0, 0, 0, 0};
float flexRates[5] = {0, 0, 0, 0, 0};
float filteredFlexNormalized[5] = {0, 0, 0, 0, 0};  // Filtered flex sensors value

// Flex sensor calibration ranges 
int flexMin[5] = {145, 145, 145, 145, 145};  // No bend values
int flexMax[5] = {430, 430, 430, 430, 430};  // Full bend values

// Timing for rate calculation
unsigned long currentTime;
unsigned long previousTime = 0;
float deltaTime;

// Rate calculation parameters
const float rateThreshold = 0.01;  // Minimum change to calculate rate
const float smoothingFactor = 0.7; // Smoothing for rate calculation (0.0-1.0)

// LED pin assignments
int ledPins[5] = {2, 4, 6, 8, 10};  

// Gesture recognize threshold
const float gestureThreshold = 0.35;  
int gesture;

// IMU variables
float accelX, accelY, accelZ;
float magX, magY, magZ;
float pitch, roll, yaw;

float filteredPitch = 0;      // Filtered pitch
float filteredRoll = 0;       // Filtered roll

// Yaw reference for relative calculation
float referenceYaw = 0;

// Pitch rate calculation variables
float prevPitch = 0;
float pitchRate = 0;
bool handWaveDetected = false;
unsigned long lastWaveTime = 0;
const float pitchRateThreshold = 800.0;  // Angle change rate threshold (degrees/second)
const unsigned long waveTimeout = 1000;  // Minimum waving interval time (ms)

// Filter coefficient (0.0-1.0, the smaller the smoother)
const float flexFilterAlpha = 0.8;    // flex sensor filter coefficient
const float imuFilterAlpha = 0.7;     // IMU filter coefficient

//Running light status management
static bool runningLightActive = false;

static unsigned long ledOffTime[5] = {0, 0, 0, 0, 0};
const unsigned long ledDelay = 500; // LED duration (ms)

// BPM control variables
int bpm = 60;  // Initial BPM value
const int bpmChangeRate = 1;  // BPM change rate
const int rollThresholdLow = -70;   // First threshold (increasing BPM)
const int rollThresholdHigh = 35;   // Second threshold (decreasing BPM)
const int bpmMin = 40;   // Minimum BPM
const int bpmMax = 200;  // Maximum BPM

void setup() {
  Serial.begin(9600);
  
  // Initialize LED pins as outputs
  for (int i = 0; i < 5; i++) {
    pinMode(ledPins[i], OUTPUT);
  }

  // Initialize IMU
  if (!IMU.begin()) {
    while (1); // Stop if IMU fails
  }
  
  // Brief delay for system stabilization
  delay(1000);
  
  // Set initial yaw reference
  initializeYawReference();

  // Brief LED test sequence
  for(int i = 0; i <= 5; i++) {
    digitalWrite(ledPins[i], HIGH);
    delay(100);
  }
  for(int i = 0; i <= 5; i++) {
    digitalWrite(ledPins[i], LOW);
    delay(100);
  }
}

void loop() {
  // Calculate time delta for rate calculation
  currentTime = millis();
  deltaTime = (currentTime - previousTime) / 1000.0; // Convert to seconds

  if (deltaTime > 0) { // Avoid division by zero
    // Read all 5 flex sensors
    for (int i = 0; i < 5; i++) {
      flexValues[i] = analogRead(flexPins[i]);
      float rawFlexNormalized = map(flexValues[i], flexMin[i], flexMax[i], 0, 1000) / 1000.0;
      rawFlexNormalized = constrain(rawFlexNormalized, 0.0, 1.0);
      
      // Applying filters to the flex sensor
      filteredFlexNormalized[i] = applyFilter(filteredFlexNormalized[i], rawFlexNormalized, flexFilterAlpha);
      
      // Use the filtered value for normalization calculation
      flexNormalized[i] = filteredFlexNormalized[i];
      
      // Calculate rate only when bending (increasing values)
      float change = flexNormalized[i] - prevFlexNormalized[i];
      
      if (change > rateThreshold) {
        // Sensor is bending - calculate rate
        float newRate = change / deltaTime;
        // Apply smoothing to reduce noise
        flexRates[i] = (smoothingFactor * flexRates[i]) + ((1.0 - smoothingFactor) * newRate);
      } else {
        // Sensor is not bending or recovering - set rate to 0
        flexRates[i] = flexRates[i] * 0.9; // Gradual decay to 0
      }
      
      // Store current value for next iteration
      prevFlexNormalized[i] = flexNormalized[i];
    }
    
    previousTime = currentTime;
  }
  
  // Detect strikes and convert to MIDI velocity
  int midiVelocity[5];
  detectStrikes(midiVelocity);

  // Read IMU data
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accelX, accelY, accelZ);
  }
  
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(magX, magY, magZ);
  }
  
  // Calculate angles
  float rawPitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
  float rawRoll = atan2(-accelX, accelZ) * 180.0 / PI;
  yaw = calculateRelativeYaw(rawPitch, rawRoll);

  // Apply filters to IMU data
  filteredPitch = applyFilter(filteredPitch, rawPitch, imuFilterAlpha);
  filteredRoll = applyFilter(filteredRoll, rawRoll, imuFilterAlpha);

  pitch = filteredPitch;
  roll = filteredRoll;

  // Recognize Gesture
  gesture = recognizeGesture();
  
  // Pitch change rate calculation
  if (!(gesture >= 1 && gesture <= 5)){
    calculatePitchRate();
  }

  // LED sequence display control
  controlLEDsWithFlexSensor();

  // BPM control
  // if (runningLightActive && !(gesture >= 1 && gesture <= 5))
  if (!(gesture >= 1 && gesture <= 5)){
    updateBPM();
  }

  // Output data: flex0 flex1 flex2 flex3 flex4 velocity0 velocity1 velocity2 velocity3 velocity4 pitch roll yaw handwave bpm
  for (int i = 0; i < 5; i++) {
    Serial.print(flexNormalized[i]);
    Serial.print(" ");
  }
  for (int i = 0; i < 5; i++) {
    Serial.print(midiVelocity[i]);
    Serial.print(" ");
  }
  Serial.print(gesture);
  Serial.print(" ");
  Serial.print(pitch, 2);
  Serial.print(" ");
  Serial.print(roll, 2);
  Serial.print(" ");
  Serial.print(yaw, 2);
  Serial.print(" ");
  Serial.print(handWaveDetected ? 1 : 0);  
  Serial.print(" ");
  Serial.println(bpm, 1);  

  delay(50); // 20Hz update rate
}

// Generic exponential moving average filter
float applyFilter(float previousValue, float newValue, float alpha) {
  // alpha = 1.0: No filtering
  // alpha = 0.0: Fully filtering 
  return alpha * newValue + (1.0 - alpha) * previousValue;
}

void detectStrikes(int midiVelocity[]) {
  static bool wasStriking[5] = {false, false, false, false, false};
  static float peakRate[5] = {0, 0, 0, 0, 0};
  static unsigned long strikeTime[5] = {0, 0, 0, 0, 0};
  static int lastVelocity[5] = {0, 0, 0, 0, 0};  // Keep the last velocity value
  
  const unsigned long strikeTimeout = 200;  // Minimum time between hits (ms)
  
  for (int i = 0; i < 5; i++) {
    midiVelocity[i] = lastVelocity[i];  
    
    bool isStriking = (flexNormalized[i] > gestureThreshold);
    unsigned long timeSinceLastStrike = currentTime - strikeTime[i];
    
    if (isStriking && !wasStriking[i] && timeSinceLastStrike > strikeTimeout) {  // Detect bending start
      // Record the bending rate at the beginning of the hit as velocity
      int velocity = map(flexRates[i] * 100, 5, 200, 1, 127);
      velocity = constrain(velocity, 1, 127);
      
      midiVelocity[i] = velocity;
      lastVelocity[i] = velocity;  
      
      wasStriking[i] = true;
      strikeTime[i] = currentTime;
    } 
    else if (!isStriking && wasStriking[i]) {  // Detect bending end
      // Keep the velocity value unchanged
      wasStriking[i] = false;
    }
  }
}

float calculateAbsoluteYaw(float pitch, float roll) {
  // Convert to radians
  float pitchRad = pitch * PI / 180.0;
  float rollRad = roll * PI / 180.0;
  
  // Tilt-compensated magnetometer readings
  float magXcomp = magX * cos(pitchRad) + magZ * sin(pitchRad);
  float magYcomp = magX * sin(rollRad) * sin(pitchRad) + 
                   magY * cos(rollRad) - 
                   magZ * sin(rollRad) * cos(pitchRad);
  
  // Calculate yaw angle
  float yaw = atan2(-magYcomp, magXcomp) * 180.0 / PI;
  
  // Normalize to 0-360 degrees
  if (yaw < 0) {
    yaw += 360;
  }
  
  return yaw;
}

float calculateRelativeYaw(float pitch, float roll) {
  float absoluteYaw = calculateAbsoluteYaw(pitch, roll);
  float relativeYaw = absoluteYaw - referenceYaw;
  
  // Normalize to -180 to +180 degrees
  while (relativeYaw > 180) {
    relativeYaw -= 360;
  }
  while (relativeYaw < -180) {
    relativeYaw += 360;
  }
  
  return relativeYaw;
}

void initializeYawReference() {
  float yawSum = 0;
  int sampleCount = 0;
  
  // Sample for 1 second
  for (int i = 0; i < 20; i++) {
    if (IMU.accelerationAvailable() && IMU.magneticFieldAvailable()) {
      IMU.readAcceleration(accelX, accelY, accelZ);
      IMU.readMagneticField(magX, magY, magZ);
      
      float pitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
      float roll = atan2(-accelX, accelZ) * 180.0 / PI;
      float currentYaw = calculateAbsoluteYaw(pitch, roll);
      
      yawSum += currentYaw;
      sampleCount++;
    }
    delay(50);
  }
  
  if (sampleCount > 0) {

    referenceYaw = yawSum / sampleCount;
    
  }
}

void calculatePitchRate() {
  static bool wasWaving = false;
  
  // Calculate the pitch change rate (degrees/second)
  if (deltaTime > 0) {
    float pitchChange = pitch - prevPitch;
    
    // Handling angle jumps
    if (pitchChange > 180) {
      pitchChange -= 360;
    } else if (pitchChange < -180) {
      pitchChange += 360;
    }
    
    pitchRate = pitchChange / deltaTime;
    prevPitch = pitch;
  }
  
  // Detecting a quick downward wave
  bool isWaving = (pitchRate < -pitchRateThreshold);
  unsigned long timeSinceLastWave = currentTime - lastWaveTime;
  
  if (isWaving && !wasWaving && timeSinceLastWave > waveTimeout) {
    // New wave detected
    handWaveDetected = true;
    lastWaveTime = currentTime;
  } else if (timeSinceLastWave > 100) {
    // Clear the waving state after 100ms
    handWaveDetected = false;
  }
  
  wasWaving = isWaving;
}

void updateBPM() {
  
    if (roll < rollThresholdLow) {
      // Roll angle is below the first threshold: increase BPM
      bpm += bpmChangeRate;
      bpm = constrain(bpm, bpmMin, bpmMax);
    } 
    else if (roll > rollThresholdHigh) {
      // Roll angle is above the second threshold: decrease BPM
      bpm -= bpmChangeRate;
      bpm = constrain(bpm, bpmMin, bpmMax);
    }
    // Roll is within the safe range: BPM remains unchanged
  
}

void controlLEDsWithFlexSensor() {
  static bool wasInLevelMode = false;
  static unsigned long animationStartTime = 0;
  static bool exitAnimationPlaying = false;
  static unsigned long gestureHoldTime = 0;
  static int lastGesture = -1;
  static bool gestureConfirmed = false;
  
  static int handWaveCount = 0;
  static bool lastHandWaveState = false;
  static unsigned long runningLightStartTime = 0;
  
  const unsigned long holdThreshold = 0;
  //const unsigned long runningLightSpeed = 200; 
  
  // Detection of hand waving trigger (rising edge detection)
  if (handWaveDetected && !lastHandWaveState) {
    handWaveCount++;
    if (handWaveCount % 2 == 1) {
      
      runningLightActive = true;
      runningLightStartTime = millis();
    } else {
      
      runningLightActive = false;
    }
  }
  lastHandWaveState = handWaveDetected;
  
  // Detect gesture changes
  if (gesture != lastGesture) {
    gestureHoldTime = millis();
    gestureConfirmed = false;
    lastGesture = gesture;
  }
  
  // Check if the gesture was held long enough
  unsigned long holdDuration = millis() - gestureHoldTime;
  if (holdDuration >= holdThreshold && !gestureConfirmed) {
    gestureConfirmed = true;
    // Confirmed the specific gesture and started the animation
    if (gesture >= 1 && gesture <= 5 && !wasInLevelMode) {
      animationStartTime = millis();
      exitAnimationPlaying = false;
    }
  }
  
  // Exit parameter mode
  if (!(gesture >= 1 && gesture <= 5) && wasInLevelMode) {
    animationStartTime = millis();
    exitAnimationPlaying = true;
    gestureConfirmed = false;
  }
  
  // Priority 1: Parameter mode (highest priority)
  if (gestureConfirmed && (gesture >= 1 && gesture <= 5)) {
    // Parameter Mode - Enter Animation
    unsigned long timeInMode = millis() - animationStartTime;
    
    for (int i = 0; i < 5; i++) {
      digitalWrite(ledPins[i], LOW);
    }
    
    if (timeInMode < 100) {
      
      digitalWrite(ledPins[0], HIGH);
      digitalWrite(ledPins[4], HIGH);
    } else if (timeInMode < 200) {
      
      digitalWrite(ledPins[1], HIGH);
      digitalWrite(ledPins[3], HIGH);
    } else if (timeInMode < 300) {
      
      digitalWrite(ledPins[2], HIGH);
    } else {
      
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(accelX, accelY, accelZ);
      }
      
      float roll = atan2(-accelX, accelZ) * 180.0 / PI;
      int ledIndex = map(roll * 10, -450, 450, 4, 0);
      ledIndex = constrain(ledIndex, 0, 4);
      
      digitalWrite(ledPins[ledIndex], HIGH);
    }
    
  } 
  // Priority 2: Exit animation
  else if (exitAnimationPlaying) {
    // Exit Animation - Play in Reverse
    unsigned long exitTime = millis() - animationStartTime;
    
    for (int i = 0; i < 5; i++) {
      digitalWrite(ledPins[i], LOW);
    }
    
    if (exitTime < 100) {
      
      digitalWrite(ledPins[2], HIGH);
    } else if (exitTime < 200) {
      
      digitalWrite(ledPins[1], HIGH);
      digitalWrite(ledPins[3], HIGH);
    } else if (exitTime < 300) {
      
      digitalWrite(ledPins[0], HIGH);
      digitalWrite(ledPins[4], HIGH);
    } else {
      
      exitAnimationPlaying = false;
    }
    
  } 
  // Priority 3: Normal mode
  else if (anyFingerBent() || anyLEDShouldStayOn()) {
    if (runningLightActive){
      handleNormalModeWithDelay();
    } else {
      // Normal mode: directly display the bent finger
      for (int i = 0; i < 5; i++) {
        if (flexNormalized[i] > gestureThreshold) {
          digitalWrite(ledPins[i], HIGH);
        } else {
          digitalWrite(ledPins[i], LOW);
        }
      }
    }
  } 
  // Priority 4: Running light (lowest priority)
  else if (runningLightActive) {
    playRunningLightAnimation(bpm);
  }
  // Priority 5: All LEDs are off
  else {
    for (int i = 0; i < 5; i++) {
      digitalWrite(ledPins[i], LOW);
    }
  }
  
  // Update parameter mode status
  wasInLevelMode = (gestureConfirmed && (gesture >= 1 && gesture <= 5));
}

// Handling normal mode with delay
void handleNormalModeWithDelay() {

  unsigned long currentTime = millis();
  
  for (int i = 0; i < 5; i++) {
    if (flexNormalized[i] > gestureThreshold) {
      // Finger bending: light up the LED and set the off time
      digitalWrite(ledPins[i], HIGH);
      ledOffTime[i] = currentTime + ledDelay;
     } else {
      // Exceeding the delay time: Turn off the LED
      digitalWrite(ledPins[i], LOW);
    }
  }
}

// Detect whether the finger is bent
bool anyFingerBent() {
  for (int i = 0; i < 5; i++) {
    if (flexNormalized[i] > gestureThreshold) {
      return true;
    }
  }
  return false;
}

// Check if any LED should remain on
bool anyLEDShouldStayOn() {

  unsigned long currentTime = millis();
  
  for (int i = 0; i < 5; i++) {
    if (flexNormalized[i] > gestureThreshold) {
      // Finger is still bent, reset the closing time
      ledOffTime[i] = currentTime + ledDelay;
      return true;
    } else if (currentTime < ledOffTime[i]) {
      // The finger has been released, but it is still within the delay period
      return true;
    }
  }
  return false;
}

// Running light animation function
void playRunningLightAnimation(int bpm) {
  static unsigned long lastUpdateTime = 0;
  static int currentLED = 0;
  static bool direction = true; // true=forward, false=reverse
  
  unsigned long currentTime = millis();
  unsigned long runningLightSpeed = 15000.0 / bpm;

  if (currentTime - lastUpdateTime >= runningLightSpeed) { // Animation speed
    // Clear all LEDs
    for (int i = 0; i < 5; i++) {
      digitalWrite(ledPins[i], LOW);
    }
    
    // Light up the current LED
    digitalWrite(ledPins[currentLED], HIGH);
    
    // Update Location
    if (direction) {
    //currentLED = (currentLED + 1) % 5;
    currentLED++;
      if (currentLED >= 5) {
        currentLED = 3; // Reach the right end, reverse
        direction = false;
      }
    } else {
      currentLED--;
      if (currentLED < 0) {
        currentLED = 1; // Arrived at the left end, forward
        direction = true;
      }
    }
    
    lastUpdateTime = currentTime;
  }
}

int recognizeGesture() {
  static int currentSingleFingerGesture = -1;    // Currently detected gestures
  static int confirmedSingleFingerGesture = -1;  // Confirmed gestures
  static unsigned long gestureStartTime = 0;     // Gesture start time
  const unsigned long gestureHoldTime = 500;    // How long the gesture needs to last
  
  // Check the bending status of each finger
  bool thumbBent = (flexNormalized[0] > 0.25);    // Thumb
  bool indexBent = (flexNormalized[1] > gestureThreshold);    // Index
  bool middleBent = (flexNormalized[2] > gestureThreshold);   // Middle
  bool ringBent = (flexNormalized[3] > gestureThreshold);     // Ring
  bool pinkyBent = (flexNormalized[4] > 0.25);    // Pinky
  
  // Count the number of bent fingers
  int bentCount = thumbBent + indexBent + middleBent + ringBent + pinkyBent;
  
  // Immediate return gesture (no 1 second confirmation required)
  if (bentCount == 0) {
    // All fingers extended - open hand
    currentSingleFingerGesture = -1;
    confirmedSingleFingerGesture = -1;
    return 0;
  }
  else if (bentCount == 5) {
    // All fingers bent - closed fist
    currentSingleFingerGesture = -1;
    confirmedSingleFingerGesture = -1;
    return 6;
  }
  
  // Determine the current specific gesture
  int instantGesture = -1;
  
  if (thumbBent && !indexBent && !middleBent && !ringBent && !pinkyBent) {
    // Thumb bent only
    instantGesture = 1;
  }
  else if (!thumbBent && indexBent && !middleBent && !ringBent && !pinkyBent) {
    // Index bent only 
    instantGesture = 2;
  }
  else if (!thumbBent && !indexBent && middleBent && !ringBent && !pinkyBent) {
    // Middle bent only
    instantGesture = 3;
  }
  else if (!thumbBent && !indexBent && !middleBent && ringBent && !pinkyBent) {
    // Ring bent only
    instantGesture = 4;
  }
  else if (!thumbBent && !indexBent && !middleBent && !ringBent && pinkyBent) {
    // Pinky bent only
    instantGesture = 5;
  }
  else if (thumbBent && !indexBent && middleBent && ringBent && pinkyBent) {
    // Make the "number 1" gesture
    instantGesture = 11;
  }
  else if (thumbBent && !indexBent && !middleBent && ringBent && pinkyBent) {
    // Make the "number 2" gesture
    instantGesture = 12;
  }
  else if (thumbBent && !indexBent && !middleBent && !ringBent && pinkyBent) {
    // Make the "number 3" gesture
    instantGesture = 13;
  }
  else if (thumbBent && indexBent && !middleBent && !ringBent && !pinkyBent) {
    // Make the "ok" gesture
    instantGesture = 14;
  }
  else {
    // Undefined gesture
    currentSingleFingerGesture = -1;
    confirmedSingleFingerGesture = -1;
    return -1;
  }
  
  // Confirm the gesture for 1 second
  if (instantGesture != currentSingleFingerGesture) {
    // The gesture has changed, restart the timer
    currentSingleFingerGesture = instantGesture;
    gestureStartTime = millis();
  }
  
  // Check if the gesture lasted long enough
  unsigned long holdDuration = millis() - gestureStartTime;
  if (holdDuration >= gestureHoldTime) {
    // The gesture lasts for more than 1 second, confirming the gesture
    confirmedSingleFingerGesture = currentSingleFingerGesture;
    return confirmedSingleFingerGesture;
  } else {
    // If the single-finger gesture duration is insufficient
    return -1;
  }
}
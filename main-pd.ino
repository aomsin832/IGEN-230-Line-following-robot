//By Aomsin Nimsombun with the help GPT 4

// Constants
const int numSensors = 5;
const int analogPins[numSensors] = {A0, A1, A2, A3, A4}; // Sensor pins
const int sonarTrig = 6; // Sonar trigger pin
const int sonarEcho = 7; // Sonar echo pin
const int motorLeftPWM = 9; // Left motor PWM pin
const int motorRightPWM = 10; // Right motor PWM pin
const int motorLeftDir = 8; // Left motor direction pin
const int motorRightDir = 11; // Right motor direction pin

// PD Controller Parameters
float Kp = 1.0; // Proportional gain
float Kd = 0.5; // Derivative gain
float previousError = 0.0;

// Other Variables
int sensorValues[numSensors] = {0};
int sensorWeights[numSensors] = {-2, -1, 0, 1, 2}; // Weight values
float linePosition = 0.0;
float error = 0.0;
float PD_output = 0.0;

void setup() {
  // Initialize pins
  pinMode(sonarTrig, OUTPUT);
  pinMode(sonarEcho, INPUT);
  pinMode(motorLeftPWM, OUTPUT);
  pinMode(motorRightPWM, OUTPUT);
  pinMode(motorLeftDir, OUTPUT);
  pinMode(motorRightDir, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // Step 1: Read Sensor Values
  for (int i = 0; i < numSensors; i++) {
    sensorValues[i] = analogRead(analogPins[i]);
  }

  // Step 2: Calculate Line Position
  float weightedSum = 0;
  float sum = 0;
  for (int i = 0; i < numSensors; i++) {
    weightedSum += sensorValues[i] * sensorWeights[i];
    sum += sensorValues[i];
  }
  linePosition = sum > 0 ? weightedSum / sum : 0; // Prevent division by zero

  // Step 3: PD Control Calculation
  error = linePosition; // Error is the current line position
  float derivative = error - previousError; // Rate of change of error
  PD_output = (Kp * error) + (Kd * derivative);
  previousError = error; // Update for next loop

  // Step 4: Obstacle Detection with Sonar
  float distance = measureDistance();
  if (distance < 10) { // Stop if obstacle detected within 10 cm
    stopMotors();
    return;
  }

  // Step 5: Motor Speed Adjustment
  int baseSpeed = 150; // Base motor speed (adjust as needed)
  int leftMotorSpeed = constrain(baseSpeed - PD_output, 0, 255);
  int rightMotorSpeed = constrain(baseSpeed + PD_output, 0, 255);

  setMotorSpeed(leftMotorSpeed, rightMotorSpeed);

  delay(10); // Small delay for stability
}

// Function to measure distance using the sonar sensor
float measureDistance() {
  digitalWrite(sonarTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(sonarTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(sonarTrig, LOW);
  long duration = pulseIn(sonarEcho, HIGH);
  float distance = duration * 0.034 / 2; // Convert to cm
  return distance;
}

// Function to set motor speeds
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  analogWrite(motorLeftPWM, leftSpeed);
  analogWrite(motorRightPWM, rightSpeed);
  digitalWrite(motorLeftDir, leftSpeed > 0 ? HIGH : LOW);
  digitalWrite(motorRightDir, rightSpeed > 0 ? HIGH : LOW);
}

// Function to stop the motors
void stopMotors() {
  analogWrite(motorLeftPWM, 0);
  analogWrite(motorRightPWM, 0);
}

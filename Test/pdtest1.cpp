const int analogInPins[] = {A0, A1, A2, A3, A4};  // Analog input pins for sensors
int sensorValue[5];  // Array to store sensor values

// Motor control pins for H-Bridge (with PWM)
const int motorLeft = 8;  // Pin for the left motor
const int motorRight = 9;  // Pin for the right motor

int lastError = 0;  // Previous error for derivative control

void setup() {
  Serial.begin(9600);

  pinMode(motorLeft, OUTPUT);
  pinMode(motorRight, OUTPUT);
}

void loop() {
  int error = 0;
  int pdTerm = 0;

  // Read the sensor values
  for (int i = 0; i < 5; i++) {
    sensorValue[i] = analogRead(analogInPins[i]);
  }

  // Calculate the error based on the sensor values
  for (int i = 0; i < 5; i++) {
    if (sensorValue[i] > 850) {  // Black
      error += (i - 2) * 1000;  // Assign weights to left (negative) and right (positive)
    } else if (sensorValue[i] < 150) {  // White
      error += 0;  // No contribution for white areas
    }
  }

  // Calculate the PD control term
  int pTerm = error;  // Proportional term
  int dTerm = error - lastError;  // Derivative term

  pdTerm = pTerm + dTerm;

  // Set motor speeds based on PD control term
  int motorSpeed = 255;  // Max speed for motors
  int leftSpeed = motorSpeed - abs(pdTerm);  // Decrease speed for turn (based on error)
  int rightSpeed = motorSpeed - abs(pdTerm);  // Decrease speed for turn (based on error)

  if (pdTerm > 0) {  // Turn right
    analogWrite(motorLeft, leftSpeed);
    analogWrite(motorRight, motorSpeed);
  } else if (pdTerm < 0) {  // Turn left
    analogWrite(motorLeft, motorSpeed);
    analogWrite(motorRight, rightSpeed);
  } else {  // Move straight
    analogWrite(motorLeft, motorSpeed);
    analogWrite(motorRight, motorSpeed);
  }

  // Update the last error for the next iteration
  lastError = error;

  // Print the sensor values and the PD control output
  Serial.print("sensor 1 = ");
  Serial.print(sensorValue[0]);
  Serial.print(" | sensor 2 = ");
  Serial.print(sensorValue[1]);
  Serial.print(" | sensor 3 = ");
  Serial.print(sensorValue[2]);
  Serial.print(" | sensor 4 = ");
  Serial.print(sensorValue[3]);
  Serial.print(" | sensor 5 = ");
  Serial.println(sensorValue[4]);

  // Wait before the next loop iteration
  delay(50);  // Short delay for motor control to settle
}

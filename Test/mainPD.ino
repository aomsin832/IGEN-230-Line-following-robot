// Function prototypes
void moveForward(int speedA, int speedB);
void stopMotors();

// Pin definitions
const int analogInPins[] = {A0, A1, A2, A3, A4};  // Analog input pins for sensors
int sensorValue[5];  // Array to store sensor values

// Motor driver pins
int in1 = 9;   // Motor A direction control
int in2 = 8;
int in3 = 11;  // Motor B direction control
int in4 = 12;
int ENA = 3;   // PWM speed control for Motor A
int ENB = 5;   // PWM speed control for Motor B

// Constants
int leftbaseSpeed = 43;  // Base speed for both motors (adjust as needed)
int rightbaseSpeed = 59;
float Kp = 35.0;           // Proportional constant for turning adjustment
int thresholdWhite = 90;  // Threshold for white detection
int thresholdBlack = 700;  // Threshold for black detection

void setup() {
  // Set motor pins as outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Initialize Serial Monitor
  Serial.begin(9600);

  // Start motors off
  stopMotors();
}

void loop() {
  // Read values from the analog input pins
  for (int i = 0; i < 5; i++) {
    sensorValue[i] = analogRead(analogInPins[i]);
  }

  // Calculate the error based on sensor values
  float error = 0.0;
  for (int i = 0; i < 5; i++) {
    if (sensorValue[i] > thresholdBlack) {  // Black detected
      error += (i - 2);  // Assign weights: Left is negative, Right is positive
    }
  }

  // Calculate motor speeds using proportional control
  float turnAdjustment = Kp * error;
  int leftMotorSpeed = leftbaseSpeed + turnAdjustment;
  int rightMotorSpeed = rightbaseSpeed - turnAdjustment;

  // Constrain motor speeds to valid PWM range (0-255)
  leftMotorSpeed = constrain(leftMotorSpeed, 5, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 5, 255);

  // Move motors based on calculated speeds
  moveForward(leftMotorSpeed, rightMotorSpeed);

  // Debugging: Print sensor values and error
  //   Serial.print("Sensor Values: ");
  //   for (int i = 0; i < 5; i++) {
  //     Serial.print(sensorValue[i]);
  //     Serial.print(" ");
  //   }
  //   Serial.print("| Error: ");
  //   Serial.println(error);
}

// Function to move forward
void moveForward(int speedA, int speedB) {
  analogWrite(ENA, speedA);  // Set speed for Motor A
  analogWrite(ENB, speedB);  // Set speed for Motor B
  digitalWrite(in1, HIGH);   // Motor A forward
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);   // Motor B forward
  digitalWrite(in4, LOW);
}

// Function to stop both motors
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

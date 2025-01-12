const int analogInPins[] = {A0, A1, A2, A3, A4};  // Analog input pins for sensors
int sensorValue[5];  // Array to store sensor values

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Read values from the analog input pins
  for (int i = 0; i < 5; i++) {
    sensorValue[i] = analogRead(analogInPins[i]);
  }

  // Calculate the error based on sensor values
  int error = 0;
  for (int i = 0; i < 5; i++) {
    if (sensorValue[i] > 850) {  // Black
      error += (i - 2) * 1000;  // Assign weights to left (negative) and right (positive)
    } else if (sensorValue[i] < 150) {  // White
      error += 0;  // No contribution for white areas
    }
  }

  // Print the sensor values to the Serial Monitor
  Serial.print("Sensor 1 = ");
  Serial.print(sensorValue[0]);
  Serial.print(" | Sensor 2 = ");
  Serial.print(sensorValue[1]);
  Serial.print(" | Sensor 3 = ");
  Serial.print(sensorValue[2]);
  Serial.print(" | Sensor 4 = ");
  Serial.print(sensorValue[3]);
  Serial.print(" | Sensor 5 = ");
  Serial.println(sensorValue[4]);

  // Determine whether to move left or right based on the error
  if (error > 0) {
    Serial.println("Move Right");
  } else if (error < 0) {
    Serial.println("Move Left");
  } else {
    Serial.println("Move Straight");
  }

  delay(700);  // Delay to make sure the sensor readings are processed properly
}

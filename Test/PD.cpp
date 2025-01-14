#include <Arduino.h>

#define sensor_num 5

#define base_speed 135
#define Kp 1.5
#define Kd 2

#define curve_speed 120
#define curve_Kp 4
#define curve_Kd 1

#define ref 50

// Motor driver pins
int in1 = 9;   // Motor A direction control
int in2 = 8;
int in3 = 11;  // Motor B direction control
int in4 = 12;
int ENA = 3;   // PWM speed control for Motor A
int ENB = 5;   // PWM speed control for Motor B

int SensorTemp[sensor_num];

// int min[sensor_num], max[sensor_num];
int min[sensor_num] = {60, 60, 60, 60, 60};
int max[sensor_num] = {750, 750, 750, 750, 750};
byte sensor_pin[sensor_num] = {A0, A1, A2, A3, A4};

void motor2F(int speedA, int speedB) {
  analogWrite(ENA, speedB);  // Set speed for Motor A
  analogWrite(ENB, speedA);  // Set speed for Motor B
  digitalWrite(in1, HIGH);   // Motor A forward
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);   // Motor B forward
  digitalWrite(in4, LOW);
}



void print_min_max() {
  Serial.print("min :");
  for (int i = 0; i < sensor_num; i++) {
    Serial.print(min[i]);
    Serial.print(",");
  }
  Serial.print("max :");
  for (int i = 0; i < sensor_num; i++) {
    Serial.print(max[i]);
    Serial.print(",");
  }
}

void print_live() {
  for (int i = 0; i < sensor_num; i++) {
    Serial.print(analogRead(sensor_pin[i]));
    Serial.print(",");
  }
  Serial.println();
}

int Sensor(int pin) {
  int temp = analogRead(sensor_pin[pin]);
  if (temp > max[pin]) {
    return 100;
  } else if (temp < min[pin]) {
    return 0;
  } else {
    return map(temp, min[pin], max[pin], 0, 100);
  }
}

void update_sensor() {
  for (int i = 0; i < sensor_num; i++) {
    SensorTemp[i] = Sensor(i);
  }
}

int last_position;

unsigned int read_line() {
  bool on_line = 0;
  unsigned long A = 0, B = 0;

  for (int i = 0; i < sensor_num; i++) {
    if (SensorTemp[i] > 50) {
      on_line = 1;
    }
    A = A + (unsigned long)SensorTemp[i] * (i * 100);
    B = B + (unsigned long)SensorTemp[i];
  }
  if (on_line) {
    return last_position = A / B;
  } else {
    if (last_position < 200) {
      return 0;
    } else {
      return 400;
    }
  }
}

int errors, previous_error, output;
void PD_Speed(byte speed, byte KP, byte KD) {
  errors = 50 - (read_line() / (sensor_num - 1));
  output = KP * errors + KD * (errors - previous_error);
  previous_error = errors;
  int m1speed = speed + output;
  int m2speed = speed - output;
  if (m1speed > 255) {
    m1speed = 255;
  } else if (m1speed < 0) {
    m1speed = 0;
  }
  if (m2speed > 255) {
    m2speed = 255;
  } else if (m2speed < 0) {
    m2speed = 0;
  }
  motor2F(m1speed, m2speed);
}




void setup() {
  Serial.begin(115200);

}
unsigned long time;

void loop() {
  update_sensor();
  Serial.println(read_line());
  PD_Speed(100,Kp,Kd);
}

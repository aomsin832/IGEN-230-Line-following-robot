#include <Arduino.h>
#include <EEPROM.h>

#define sensor_num 5

#define base_speed 135
#define Kp 1.2
#define Kd 1

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

void writeIntIntoEEPROM(int address, int number) {
  EEPROM.write(address, number >> 8);
  EEPROM.write(address + 1, number & 0xFF);
}

int readIntFromEEPROM(int address) {
  byte byte1 = EEPROM.read(address);
  byte byte2 = EEPROM.read(address + 1);
  return (byte1 << 8) + byte2;
}

void carlibrate_sensor(unsigned int t) {
  for (int i = 0; i < sensor_num; i++) {
    min[i] = 1023;
    max[i] = 0;
  }
  unsigned long time = millis();
  while (millis() - time <= t) {
    for (byte i = 0; i < sensor_num; i++) {
      int sensorval = analogRead(sensor_pin[i]);
      if (sensorval < min[i]) {
        min[i] = sensorval;
      } else if (sensorval > max[i]) {
        max[i] = sensorval;
      }
    }
  }
  for (int i = 0; i < sensor_num; i++) {
    writeIntIntoEEPROM(i * 2, min[i]);
  }
  for (int i = 0; i < sensor_num; i++) {
    writeIntIntoEEPROM((i + sensor_num) * 2, max[i]);
  }
}
void read_min_max_from_eeprom() {
  for (int i = 0; i < sensor_num; i++) {
    min[i] = readIntFromEEPROM(i * 2);
  }
  for (int i = 0; i < sensor_num; i++) {
    max[i] = readIntFromEEPROM((i + sensor_num) * 2);
  }
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

byte Sensor(int pin) {
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

bool crossing = 0;
int lineCrossed = 0;

void track_ramp_up(byte Numline,byte start_speed,byte end_speed, byte KP, byte KD) {
  // use when start prevent fronnt of robot from lifting up
  unsigned long start_time = millis();
  lineCrossed = 0;
  while (lineCrossed < Numline) {
    if (millis() - start_time > 100 && start_speed < end_speed) {
      start_speed++;
    }
    update_sensor();
    if (SensorTemp[0] > ref && SensorTemp[1] > ref && SensorTemp[2] > ref &&
        SensorTemp[3] > ref && SensorTemp[4] > ref && SensorTemp[5] > ref &&
        SensorTemp[6] > ref) {
      if (crossing == 0) {
        crossing = 1;
        lineCrossed++;
        motor2F(start_speed, start_speed);
      }
    } else {
      crossing = 0;
      PD_Speed(start_speed, KP, KD);
    }
  }
}

void track_cross_line(int Numline) {
  lineCrossed = 0;
  while (lineCrossed < Numline) {
    update_sensor();
    if (SensorTemp[0] > ref && SensorTemp[1] > ref && SensorTemp[2] > ref &&
        SensorTemp[3] > ref && SensorTemp[4] > ref && SensorTemp[5] > ref &&
        SensorTemp[6] > ref) {
      if (crossing == 0) {
        crossing = 1;
        lineCrossed++;
        motor2F(base_speed, base_speed);
      }
    } else {
      crossing = 0;
      PD_Speed(base_speed, Kp, Kd);
    }
  }
}
void track_custom_to_line(int Numline, byte speed, byte KP, byte KD) {
  lineCrossed = 0;
  while (lineCrossed < Numline) {
    update_sensor();
    if (SensorTemp[0] > ref && SensorTemp[1] > ref && SensorTemp[2] > ref &&
        SensorTemp[3] > ref && SensorTemp[4] > ref && SensorTemp[5] > ref &&
        SensorTemp[6] > ref) {
      if (crossing == 0) {
        crossing = 1;
        lineCrossed++;
      }
    } else {
      crossing = 0;
      PD_Speed(speed, KP, KD);
    }
  }
}
void track_curve() {
  lineCrossed = 0;
  while (lineCrossed == 0) {
    update_sensor();
    if (SensorTemp[0] > ref && SensorTemp[1] > ref && SensorTemp[2] > ref &&
        SensorTemp[3] > ref && SensorTemp[4] > ref && SensorTemp[5] > ref &&
        SensorTemp[6] > ref) {
      if (crossing == 0) {
        crossing = 1;
        lineCrossed++;
        motor2F(base_speed, base_speed);
      }
    } else {
      crossing = 0;
      PD_Speed(curve_speed, curve_Kp, curve_Kd);
    }
  }
}
void turn_right() {
  while (!(SensorTemp[0] < ref && SensorTemp[1] < ref && SensorTemp[2] < ref &&
           SensorTemp[3] < ref && SensorTemp[4] < ref && SensorTemp[5] < ref &&
           SensorTemp[6] < ref)) {
    update_sensor();
    motor2F(0, base_speed);
  }
  while (!(SensorTemp[0] < ref && SensorTemp[1] < ref && SensorTemp[2] < ref &&
           SensorTemp[3] < ref && SensorTemp[4] < ref && SensorTemp[5] < ref &&
           SensorTemp[6] > ref)) {
    update_sensor();
    motor2F(0, base_speed);
  }

}
void setup() {
  Serial.begin(115200);
  // read_min_max_from_eeprom();
  /*carlibrate_sensor(2000);
  print_min_max();*/
  /*motor2F(74,80);
  delay(2000);*/
  /*track_ramp_up(1, 50,base_speed,2, 0);
  track_cross_line(5);
  track_curve();
  track_cross_line(5);
  track_curve();
  track_cross_line(5);
  track_curve();
  track_cross_line(5);
  track_curve();
  track_cross_line(5);
  track_curve();
  track_cross_line(5);

  // ENTER LONG PART
  track_custom_to_line(1, base_speed, 2, 1);
  track_cross_line(5);
  track_curve();
  track_cross_line(5);
  track_curve();
  track_cross_line(5);
  track_curve();
  track_cross_line(5);
  track_curve();
  track_cross_line(5);
  track_curve();
  track_cross_line(5);
  track_custom_to_line(1, base_speed, 2, 2);
  turn_right();
  track_ramp_up(1, 80,180,1.4, 2);*/
  //turn_right();
}
unsigned long time;
void loop() {
  //motor2F(0, 0);
  /*time = millis();
  for (int i=0;i < 2000;i++){
    update_sensor();
    //Serial.println(SensorTemp[0]);
    PD_Speed();
  }
  Serial.printf("time = %d \n",millis()-time);*/
  update_sensor();
  Serial.println(read_line());
  PD_Speed(75,Kp,Kd);
  // Serial.println(read_line());
}

#include "conversions.h"
#define IN3 A5
#define IN4 A6
#define OUT4 5

int beam_angle_adc = 0;
int ball_pos_adc = 0;

int user_input = 0;
int auto_mode = 0;

void displaySensors(void){
  // print data
  // Serial.println("__________________________________");
  Serial.print("Beam Angle: ");
  Serial.print(beam_angle_adc);
  Serial.print("  |   Ball Pos: ");
  Serial.print(ball_pos_adc);
  Serial.print(",");
  Serial.print(adcToBallPosition(ball_pos_adc));
  Serial.println(" ");
}
void setup() {
  Serial.begin(115200);

  pinMode(IN3, INPUT);
  pinMode(IN4, INPUT);
  pinMode(OUT4, OUTPUT);

  analogWriteResolution(12);
  analogWriteFrequency(OUT4, 100000);
  analogWrite(OUT4, driveVoltageToDAC(0));
}

void loop() {
  // sample sensors
  // beam_angle_adc = analogRead(IN3);
  // ball_pos_adc = analogRead(IN4);

if(Serial.available() > 0){
  user_input = Serial.parseFloat();
  // flush buffer
  if(user_input == 11.0){
    Serial.println("switching modes");
    if(auto_mode){
      auto_mode = 0;
    }
    else{
      auto_mode = 1;
    }
    Serial.print("Mode: ");
    Serial.println(auto_mode);
  }

  if(auto_mode == 0){
    Serial.println(driveVoltageToDAC(user_input));
  analogWrite(OUT4, driveVoltageToDAC(user_input));
  delay(100);
  analogWrite(OUT4, driveVoltageToDAC(0));
  }

  while(Serial.available())
  {
  char t= Serial.read();
  }
Serial.flush();
}

if(auto_mode == 1){
  float voltage = 0.6;
  float delay_1 = 80;
  float delay_2 = 1000;
  analogWrite(OUT4, driveVoltageToDAC(voltage));
  delay(delay_1);
  analogWrite(OUT4, driveVoltageToDAC(0));
  delay(delay_2);
  analogWrite(OUT4, driveVoltageToDAC(-voltage));
  delay(delay_1);
  analogWrite(OUT4, driveVoltageToDAC(0));
  delay(delay_2);

  analogWrite(OUT4, driveVoltageToDAC(-voltage));
  delay(delay_1);
  analogWrite(OUT4, driveVoltageToDAC(0));
  delay(delay_2);
  analogWrite(OUT4, driveVoltageToDAC(voltage));
  delay(delay_1);
  analogWrite(OUT4, driveVoltageToDAC(0));
  delay(delay_2);
}

}

/*
  Grove- i2C motor driver demo v1.0
  by: http://www.seeedstudio.com
//  Author:LG
//  
//  
//  This demo code is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
*/
#include <Wire.h>

#define MotorSpeedSet             0x82
#define PWMFrequenceSet           0x84
#define DirectionSet              0xaa
#define MotorSetA                 0xa1
#define MotorSetB                 0xa5
#define Nothing                   0x01
#define EnableStepper             0x1a
#define UnenableStepper           0x1b
#define Stepernu                  0x1c
#define I2CMotorDriverAdd         0x0f   // Set the address of the I2CMotorDriver
// set the steps you want, if 255, the stepper will rotate continuely;
void SteperStepset(unsigned char stepnu)
{
  Wire.beginTransmission(I2CMotorDriverAdd); // transmit to device I2CMotorDriverAdd
  Wire.write(Stepernu);          // Send the stepernu command 
  Wire.write(stepnu);            // send the steps
  Wire.write(Nothing);           // send nothing   
  Wire.endTransmission();        // stop transmitting 
}
///////////////////////////////////////////////////////////////////////////////
// Enanble the i2c motor driver to drive a 4-wire stepper. the i2c motor driver will
//driver a 4-wire with 8 polarity  .
//Direction: stepper direction ; 1/0
//motor speed: defines the time interval the i2C motor driver change it output to drive the stepper
//the actul interval time is : motorspeed * 4ms. that is , when motor speed is 10, the interval time 
//would be 40 ms
//////////////////////////////////////////////////////////////////////////////////
void StepperMotorEnable(unsigned char Direction, unsigned char motorspeed)
{
  Wire.beginTransmission(I2CMotorDriverAdd); // transmit to device I2CMotorDriverAdd
  Wire.write(EnableStepper);        // set pwm header 
  Wire.write(Direction);              // send pwma 
  Wire.write(motorspeed);              // send pwmb    
  Wire.endTransmission();    // stop transmitting 
}
//function to uneanble i2C motor drive to drive the stepper.
void StepperMotorUnenable()
{
  Wire.beginTransmission(I2CMotorDriverAdd); // transmit to device I2CMotorDriverAdd
  Wire.write(UnenableStepper);        // set unenable commmand
  Wire.write(Nothing);              
  Wire.write(Nothing);              
  Wire.endTransmission();    // stop transmitting 
}
//////////////////////////////////////////////////////////////////////
//Function to set the 2 DC motor speed
//motorSpeedA : the DC motor A speed; should be 0~100;
//motorSpeedB: the DC motor B speed; should be 0~100;

void MotorSpeedSetAB(unsigned char MotorSpeedA , unsigned char MotorSpeedB)  {
  MotorSpeedA=map(MotorSpeedA,0,100,0,255);
  MotorSpeedB=map(MotorSpeedB,0,100,0,255);
  Wire.beginTransmission(I2CMotorDriverAdd); // transmit to device I2CMotorDriverAdd
  Wire.write(MotorSpeedSet);        // set pwm header 
  Wire.write(MotorSpeedA);              // send pwma 
  Wire.write(MotorSpeedB);              // send pwmb    
  Wire.endTransmission();    // stop transmitting
}
//set the prescale frequency of PWM, 0x03 default;
void MotorPWMFrequenceSet(unsigned char Frequence)  {    
  Wire.beginTransmission(I2CMotorDriverAdd); // transmit to device I2CMotorDriverAdd
  Wire.write(PWMFrequenceSet);        // set frequence header
  Wire.write(Frequence);              //  send frequence 
  Wire.write(Nothing);              //  need to send this byte as the third byte(no meaning)  
  Wire.endTransmission();    // stop transmitting
}
//set the direction of DC motor. 
void MotorDirectionSet(unsigned char Direction)  {     //  Adjust the direction of the motors 0b0000 I4 I3 I2 I1
  Wire.beginTransmission(I2CMotorDriverAdd); // transmit to device I2CMotorDriverAdd
  Wire.write(DirectionSet);        // Direction control header
  Wire.write(Direction);              // send direction control information
  Wire.write(Nothing);              // need to send this byte as the third byte(no meaning)  
  Wire.endTransmission();    // stop transmitting 
}

void MotorDriectionAndSpeedSet(unsigned char Direction,unsigned char MotorSpeedA,unsigned char MotorSpeedB)  {  //you can adjust the driection and speed together
  MotorDirectionSet(Direction);
  MotorSpeedSetAB(MotorSpeedA,MotorSpeedB);  
}

const unsigned int Near = 7;
const unsigned int CLP = 8;

void setup()  {
  Wire.begin(); // join i2c bus (address optional for master)
  delayMicroseconds(10000);
  Serial.begin(57600);
  Serial.println("setup begin");
  
  pinMode(2, INPUT);  // T1
  pinMode(3, INPUT);  // T2
  pinMode(4, INPUT);  // T3
  pinMode(5, INPUT);  // T4
  pinMode(6, INPUT);  // T5
  pinMode(Near, INPUT);  // Near
  pinMode(CLP, INPUT);  // CLP
  
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
}



/*

*/
unsigned short int initialSpeed = 30, Kp = 15, Ki = 0, Kd = 0;
float error = 0, previousError = 0;
float P, I, D, correction;

unsigned int S1, S2, S3, S4, S5;

char lastPosition = 'r';

void CalculateError() {
  
  // reset all sensors
  S1 = digitalRead(2);
  if (S1 == HIGH) lastPosition = 'l';
  S2 = digitalRead(3);
  S3 = digitalRead(4);
  S4 = digitalRead(5);
  S5 = digitalRead(6);
  if (S5 == HIGH) lastPosition = 'r';
  
  previousError = error;
  
  error = (S1*1) + (S2*2) + (S3*3) + (S4*4) + (S5*5);
  error = error / (S1+S2+S3+S4+S5);
  error -= 3;
  
  Serial.print("Error: ");
  Serial.println(error);
}

void loop()  {
  
  if (digitalRead(CLP) == LOW) {
    CalculateError();
    
    if ((S1+S2+S3+S4+S5) == 0) {
      if (lastPosition == 'l') {
        Serial.println("Full left");
        MotorSpeedSetAB(100, 100);
        delay(10);
        MotorDirectionSet(0b1001);
      } else if (lastPosition == 'r') {
        Serial.println("Full right");
        MotorSpeedSetAB(100, 100);
        delay(10);
        MotorDirectionSet(0b0110);
      }
    } else {
      P = error * Kp;
      I += error;
      I = I * Ki;
      D = error - previousError;
      
      correction = P + I + D;
      Serial.print("Correction: ");
      Serial.println(correction);
      
      MotorSpeedSetAB(initialSpeed + correction, initialSpeed - correction);
      delay(10);
      MotorDirectionSet(0b1010);
    }
  } else {
    MotorSpeedSetAB(0,0);
    delay(10);
  }
}

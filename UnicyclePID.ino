#include <Servo.h>

/* Get tilt angles on X and Y, and rotation angle on Z
 * Angles are given in degrees
 * 
 * License: MIT
 */
#include <PID_v1.h> //PID library
#include "Wire.h" //I2C
#include <MPU6050_light.h> //Gyroscope and Accelerometer library

#define OUTPUT_PIN 9 //Output pin connected to VESC

Servo VESC; //Treats the VESC like a Servo using PPM 

MPU6050 mpu(Wire); //Create MPU
double Attitude[3];
unsigned long timer = 0; //Timer

double Setpoint, Input, Output; // Create variables to adjust the loop
double Kp=2, Ki=5, Kd=1; //K values for PID loop
PID Unicycle(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); //Create the Unicycle PID

void setup() {
  Serial.begin(9600); //STart the print
  Wire.begin(); //Start the I2C
  VESC.attach(OUTPUT_PIN); //Attaches Vesc to D9
  
  byte status = mpu.begin(); //Start Init
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050

  Setpoint = 90; //The point where the PID wants to converge so it means balance

  Unicycle.SetMode(AUTOMATIC); //Turn PID on
  Unicycle.SetOutputLimits(0,180); //The PID Loop will work within the limits of the PID/Servo
  Input = 90;

  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
}

void loop() {
  mpu.update();
  
  set_attitude(mpu.getAngleX(), mpu.getAngleY(), mpu.getAngleZ());
  
  emergency_stop(Attitude[0], Attitude[2]);

  Input = Attitude[2] + 90;
  Unicycle.Compute();

  VESC.write(Output);
}

void emergency_stop(double yaw, double roll){
  if (abs(int(yaw) % 360) > 20){
    VESC.write(0);
  }
  if (abs(int(roll) % 360) > 20){
    VESC.write(0);
  }
}

void set_attitude(double yaw, double pitch, double roll){
  Attitude[0] = yaw;
  Attitude[1] = pitch;
  Attitude[2] = roll;
}

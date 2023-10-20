#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "MPU6050_tockn.h" // Include a library for the MPU6050; this is an example, ensure ESP32 compatibility
#include "PID_v1.h" // Include a library for the PID control; this is an example, ensure ESP32 compatibility

#define OUTPUT_PIN 9

Servo VESC; 
MPU6050 mpu6050(Wire); // Initialize the MPU6050 object

double Attitude[3];
unsigned long timer = 0;

double Setpoint, Input, Output;
double Kp = 2, Ki = 5, Kd = 1;
PID Unicycle(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); // Initialize the PID object

void setup() {
  Serial.begin(9600); 
  Wire.begin(); 
  VESC.attach(OUTPUT_PIN); // Attach the Servo object to the output pin
  
  mpu6050.begin(); // Initialize communication with the MPU6050
  mpu6050.calcGyroOffsets(true); // Calculate the gyro offsets

  Setpoint = 90; // The desired balance point

  Unicycle.SetMode(AUTOMATIC); // Set PID controller to automatic mode
  Unicycle.SetOutputLimits(0, 180); // Set limits for the output
}

void loop() {
  mpu6050.update(); // Update sensor values
  
  set_attitude(mpu6050.getAngleX(), mpu6050.getAngleY(), mpu6050.getAngleZ()); // Read the current angles
  
  emergency_stop(Attitude[0], Attitude[2]); // Check if emergency stop is needed

  Input = Attitude[2] + 90; // Adjust the input based on the current angle
  Unicycle.Compute(); // Compute the PID update

  VESC.write(Output); // Send the output to the servo
}

void emergency_stop(double yaw, double roll){
  if (abs(int(yaw) % 360) > 20){
    VESC.write(0); // Stop if the yaw is over 20 degrees
    while(1); // Infinite loop to stop further execution
  }
  if (abs(int(roll) % 360) > 20){
    VESC.write(0); // Stop if the roll is over 20 degrees
    while(1); // Infinite loop to stop further execution
  }
}

void set_attitude(double yaw, double pitch, double roll){
  Attitude[0] = yaw;
  Attitude[1] = pitch;
  Attitude[2] = roll;
}

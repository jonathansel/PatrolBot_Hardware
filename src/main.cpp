/* TODO
 *  1. Improve robustness by calculating checksum/CRC - Using second marker now, this might be good enough
 *  2. if (rb != startMarker) feels sketchy. Missing one packet of data? - Using second marker now, this might be good enough - ADD CRC
 *  3. Shift out old bytes per markd833?
 *  4. Angular rate required or not?
 *  5. PID Controller - implement integral term reset to avoid saturation in case of no movement for long durations. May not be a concern depending on Ki? 
 *  6. When both VESC and Roboclaw active, steering controller performance seems to decline (increased oscillations). Could be related to the noise? on TX. Digital isolator?  
 */

#define USE_USBCON  //don't think this is required

#include <Arduino.h>
#include <math.h> 
#include <ros.h>
#include "RoboClaw.h"
#include <PID_v1.h>

#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

const byte numBytes = 12; //bytes per full message is 12 i.e. AA BB 01 FF 00 00 00 00 00 06 5F CA 
byte receivedBytes[numBytes];
byte numReceived = 0;
boolean newData = false;
double abs_pos = 0;
double currentHeading = 0;
int potValue; //pin A0
int mode = 0;
float steering_angle; 

double Kp=15, Ki=0, Kd=0; //PID Gains
double Setpoint = 0, Input, Output; //PID variables - changed to float
PID PID1(&Input, &Output, &Setpoint, Kp, Kd, Ki, DIRECT); //PID Setup

RoboClaw roboclaw(&Serial3,10000);
#define address 0x80

const int switchPin = 5;

const float MIN_STEERING_ANGLE = -35;       // min steering angle (0-180)
const float MAX_STEERING_ANGLE = 35;      // max steering angle (0-180)

void ackermannCallback(const ackermann_msgs::AckermannDriveStamped & ackermann) {

  // Convert steering angle from radian to degree
  steering_angle = ackermann.drive.steering_angle * (180 / M_PI);

  // Check for allowed min steering angle
  if( steering_angle < MIN_STEERING_ANGLE )
    steering_angle = MIN_STEERING_ANGLE;
    
  // Check for allowed max steering angle
  if( steering_angle > MAX_STEERING_ANGLE )
    steering_angle = MAX_STEERING_ANGLE;

  Setpoint = steering_angle;
}

// ROS Setup
ros::NodeHandle nh;
ros::Subscriber<ackermann_msgs::AckermannDriveStamped> ackermannSubscriber("/car/mux/ackermann_cmd_mux/output", &ackermannCallback);
std_msgs::Float64MultiArray debug_data;
ros::Publisher debug_pub("teensy_core", &debug_data);

void recvBytesWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  byte startMarker = 0xAA;
  byte secondMarker = 0xBB;
  byte endMarker = 0x0D; //calculate checksum for robustness
  byte rb;
  
  while (Serial2.available() > 0 && newData == false) {
    rb = Serial2.read();
    if (recvInProgress == true) {
      if (rb != startMarker) { 
          receivedBytes[ndx] = rb;
          ndx++;
          if (ndx >= numBytes) {
              ndx = numBytes - 1;
          }
      }

      else {
          receivedBytes[ndx] = '\0'; // terminate the string
          recvInProgress = false;
          numReceived = ndx;  // save the final index to denote how much data recv
          ndx = 0;
          newData = true;
      }
    }
    
    else if (rb == startMarker) { 
        recvInProgress = true;
    }
  }
}

void showNewData() {
  if (newData == true) {
    // UNCOMMENT BELOW TO PRINT ABS SENSOR DATA
    // Serial.print("This just in (HEX values)... ");
    // for (byte n = 0; n < numReceived; n++) {
    //     Serial.print(receivedBytes[n], HEX);
    //     Serial.print(' ');
    // }

    abs_pos = (receivedBytes[8] * 256) + receivedBytes[9]; //X7 + X8 - Can zero this at anytime using serial monitor AA BB 03 68
    newData = false;
  }
}


void absToAngle() {
  // left turn side is towards 4069. Right turn side is towards 131. Less than 131 is a gltich on left turn side. 
  if(abs_pos<=65 || abs_pos>4069){ //less than 65 (inclusive) or 4069+. This only happens on left turn (i.e. -35 side)
    abs_pos = 4069;
    currentHeading = abs_pos*0.0178 - 37.3; 
  }

  if(131 <= abs_pos && abs_pos <= 4069){ //131 to 4069 inclusive equating to -35deg (right turn) and 35deg (left turn)
    currentHeading = abs_pos*0.0178 - 37.3; 
  }
}

void driveMotor(){
  // Turning to the left
  if(Output <= 0){
    roboclaw.ForwardM1(address,abs(Output)); 
  }

  // Turning to the right
  if(Output > 0){
    roboclaw.BackwardM1(address, Output);
  }
}

void checkSwitchPosition(){
  if(digitalRead(switchPin) == HIGH){
    mode = 1;
  }
  
  //Left position of switch for off (towards teensy on breadboard)
  if(digitalRead(switchPin) == LOW){
    mode = 0;
  }
}

void setup() {
  Serial2.begin(115200); //RS485 to TTL converter (Absolute Position Sensor Data)
  roboclaw.begin(115200); //Roboclaw DC Motor Controller 
  
  pinMode(switchPin, INPUT);

  PID1.SetMode(AUTOMATIC);              
  PID1.SetOutputLimits(-45, 45); //True limit is 127 but don't want to go that fast, this is around 30% duty cycle.. may need to inverse this due to swapping angles
  PID1.SetSampleTime(10); //10ms

  //vel array initialization
  char dim1_label[] = "teensy_core";
  debug_data.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  debug_data.layout.dim[0].label = dim1_label;
  debug_data.layout.dim[0].size = 4;
  debug_data.layout.dim[0].stride = 1*4; 
  debug_data.data = (float *)malloc(sizeof(float)*4); //What does this do
  debug_data.layout.dim_length = 0;
  debug_data.data_length = 4;
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();

  nh.subscribe(ackermannSubscriber);
  nh.advertise(debug_pub);
}

void loop() {
  
  recvBytesWithStartEndMarkers(); //absolute position sensor raw data
  showNewData(); //abs_pos update

  checkSwitchPosition(); //switch mode check: 0 or 1

  absToAngle(); //convert absolute position to heading angle in degrees

  if(mode == 0){ //towards teensy - null mode
    debug_data.data[0] = currentHeading; //current heading - name this angle aswell
    debug_data.data[1] = steering_angle; //desired heading
    debug_data.data[2] = abs_pos; //Raw absolute position of encoder
    debug_data.data[3] = Output; 
    debug_pub.publish(&debug_data);
  }

  if(mode == 1){
    debug_data.data[0] = currentHeading; //current heading - name this angle aswell
    debug_data.data[1] = steering_angle; //desired heading
    debug_data.data[2] = Setpoint; //Raw absolute position of encoder
    debug_data.data[3] = Output; 
    debug_pub.publish(&debug_data); //publish current heading before movement

    Input = currentHeading; 

    PID1.Compute();
    driveMotor();
  }
  nh.spinOnce(); //spin the ros node
}
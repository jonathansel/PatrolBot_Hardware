/* @todo
 *  1. Improve robustness by calculating checksum/CRC - good for when teensy has power but encoder doesnt. Getting false values at start
 *  2. if (rb != startMarker) feels sketchy. Missing one packet of data? - Using second marker now, this might be good enough - ADD CRC
 *  3. Shift out old bytes per markd833? 
 *  4. Angular rate required or not?
 *  5. PID Controller - implement integral term reset to avoid saturation in case of no movement for long durations. May not be a concern depending on Ki? 
 *  7. Benefits of using whole range of PWM
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
double abs_pos = 0; //raw position from encoder
double currentHeading = 0; //calculated current heading of robot
float steering_angle; //variable used in ackermann callback

double Kp=14, Ki=3, Kd=0; //PID Gains
double Setpoint = 0, Input = 0, Output = 0; //PID variables 
PID PID1(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); //PID Setup

RoboClaw roboclaw(&Serial3,10000); //DC Motor Controller Setup
#define address 0x80

enum SwitchStatus {
  SWITCH_OFF,
  SWITCH_ON,
};

enum SerialStatus {
  SERIAL_OFF,
  SERIAL_ON,
};

SerialStatus serialState; //how to init
bool lastSerialState = SERIAL_ON;
const unsigned long serialTimeout = 1000; // Timeout in milliseconds
unsigned long lastSerialTime = 0;

const int switchPin = 5; //for onboard switch
volatile bool modeChanged = true;
volatile SwitchStatus switchState;

const float MIN_STEERING_ANGLE = -30;       // min steering angle (0-180)
const float MAX_STEERING_ANGLE = 30;      // max steering angle (0-180)

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
    serialState = SERIAL_ON;
    lastSerialTime = millis();
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

  // Check for serial inactivity
  if (millis() - lastSerialTime > serialTimeout && serialState == SERIAL_ON) {
      serialState = SERIAL_OFF;
  }
  
  if (serialState != lastSerialState) {
    modeChanged = true;
    lastSerialState = serialState;
  }
}

void showNewData() {
  if (newData == true) {
    // UNCOMMENT BELOW TO PRINT SENSOR DATA
    // Serial.print("This just in (HEX values)... ");
    // for (byte n = 0; n < numReceived; n++) {
    //     Serial.print(receivedBytes[n], HEX);
    //     Serial.print(' ');
    // }

    abs_pos = (receivedBytes[8] * 256) + receivedBytes[9]; //X7 + X8 - Can zero this at anytime using serial monitor AA BB 03 68
    newData = false;
  }
}


// Right turn had some instance where it was requesting 35deg rather than -35deg. Must have gotten in the wrong bounds
void absToAngle() {
  // left turn side is towards 4069. Right turn side is towards 131. Less than 131 is a gltich on left turn side. 
  if(abs_pos>3943){ //less than 65 (inclusive) or 4069+. This only happens on left turn (i.e. -35 side)
    abs_pos = 3943;
  }

  if(abs_pos<12){ //less than 65 (inclusive) or 4069+. This only happens on left turn (i.e. -35 side)
    abs_pos = 12;
  }

  // if(15 <= abs_pos && abs_pos <= 3995){ //131 to 4069 inclusive equating to -35deg (right turn) and 35deg (left turn)
  //   currentHeading = abs_pos*0.0176 - 35.3; 
  // }
  currentHeading = abs_pos*0.0178 - 35.2; 
  Input = currentHeading;
}

void driveMotor(){
  // Turning to the left
  if(Output <= 0){
    roboclaw.BackwardM1(address, abs(Output));
  }

  // Turning to the right
  if(Output > 0){
    roboclaw.ForwardM1(address,abs(Output));
  }
}

//Interrupt service routine for switch
void switchPosition(){
  modeChanged = true;

  if(digitalRead(switchPin) == HIGH) {
    switchState = SWITCH_ON;
  }

  if(digitalRead(switchPin) == LOW) {
    switchState = SWITCH_OFF;
  }
}

void setup() {
  Serial2.begin(115200); //RS485 to TTL converter (Absolute Position Encoder Sensor Data)
  roboclaw.begin(115200); //Roboclaw DC Motor Controller 
  Serial.begin(115200); //Serial start for computer (ROS)

  pinMode(switchPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(switchPin), switchPosition, CHANGE);
  switchPosition(); //initalization of switch position

  PID1.SetOutputLimits(-60, 60); //True limit is 127 but don't want to go that fast, this is around 30% duty cycle.. may need to inverse this due to swapping angles
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
  absToAngle(); //convert absolute position to heading angle in degrees

  if(modeChanged) {
    modeChanged = false; //reset flag

    if(switchState == SWITCH_OFF || serialState == SERIAL_OFF){ //switch towards teensy - shutdown mode
      PID1.SetMode(MANUAL); //Set to manual i.e. turn off PID
      Output = 0;
    }

    if(switchState == SWITCH_ON && serialState == SERIAL_ON){ //switch away from teensy - regular operation   
      PID1.SetMode(AUTOMATIC); //Set to automatic, initialization occurs to ensure no bump to old 
    }
  }

  PID1.Compute(); //if mode is manual it won't modify Output var 
  driveMotor(); // if mode is manual, output overridden to 0

  debug_data.data[0] = currentHeading; //current heading - confirms encoder good
  debug_data.data[1] = steering_angle; //desired heading - confirms ackermann output good
  debug_data.data[2] = (serialState == SERIAL_ON); //state of serial port - testing
  debug_data.data[3] = (switchState == SWITCH_ON); //i.e. motor state
  debug_pub.publish(&debug_data);

  nh.spinOnce(); //spin the ros node
}
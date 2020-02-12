#include <ArduinoHardware.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <ros.h>
#include <vugc2_control/Torque_param.h>
#include <vugc2_control/Angle_param.h>
#include <std_msgs/String.h>
#include "Encoder.h"

#define outputA 13
#define outputB 12
#define index 11

// 12 bit DAC
Adafruit_MCP4725 T1;
Adafruit_MCP4725 T2;

int TRQ_CENTER = 2048; //  2.5V
int TRQ_LOWER = 0;   //  0V
int TRQ_HIGHER = 4095;  // 12 bits 5V

vugc2_control::Angle_param prevAngle;
vugc2_control::Torque_param prev;
void messageSteer(const vugc2_control::Torque_param &trq);

long angleRead;
long lastAngleRead;
Encoder encoder(outputB, outputA);

// ROS
ros::NodeHandle nh;
ros::Subscriber<vugc2_control::Torque_param> vugc2_control_torque_parameters("vugc2_control_torque_parameters", &messageSteer);
ros::Publisher<vugc2_control::Angle_encoder_param> vugc2_control_angle_parameters("vugc2_control_angle_parameters", &prevAngle);


void messageSteer(const vugc2_control::Torque_param &trq) {  
  vugc2_control::Torque_param safe;
  safe.torque1 = min(TRQ_HIGHER, max(TRQ_LOWER, trq.torque1));
  safe.torque2 = min(TRQ_HIGHER, max(TRQ_LOWER, trq.torque2));
  
  T1.setVoltage(safe.torque1, false);
  T2.setVoltage(safe.torque2, false);
  delay(50);

  // reset to center
  T1.setVoltage(TRQ_CENTER, false);
  T2.setVoltage(TRQ_CENTER, false);

  prev.torque1 = safe.torque1;
  prev.torque2 = safe.torque2;
}

void publishAngle() {
  angleRead = encoder.read();
  if (angleRead != lastAngleRead) {
    lastAngleRead = angleRead;
    
    prevAngle.angle = angleRead;
    vugc2_control_angle_parameters.publish(&prevAngle);
  }
}

void setup() {
  Serial.begin(9600);
  lastAngleRead = digitalRead(outputA);

  T1.begin(0x62);
  T2.begin(0x63);

  T1.setVoltage(TRQ_CENTER, false);
  T2.setVoltage(TRQ_CENTER, false);

  nh.initNode();
  nh.subscribe(vugc2_control_torque_parameters);
  nh.advertise(vugc2_control_angle_parameters);

  prev.torque1 = TRQ_CENTER;
  prev.torque2 = TRQ_CENTER;

}

void loop() {
  nh.spinOnce();
  publishAngle();
}

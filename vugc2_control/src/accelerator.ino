#include <ArduinoHardware.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <ros.h>
#include <vugc1_control/drive_param.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

// 12 bit DAC
Adafruit_MCP4725 T1;
Adafruit_MCP4725 T2;

int TRQ_CENTER = 2048; //  2.5V
int TRQ_LOWER = 0;   //  0V
int TRQ_HIGHER = 4095;  // 12 bits 5V

void callback(const vugc1_control::drive_param &msg);

// ROS
ros::NodeHandle nh;
ros::Subscriber<vugc1_control::drive_param> vugc1_control_drive_parameters("vugc1_control_drive_parameters", &callback);

void callback(const vugc1_control::drive_param &msg) {
  T1.setVoltage(map(msg.velocity, 0, 100, 0, 2000), false);
}


void setup() {
  T1.begin(0x62);
  T1.setVoltage(0, false);

  nh.initNode();
  nh.subscribe(vugc1_control_drive_parameters);
}

void loop() {
  nh.spinOnce();

}

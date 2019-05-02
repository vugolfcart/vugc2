#include <ArduinoHardware.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <ros.h>
#include <vugc2_control/Steering.h>
#include <std_msgs/String.h>

// 12 bit DAC
Adafruit_MCP4725 T1;
Adafruit_MCP4725 T2;

int TRQ_CENTER = 2048; //  2.5V
int TRQ_LOWER = 0;   //  0V
int TRQ_HIGHER = 4095;  // 12 bits 5V

vugc2_control::Steering prev;
void messageSteer(const vugc2_control::Steering &trq);

// ROS
ros::NodeHandle nh;
ros::Subscriber<vugc2_control::Steering> vugc1_control_torque_parameters("vugc2_control_torque_parameters", &messageSteer);


void messageSteer(const vugc2_control::Steering &trq) {  
  vugc2_control::Steering safe;
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

void setup() {
  Serial.begin(9600);
  T1.begin(0x62);
  T2.begin(0x63);

  T1.setVoltage(TRQ_CENTER, false);
  T2.setVoltage(TRQ_CENTER, false);

  nh.initNode();
  nh.subscribe(vugc1_control_torque_parameters);

  prev.torque1 = TRQ_CENTER;
  prev.torque2 = TRQ_CENTER;

}

void loop() {
  nh.spinOnce();
}

#include <ros.h>
#include <Arduino.h>
#include <std_srvs/SetBool.h>

const int POWER_PIN = 2;
ros::NodeHandle nh;
void callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response> service("set_power_status", &callback);

void callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  if (req.data == 0) {
    digitalWrite(POWER_PIN, LOW);
    res.success = true;
    res.message = "Power status set to low.";
  }
  else if (req.data == 1) {
    digitalWrite(POWER_PIN, HIGH);
    res.success = true;
    res.message = "Power status set to high.";
  }  
}

void setup() {
  nh.initNode();
  nh.advertiseService(service);
  pinMode(POWER_PIN, OUTPUT);
}

void loop() {
  nh.spinOnce();
  delay(1000);
}

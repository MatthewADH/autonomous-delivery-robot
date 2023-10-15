#include "mbed.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <math.h>
#include <string>

#include "PID.h"


const static double D = 0.4;  // m, distance between left and right wheels
const static float SAMPLE_TIME = 1/50.0;
const static float HZ2MPS = 1e-4;  // Scaling factor for tachometer, Hz/mps
const static float KP = 1.0;
const static float KI = 1.0;
const static float KD = 0.0;
const static float MAX_REVERSABLE = 0.05;

ros::NodeHandle nh;
DigitalOut greenLed(LED_GREEN);
DigitalOut redLed(LED_RED);

Thread eventThread;
EventQueue eventQueue;
Ticker ticker;

PID leftPID(PTD1, HZ2MPS, PTC9, KP, KI, KD, SAMPLE_TIME);
PID rightPID(PTD2, HZ2MPS, PTA13, KP, KI, KD, SAMPLE_TIME);

DigitalOut leftForward(PTC8);
DigitalOut leftReverse(PTA5);

DigitalOut rightForward(PTD5);
DigitalOut rightReverse(PTD0);

double vl;  // m/s, desired velocity for left  wheels
double vr;  // m/s, desired velocity for right wheels

std_msgs::String out_msg;
ros::Publisher pubOut("mbed_out", &out_msg);


void pub(const char* str) {
    out_msg.data = str;
    pubOut.publish(&out_msg);
}

void velocityCallback(const geometry_msgs::Twist& msg){
  // Calculate desired left and right wheel velocity
  vl = msg.linear.x - D * msg.angular.z / 2;
  vr = msg.linear.x + D * msg.angular.z / 2;

  std::string str = "Vl = ";
  str += std::to_string(vl);
  str += ", Vr = ";
  str += std::to_string(vr);
  pub(str.data());
}

ros::Subscriber<geometry_msgs::Twist> velocitySub("cmd_vel", &velocityCallback);

void tuneCallback(const geometry_msgs::Point& msg) {
  leftPID.setGains(msg.x, msg.y, msg.z);
  rightPID.setGains(msg.x, msg.y, msg.z);

  std::string str = "Kp = ";
  str += std::to_string(msg.x);
  str += ", Ki = ";
  str += std::to_string(msg.y);
  str += ", Kd = ";
  str += std::to_string(msg.z);
  pub(str.data());
}

ros::Subscriber<geometry_msgs::Point> tuneSub("tune", &tuneCallback);

void tickerCallback() {
  if (vl != 0 && leftForward != (vl > 0)) {    // if motor direction needs to flip 
    if 	(leftPID < MAX_REVERSABLE) {
      leftReverse = vl < 0;
      leftForward = !leftReverse;
      leftPID = fabs(vl);
    }
    else
      leftPID = 0;
  }
  else
    leftPID = fabs(vl);
  if (vr != 0 && rightForward != (vr > 0)) {    // if motor direction needs to flip 
    if 	(rightPID < MAX_REVERSABLE) {
      rightReverse = vr < 0;
      rightForward = !rightReverse;
      rightPID = fabs(vr);
    }
    else
      rightPID = 0;
  }
  else
    rightPID = fabs(vr);

  leftPID.step();
  rightPID.step();
}

int main() {
    nh.initNode();
    nh.subscribe(velocitySub);
    nh.subscribe(tuneSub);
    nh.advertise(pubOut);

    eventThread.start(
          Callback<void()>(&eventQueue, &EventQueue::dispatch_forever));
    ticker.attach(
          eventQueue.event(Callback<void()>(&tickerCallback)), SAMPLE_TIME);


    while (1) {
        nh.spinOnce();
        redLed = nh.connected();
        wait_ms(20);
    }
}


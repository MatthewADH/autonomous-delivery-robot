#include "mbed.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <math.h>

#include "PID.h"


const static double D = 0.3;  // m, distance between left and right wheels
const static float SAMPLE_TIME = 1/50.0;
const static float KP = 1.0;
const static float KI = 1.0;
const static float KD = 0.0;

ros::NodeHandle nh;
DigitalOut statusLight(LED_GREEN);

Thread eventThread;
EventQueue eventQueue;
Ticker ticker;

PID leftPID(PTB0, PTB1, KP, KI, KD, SAMPLE_TIME);
PID rightPID(PTB3, PTC2, KP, KI, KD, SAMPLE_TIME);
DigitalOut leftReverse(PTB2);
DigitalOut rightReverse(PTC1);

double vl;  // m/s, desired velocity for left  wheels
double vr;  // m/s, desired velocity for right wheels


geometry_msgs::Point out_msg;
ros::Publisher pubOut("mbed_out", &out_msg);


void pub(double x, double y, double z) {
    out_msg.x = x;
    out_msg.y = y;
    out_msg.z = z;
    pubOut.publish(&out_msg);
}

void velocityCallback(const geometry_msgs::Twist& msg){
  // Calculate desired left and right wheel velocity
  vl = msg.linear.x - D * msg.angular.z / 2;
  vr = msg.linear.x + D * msg.angular.z / 2;

  // Set direction of motors
  leftReverse = vl < 0;
  rightReverse = vr < 0;

  // Set speed of motors
  leftPID = fabs(vl);
  rightPID = fabs(vr);

  pub(vl, vr, 0);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &velocityCallback);

void tickerCallback() {
  leftPID.step();
  rightPID.step();
}

int main() {
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(pubOut);

    eventThread.start(
          Callback<void()>(&eventQueue, &EventQueue::dispatch_forever));
    ticker.attach(
          eventQueue.event(Callback<void()>(&tickerCallback)), SAMPLE_TIME);


    while (1) {
        nh.spinOnce();
        wait_ms(1);
    }
}


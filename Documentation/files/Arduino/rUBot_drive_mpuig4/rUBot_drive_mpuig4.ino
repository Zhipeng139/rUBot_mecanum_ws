// Program rUBot mecanum
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

#include "encoder.h"
#include "kinematics.hpp"
#include "motor.h"
#include "pid.hpp"


// Global Variables
float ctrlrate=1.0;
unsigned long lastctrl;
float x=0,y=0,theta=0;
float vx=0,vy=0,w=0;
float vxi=0,vyi=0,wi=0;
float wA=0,wB=0,wC=0,wD=0;
float pwma=0,pwmb=0,pwmc=0,pwmd=0;

// Motors PID
float KP=0.3,KI=0.2,KD=0.2;
MPID PIDA(encA,KP,KI,KD,true);
MPID PIDB(encB,KP,KI,KD,false);
MPID PIDC(encC,KP,KI,KD,true);
MPID PIDD(encD,KP,KI,KD,false);


void cmdVelCb( const geometry_msgs::Twist& twist_msg){
  vx=twist_msg.linear.x;
  vy=twist_msg.linear.y;
  w=twist_msg.angular.z;

  lastctrl=millis();
}

void resetCb(const std_msgs::Bool& reset){
  if(reset.data){
    x=0.0;y=0.0;theta=0.0;
  }
}

ros::NodeHandle nh;

tf::TransformBroadcaster broadcaster;
geometry_msgs::TransformStamped t;
geometry_msgs::Twist twist;
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmdVelCb );
ros::Subscriber<std_msgs::Bool> resub("reset_odom", resetCb );


void setup(){
  IO_init();
  PIDA.init();
  PIDB.init();
  PIDC.init();
  PIDD.init();

  nh.initNode();
  broadcaster.init(nh);
  nh.subscribe(sub);
  nh.subscribe(resub);
  nh.advertise(odom_pub);
  lastctrl=millis();
}

void loop(){
  delay(10);//very important to slow down the loop!

  InverseKinematic(vx, vy, w, pwma, pwmb, pwmc, pwmd);
  PIDA.tic();
  MotorA(PIDA.getPWM(pwma));
  wA=PIDA.getWheelRotatialSpeed();
  PIDA.toc();

  PIDB.tic();
  MotorB(PIDB.getPWM(pwmb));
  wB=PIDB.getWheelRotatialSpeed();
  PIDB.toc();

  PIDC.tic();
  MotorC(PIDC.getPWM(pwmc));
  wC=PIDC.getWheelRotatialSpeed();
  PIDC.toc();

  PIDD.tic();
  MotorD(PIDD.getPWM(pwmd));
  wD=PIDD.getWheelRotatialSpeed();
  PIDD.toc();

  ForwardKinematic(wA,wB,wC,wD,vxi,vyi,wi);//compute the real PID twist value
  float dt=PIDA.getDeltaT();
  theta+=wi*dt;
  if(theta > 6.28)
    theta=0;//1 turn positive angle 0-360
  x+=vxi*cos(theta)*dt-vyi*sin(theta)*dt;
  y+=vxi*sin(theta)*dt+vyi*cos(theta)*dt;
  
    t.header.stamp = nh.now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_footprint";
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.rotation = tf::createQuaternionFromYaw(theta);//theta
    broadcaster.sendTransform(t);
    
    odom.header.stamp = nh.now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation =tf::createQuaternionFromYaw(theta);//theta
    odom.twist.twist.linear.x = vxi;
    odom.twist.twist.linear.y = vyi;
    odom.twist.twist.angular.z = wi;
    odom_pub.publish(&odom);
 
  if((millis()-lastctrl)>1000*ctrlrate){
    STOP();//Stops the 4 motors if there is no Twist message receives in 1second
    vx=0;
    vy=0;
    w=0;
    vxi=0;
    vyi=0;
    wi=0;
  }
  nh.spinOnce();// Process incoming ROS messages
}

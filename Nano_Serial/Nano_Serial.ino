
#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <SoftwareSerial.h>

SoftwareSerial pin(3, 2); 

ros::NodeHandle nh;

double a=0, l=0;
void messageCb( const geometry_msgs::Twist& msg){
  a = msg.angular.z;
  l = msg.linear.x;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );


void setup(){

 pin.begin(9600);
 Serial.begin(9600);
 nh.getHardware()->setBaud(9600);
 
 
 nh.initNode();
 nh.subscribe(sub);

 pin.write('d');
 pin.write('\r');

}
void loop(){


 if(a==0 && l>0)
   pin.write('a');

 else if(a<0 && l>0)
   pin.write('b');

 else if(a>0 && l>0)
   pin.write('c');

 else 
    pin.write('d');
    
 pin.write('\r');
 nh.spinOnce();

}

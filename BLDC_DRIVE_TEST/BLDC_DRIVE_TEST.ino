#include <ModbusMaster.h>
#include <SoftwareSerial.h>

#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#define MODBUS_DATA_TRANSACTION_PIN_LEFT A0   //DE & DE Pin of MAX485
#define MODBUS_DATA_TRANSACTION_PIN_RIGHT A2
#define VOLTAGE_READING_PIN A3
//Constant Setup Values
short leftMotorSpeed = 400;
short rightMotorSpeed = 400;
short spinSpeed = 400;
short batteryLow=30;

//double w_r=0, w_l=0;
////wheel_rad is the wheel radius ,wheel_sep is
//double wheel_rad = 0.15, wheel_sep = 0.3;

ros::NodeHandle nh;

double speed_ang=0, speed_lin=111.0;

void messageCb( const geometry_msgs::Twist& msg){
  nh.loginfo("Callback function entered");
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  //speed_lin = 123.0;
//  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
//  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
   /*
   if(speed_lin>0 && speed_ang==0)
     ForwardTheBot();
   else if(speed_lin>0 && speed_ang<0)
   SpinRightTheBot();

   else if(speed_lin>0 && speed_ang>0)
   SpinLeftTheBot();

   else 
     StopTheBot();
   */
   


}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

/*
std_msgs::Float64 lin_vel;
ros::Publisher lin_velPub("linvel", &lin_vel);

std_msgs::Float64 ang_vel;
ros::Publisher ang_velPub("angvel", &ang_vel);
*/
uint8_t LeftMotorSlaveId = 1;   //Slave ID of LEFT Drive
uint8_t RightMotorSlaveId = 1;   //Slave ID of RIGHT Drive
ModbusMaster modbusMasterLeftMotorNode;                     
ModbusMaster modbusMasterRightMotorNode;
SoftwareSerial Max485LeftMotorSerial(3, 2);   //Serial Port(Rx,Tx) Connected with MAX 485 Module on Left Motor
SoftwareSerial Max485RightMotorSerial(5, 4);  //Serial Port(Rx,Tx) Connected with MAX 485 Module on Right Motor

//Working Variables
int runningSpeed = 0;

void setup() 
{
  //Serial.begin(9600);
  //while(!Serial.available());
  
  // put your setup code here, to run once:
  pinMode(MODBUS_DATA_TRANSACTION_PIN_LEFT, OUTPUT);
  pinMode(MODBUS_DATA_TRANSACTION_PIN_RIGHT, OUTPUT);
  //Setting BaudRate 
  Max485LeftMotorSerial.begin(9600);   //Modbus Baud rate is 9600 8N1
  Max485RightMotorSerial.begin(9600);   //Modbus Baud rate is 9600 8N1
  
  
  modbusMasterLeftMotorNode.begin(Max485LeftMotorSerial);
  modbusMasterLeftMotorNode.preTransmission(preTransmission);
  modbusMasterLeftMotorNode.postTransmission(postTransmission);
  
  modbusMasterRightMotorNode.begin(Max485RightMotorSerial);
  modbusMasterRightMotorNode.preTransmission(preTransmission);
  modbusMasterRightMotorNode.postTransmission(postTransmission);

  StopTheBot();
  
  nh.getHardware()->setBaud(19200);
  
  nh.initNode();
  nh.subscribe(sub);
  /*
  nh.advertise(lin_velPub);
  nh.advertise(ang_velPub);
  */
}

void loop() 
{
//  double p1= w_l*10;
//  double p2= w_r*10;
 ForwardTheBot();
/*
if(Serial.available()>0)
 {
  byte a=Serial.read();
  Serial.read();
  //Serial.println(a);

 
  if(speed_lin>0 && speed_ang==0)
   ForwardTheBot();

 else if(speed_lin<0 && speed_ang==0)
   ReverseTheBot();

 else if(speed_lin>=0 && speed_ang<0)
   SpinRightTheBot();

 else if(speed_lin>=0 && speed_ang>0)
   SpinLeftTheBot();

 else 
    StopTheBot();
/*
 lin_vel.data=speed_lin;
 ang_vel.data=speed_ang;
 lin_velPub.publish( &lin_vel );
 ang_velPub.publish( &ang_vel );
 */
 nh.spinOnce();

}


void StopTheBot()
{
  modbusMasterLeftMotorNode.Run_Motor(LeftMotorSlaveId, 515); // CCW-521, stop-512, brake-515, CW-513
  modbusMasterRightMotorNode.Run_Motor(RightMotorSlaveId, 515); // CCW-521, stop-512, brake-515, CW-513
  
}
void ReverseTheBot()
{
  //Serial.println("Forward");
  setMotorSpeed(leftMotorSpeed, rightMotorSpeed);
  modbusMasterLeftMotorNode.Run_Motor(LeftMotorSlaveId, 513); // CCW-521, stop-512, brake-515, CW-513
  //delay(1);
  modbusMasterRightMotorNode.Run_Motor(RightMotorSlaveId, 521); // CCW-521, stop-512, brake-515, CW-513
  //delay(1);

  
}
void ForwardTheBot()
{
  //Serial.println("Backward");
  modbusMasterLeftMotorNode.Run_Motor(LeftMotorSlaveId, 521); // CCW-521, stop-512, brake-515, CW-513
  //delay(1);
  modbusMasterRightMotorNode.Run_Motor(RightMotorSlaveId, 513); // CCW-521, stop-512, brake-515, CW-513
  //delay(1);
 
  setMotorSpeed(leftMotorSpeed, rightMotorSpeed);
  
}
void TurnRightTheBot()
{
  //Serial.println("TurnRight");
  setMotorSpeed(leftMotorSpeed, rightMotorSpeed);
  modbusMasterLeftMotorNode.Run_Motor(LeftMotorSlaveId, 512); // CCW-521, stop-512, brake-515, CW-513
  //delay(1);
  modbusMasterRightMotorNode.Run_Motor(RightMotorSlaveId, 521); // CCW-521, stop-512, brake-515, CW-513
  //delay(1);
 
  
}
void TurnLeftTheBot()
{
  //Serial.println("TurnLeft");
  modbusMasterLeftMotorNode.Run_Motor(LeftMotorSlaveId, 521); // CCW-521, stop-512, brake-515, CW-513
  //delay(1);
  modbusMasterRightMotorNode.Run_Motor(RightMotorSlaveId, 512); // CCW-521, stop-512, brake-515, CW-513
  //delay(1);
  setMotorSpeed(leftMotorSpeed, rightMotorSpeed);
  
}
void spinRobot(int m, int n)
{
  //Serial.println("Spin");
  modbusMasterLeftMotorNode.Run_Motor(LeftMotorSlaveId, m); // CCW-521, stop-512, brake-515, CW-513
  //delay(1);
  modbusMasterRightMotorNode.Run_Motor(RightMotorSlaveId, n); // CCW-521, stop-512, brake-515, CW-513
  //delay(1);
  setMotorSpeed(leftMotorSpeed, rightMotorSpeed);
}
void SpinRightTheBot()
{
   //Serial.println("SpinRight");
   spinRobot(513, 513);
  
}
void SpinLeftTheBot()
{
  //Serial.println("SpinLeft");
  spinRobot(521, 521);
}


void setMotorSpeed(int leftMotorRPM, int rightMotorRPM)
{
  modbusMasterLeftMotorNode.Set_Speed(LeftMotorSlaveId, leftMotorSpeed  ); // Set Speed, speed range is (0-1000 RPM)
  //delay(1);
  modbusMasterRightMotorNode.Set_Speed(RightMotorSlaveId, rightMotorSpeed ); // Set Speed, speed range is (0-1000 RPM)
   //delay(1);
  
}

void preTransmission() {
  digitalWrite(MODBUS_DATA_TRANSACTION_PIN_LEFT, 1);
  digitalWrite(MODBUS_DATA_TRANSACTION_PIN_RIGHT, 1);
}
void postTransmission() {
  digitalWrite(MODBUS_DATA_TRANSACTION_PIN_LEFT, 0);
  digitalWrite(MODBUS_DATA_TRANSACTION_PIN_RIGHT, 0);
}

bool GetBatteryVoltage() {
  // read the input on analog pin 0:
  float voltage;
  int voltageDigital = analogRead(VOLTAGE_READING_PIN);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5.0V):
  voltage = (voltageDigital * (5.0 / 1024.0) ) / 0.09;
  voltage = voltage - 2; // add on for calibration only for AB2
  float batteryPercentage = (voltage - 19.2) / 0.074; // full charge 26.6V
  if (voltage < 19.2)
  {
    //Serial.print("LOW BATTERY : " + String(voltage) + "V, " + String(batteryPercentage) + "%");
    return false;
    //Serial.print("LOW BATTERY : " + String(voltage) + "V, " + String(batteryPercentage) + "%");
  }
  else 
  {
    //Serial.print("Battery : " + String(voltage) + "V, " + String(batteryPercentage) + "%");
    return true;
    //Serial.print("Battery : " + String(voltage) + "V, " + String(batteryPercentage) + "%");
  }
}

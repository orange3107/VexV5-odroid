#include <Vex5.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

int32_t millRequest1;
int32_t millRequest2;

int32_t time_millis = 0;

Vex5_Motor motor1;
Vex5_Motor motor2;
int16_t goalSpeed = 504;
int16_t realSpeed1 = 0;
int16_t realSpeed2 = 0;
int32_t realPosition = 0;

double* realRobotVel;
float realWheelVel[2] = {0.0, 0.0};

double* goalWheelVel;
double* goalRobotVel;


double X, Y, A;
int count = 1;


void cmd_cb(const geometry_msgs::Twist& vel){
  
  goalRobotVel[0] = vel.linear.x;
  goalRobotVel[1] =vel.angular.z;

  goalWheelVel[0] = (2 * goalRobotVel[0] - 0.294 * goalRobotVel[1]) / (2 * 0.052);
  goalWheelVel[1] = (2 * goalRobotVel[0] + 0.294 * goalRobotVel[1]) / (2 * 0.052);

  motor1.setSpeed((goalWheelVel[0]*60*14)/(2*3.14));
  motor2.setSpeed(-(goalWheelVel[1]*60*14)/(2*3.14));
}

ros::NodeHandle nh;
sensor_msgs::JointState joint_state;
nav_msgs::Odometry odom;
ros::Subscriber<geometry_msgs::Twist> sub_cmd("cmd_vel", cmd_cb);
ros::Publisher joint_state_pub("joint_state", &joint_state);
ros::Publisher odom_pub("odom", &odom);

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode(); //инициализируем ноду
  nh.advertise(joint_state_pub);//инициализируем паблишеры и субскрайберы
  nh.advertise(odom_pub);
  nh.subscribe(sub_cmd);
  
  millRequest1 = millis();
  millRequest2 = millis();
  
  Vex5.begin();
  motor1.begin(VEX5_PORT_1);
  motor2.begin(VEX5_PORT_6);
  
  goalWheelVel = new double[2];
  goalRobotVel = new double[2];
  
}

void loop() {

  millRequest1 = millis() - time_millis;
 
  if (millRequest1 > 15)
    {
           
  time_millis = millis();

  nh.spinOnce();
  
  motor1.getSpeed(realSpeed1);
  realWheelVel[0] = (2*realSpeed1/14*3.14)/60;
  
  motor2.getSpeed(realSpeed2);
  realWheelVel[1] = -(2*realSpeed2/14*3.14)/60;

  motor1.setSpeed((goalWheelVel[0]*60*14)/(2*3.14));
  motor2.setSpeed(-(goalWheelVel[1]*60*14)/(2*3.14));
  
  calcOdometry();
  
  joint_state.header.stamp = nh.now();
  joint_state.velocity = realWheelVel;
  joint_state_pub.publish(&joint_state); 

  odom.pose.pose.position.x = X;
  odom.pose.pose.position.y = Y;
  odom.pose.pose.orientation.z = sin(A/2);
  odom.pose.pose.orientation.w = cos(A/2);
  odom.twist.twist.linear.x = realRobotVel[0];
  odom.twist.twist.angular.z = realRobotVel[1];
  
  odom_pub.publish(&odom);
    }
    
    
}

void calcOdometry()
 {
   realRobotVel[0] = (0.052 * (realWheelVel[0] + realWheelVel[1])) / 2;
   realRobotVel[1] = (0.052 * (realWheelVel[1] - realWheelVel[0])) / 0.294;

// 5,2 см - радиус колёс
// 29,4 - между колёсами

   double delta_s = realRobotVel[0] * millRequest1/1000;
   double delta_a = realRobotVel[1] * millRequest1/1000;

   A += delta_a;
   X += (delta_s * cos(A));
   Y += (delta_s * sin(A));
   
 }

void Serialprint(){
  Serial.println(X);
  Serial.println(Y);
  Serial.println(A);
  
  
  Serial.println(realWheelVel[0]);
  Serial.println(realWheelVel[1]);

  Serial.println(realRobotVel[0]);
  Serial.println(realRobotVel[1]);
  
  Serial.println("");
}

void setGoalRobotVel(double *goalRobotVel, double *goalWheelVel)
 {    
     goalWheelVel[0] = (2 * goalRobotVel[0] - 0.294 * goalRobotVel[1]) / (2 * 0.052);
     goalWheelVel[1] = (2 * goalRobotVel[0] + 0.294 * goalRobotVel[1]) / (2 * 0.052);
 }
 

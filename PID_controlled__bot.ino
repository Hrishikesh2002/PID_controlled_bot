#include "utils.h"
#include <PID_v1.h>
//#if TCP
//#include "ArduinoTcpHardware.h"
//#else
//#include "ArduinoHardware.h"
//#endif
//
//
//#include "geometry_msgs/Twist.h"
//#include <ros/time.h>
//#include <tf/tf.h>
//#include <tf/transform_broadcaster.h>
//#include <nav_msgs/Odometry.h>
//#include <geometry_msgs/Vector3.h>
//
//
////initializing handle
//
//ros::Nodehandle nh;
//geometry_msgs::TransformStamped t;
//nav_msgs::Odometry odom_msg;
//ros::Publisher odom_pub("odom", &odom_msg);
//tf::TransformBroadcaster broadcaster;
//
////tf variables to be
//char base_link[] = "/base_link";
//char odom[] = "/odom";
//
////position and velocity variables read from the ODrive
//
//long posR;
//long posL;
//
////variables to work out the difference in each cycle
//
//long posR_old;
//long posL_old;
//long posR_diff;
//long posL_diff;
//float posR_mm_diff;
//float posL_mm_diff;
//float pos_average_mm_diff;
//float pos_total_mm; broadcast
//
//double x = 0;
//double y = 0;
//double theta = 0;
//
//
//char base_link[] = "/base_link";
//char odom[] = "/odom";
//
////position and velocity variables read from the ODrive
//
//long posR;
//long posL;
//
////variables to work out the difference in each cycle
//
//long posR_old;
//long posL_old;
//long posR_diff;
//long posL_diff;
//float posR_mm_diff;
//float posL_mm_diff;
//float pos_average_mm_diff;
//float pos_total_mm;
//


double SetRPS_R = 3;
double SetRPS_L = 3;
double Kp = 1.2; double Ki = 9.5 ; double Kd = 0;
// The values of Kp, Ki, Kd for rpm are 1,1,0 (for 100 ms timer)
double ctrl_speed = 0;

double input_L = 0;
double input_R = 0;
double output_L = 0;
double output_R = 0;

PID PID_R(&input_R, &output_R, &SetRPS_R, Kp, Ki, Kd, DIRECT);
PID PID_L(&input_L, &output_L, &SetRPS_L, Kp, Ki, Kd, DIRECT);


void setup() {
  PID_R.SetMode(AUTOMATIC);
  PID_L.SetMode(AUTOMATIC);
  
  SetPins();

  SetTimer();

  Serial.begin(9600);
}

void loop() {


    input_R = rps_R;
    input_L = rps_L;

    PID_R.Compute();
    PID_L.Compute();

    driveMotor(LEFT, output_L, ClkWise);
    driveMotor(RIGHT, output_R, ClkWise);

    
//
//    Serial.print("error right: ");
//    Serial.println((rps_R-SetRPS_R)/SetRPS_R);

      Serial.println(distance_travelled);   

//    Serial.print("rpm Right: ");
//    Serial.println(rpm_R);
//    Serial.print("rpm Left: ");
//    Serial.println(rpm_L);


}

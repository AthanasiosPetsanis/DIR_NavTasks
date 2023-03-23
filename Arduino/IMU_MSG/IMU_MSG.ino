#include <FreeSixIMU.h>

#include <Wire.h>

#include <ros.h>
#include <sensor_msgs/Imu.h>


float values[9]; // angles in degrees

// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();

ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu_data", &imu_msg);


void setup() { 
  // total_w[0] = 0; total_w[1] = 0; total_w[2] = 0;


  Serial.begin(57600);
  nh.initNode();
  nh.advertise(imu_pub);
  Wire.begin();
  
  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);
}

void loop() { 
  // Test

  // Read Angles
  sixDOF.getValues(values);
  // angle_x = angles[1];
  // angle_y = angles[2];
  // angle_z = angles[0];


  // Publish
  // imu_msg.header.stamp = nh.now();
  // imu_msg.orientation.x = values[1];
  // imu_msg.orientation.y = values[2];
  // imu_msg.orientation.z = values[0];
  // imu_msg.orientation.w = event.orientation.w;

  // imu_msg.angular_velocity.x = w[0];
  // imu_msg.angular_velocity.y = w[1];
  // imu_msg.angular_velocity.z = w[2];

  // imu_msg.linear_acceleration.x = values[0];
  // imu_msg.linear_acceleration.y = values[1];
  // imu_msg.linear_acceleration.z = values[2];

  imu_pub.publish(&imu_msg);
  nh.spinOnce();

  delay(1); 
}


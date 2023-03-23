#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>

#include <Wire.h>

#include <ros.h>
#include <geometry_msgs/Vector3.h>

// float Q_angles[3]; // angles in quaternion
float angles[3]; // angles in degrees
int int_accel[3]; 
float accel[3]; // acceleration in xyz (including graivity) in g


// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();

ros::NodeHandle nh;
geometry_msgs::Vector3 angle_msg;
geometry_msgs::Vector3 accel_msg;
ros::Publisher angle_pub("angle", &angle_msg);
ros::Publisher accel_pub("acceleration", &accel_msg);

void setup() { 

  Serial.begin(57600);
  nh.initNode();
  nh.advertise(angle_pub);
  nh.advertise(accel_pub);
  Wire.begin();
  
  delay(5);
  sixDOF.init(); //begin the IMU
  // adx.init(FIMU_ACC_ADDR);
  delay(5);
}

void loop() { 
  // Test
  // Serial.println(*p_cnt);

  // Read Angles
  // sixDOF.getQ(Q_angles); 
  sixDOF.getAngles(angles);
  if ( angles[0]  > 180) { angles[0] = angles[0] - 360;} // Change degrees between 180 and -180
  if ( angles[1]  > 180) { angles[1] = angles[1] - 360;} // Change degrees between 180 and -180
  if ( angles[2]  > 180) { angles[2] = angles[2] - 360;} // Change degrees between 180 and -180
  // sixDOF.getValues(values);
  // angle_x = angles[1];
  // angle_y = angles[2];
  // angle_z = angles[0];
  
  // Serial.print("around z: ");
  // Serial.print(angles[0]);
  // Serial.print(" | ");
  // Serial.print("around x: ");
  // Serial.print(angles[1]);
  // Serial.print(" | ");
  // Serial.print("around y: ");
  // Serial.print(angles[2]);
  // Serial.println(" | ");

  // Read Acceleration
  sixDOF.acc.readAccel(&int_accel[0], &int_accel[1], &int_accel[2]);

  // accel[0] = ((float) int_accel[0])/26;  
  // accel[1] = ((float) int_accel[1])/26;  
  // accel[2] = ((float) int_accel[2])/26;  

  // Serial.print("g in x: ");
  // Serial.print(accel_x);
  // Serial.print(" | ");
  // Serial.print("g in y: ");
  // Serial.print(accel_y);
  // Serial.print(" | ");
  // Serial.print("g in z: ");
  // Serial.print(int_accel[2]);
  // Serial.print(" | ");
  // Serial.print("\n");

  // Publish
  // accel_msg.x = accel[0];
  // accel_msg.y = accel[1];
  // accel_msg.z = accel[2];
  accel_msg.x = int_accel[0];
  accel_msg.y = int_accel[1];
  accel_msg.z = int_accel[2];
  accel_pub.publish( &accel_msg );

  angle_msg.x = angles[1];
  angle_msg.y = angles[2];
  angle_msg.z = angles[0];
  angle_pub.publish( &angle_msg );

  nh.spinOnce();

  delay(1); 
}


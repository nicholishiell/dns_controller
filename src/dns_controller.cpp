#include "ros/ros.h"
#include "bupimo_msgs/VelocityCommand.h"
#include "bupimo_msgs/BlobArray.h"
#include "Vector2d.h"

#include <stdio.h>
#include <string>
#include <fstream>

using namespace std;

// Global variables
float formationNormal = -45.;
float avoidanceThreshold = 235.;
float angularWidthOfZoneC = 25.;
float avoidanceAngle = 135.;

bool firstBlob = false;
bool commandRecieved = false;
bool zoneA = false;
bool zoneB = false;
bool zoneC = false;
bool zoneD = false;

float maxDot = -1.;
float minDot = 1.;
float avoidBearing = 0.;


void BlobBearingsCallback(const bupimo_msgs::BlobArray::ConstPtr& msg){

    
  // Get number of blobs detected
  int nBlobs = msg->blobArray.size();

  // reset values
  maxDot = -1.;
  minDot = 1.;
  zoneA = zoneB = zoneC = zoneD = false;
  
  for(int i = 0; i < nBlobs; i++){
    firstBlob = true;

    float blobBearing = msg->blobArray[i].bearing;
    float blobX = msg->blobArray[i].x;
    float blobY = msg->blobArray[i].y;
    float blobDistance = sqrt( blobX*blobX + blobY*blobY);
    
    // Check avoidance threshold
    if(blobDistance <= avoidanceThreshold){
      zoneA = true;
      avoidBearing = blobBearing;
    }

    // Now check to see what other zone the blob is in
    float diff = fabs(blobBearing - formationNormal);
    if(diff <= angularWidthOfZoneC){
      zoneC = true;
    }
    else if(diff <= 90.){
      zoneB = true;
    }
    else{
      zoneD = true;
    }

    // Now calculate the dot product between blob and formation normal
    float dot = DotProduct(new Vector2d(blobBearing*M_PI/180.), new Vector2d(formationNormal*M_PI/180.));
    if(dot > maxDot){
      maxDot = dot;
    }
    if(dot < minDot){
      minDot = dot;
    }
    
  }
}

int CalculateSensorState(){
  int state = 0;

  if(zoneA) state += 1;
  if(zoneB) state += 2;
  if(zoneC) state += 4;
  if(zoneD) state += 8;
  
  return state;
}

void GetParameters(){
  fstream paramFile;
  
  paramFile.open("/home/pi/ns_catkin_ws/calibData/paramFile.dat");
  paramFile >> formationNormal;
  paramFile >> avoidanceThreshold;
  paramFile >> angularWidthOfZoneC;
  paramFile >> avoidanceAngle;
  
  paramFile.close();  
}

int main(int argc, char **argv){

  // Set all parameters
  GetParameters();

  ros::init(argc, argv, "behaviour_controller");
  
  ros::NodeHandle n;

  ros::Subscriber blobBearing_Sub = n.subscribe("blobsGlobal", 1000, BlobBearingsCallback);  
  ros::Publisher command_pub = n.advertise<bupimo_msgs::VelocityCommand>("dns_command", 1000);
  
  ros::Rate loop_rate(15);

  while (ros::ok()){

    float linearSpeed = 0.;
    float heading = formationNormal;
    
    // Do not issue any movement commands until blobs recieved
    if(!firstBlob){
      printf("Waiting for blob data...\n");
    }
    else{
      int sensorState = CalculateSensorState();
      printf("SensorState = %d\n", sensorState);
      // Avoidance
      if( sensorState % 2 != 0){
	printf("Avoidance\n");
	heading = avoidBearing - avoidanceAngle.;
	linearSpeed = 0.5;
      }
      // Alter course
      else if( sensorState == 4 || sensorState == 6 || sensorState == 12 || sensorState == 14){
	printf("Alter Course\n");
	heading = formationNormal - 90.;
	linearSpeed = 1.;
      }
      // Backwards
      else if( sensorState == 8){
	printf("Back\n");
	heading = formationNormal;
	//linearSpeed = minDot;
	linearSpeed = 0.;
      }
      // Forward
      else if( sensorState == 2 || sensorState == 10){
	printf("Forward\n");
	heading = formationNormal;
	linearSpeed = maxDot;
      }
      // Stop
      else{
	printf("Stop\n");
	linearSpeed = 0.;
	heading = formationNormal;
      }
      printf("%f\t%f\n", linearSpeed, heading);
      printf("=====================\n");
    }

    // Create a VelocityCommand msg and publish it to the topic
    bupimo_msgs::VelocityCommand msg;
    msg.bearing = heading;
    msg.linearSpeed = linearSpeed;

    command_pub.publish(msg);

    // Wait for ROS to go through one step.
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
};

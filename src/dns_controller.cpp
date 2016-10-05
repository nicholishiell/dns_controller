#include "ros/ros.h"
#include "bupimo_msgs/VelocityCommand.h"
#include "bupimo_msgs/BlobArray.h"
#include "Vector2d.h"

#include <stdio.h>
#include <string>

using namespace std;

// Global variables
float formationNormal = 0.;
float avoidanceThreshold = 235.;
float angularWidthOfZoneC = 25.;

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

    float blobBearing = fabs( msg->blobArray[i].bearing);
    float blobX = msg->blobArray[i].x;
    float blobY = msg->blobArray[i].y;
    float blobDistance = sqrt( blobX*blobX + blobY*blobY);

    // Check avoidance threshold
    if(blobDistance <= avoidanceThreshold){
      zoneA = true;
      avoidBearing = blobBearing;
    }

    // Now check to see what other zone the blob is in
    if(blobBearing - formationNormal <= angularWidthOfZoneC){
      zoneC = true;
    }
    else if(blobBearing - formationNormal <= 90.){
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


int main(int argc, char **argv){
  ros::init(argc, argv, "behaviour_controller");

  ros::NodeHandle n;

  ros::Subscriber blobBearing_Sub = n.subscribe("blobsGlobal", 1000, BlobBearingsCallback);  
  ros::Publisher command_pub = n.advertise<bupimo_msgs::VelocityCommand>("dns_command", 1000);
  
  ros::Rate loop_rate(50);

  while (ros::ok()){

    float linearSpeed = 0.;
    float heading = formationNormal;
    
    // Do not issue any movement commands until blobs recieved
    if(!firstBlob){
      printf("Waiting for blob data...\n");
    }
    else{
      int sensorState = CalculateSensorState();

      // Avoidance
      if( sensorState % 2 != 0){
	heading = avoidBearing + 90.;
	linearSpeed = 0.25;
      }
      // Alter course
      else if( sensorState == 4 || sensorState == 6 || sensorState == 12 || sensorState == 14){
	heading = formationNormal + 90.;
	linearSpeed = 1.;
      }
      // Backwards
      else if( sensorState == 8){
	heading = formationNormal - 180.;
	linearSpeed = minDot;
      }
      // Forward
      else if( sensorState == 2 || sensorState == 10){
	heading = formationNormal;
	linearSpeed = maxDot;
      }
      // Stop
      else{
	linearSpeed = 0.;
	heading = formationNormal;
      }
      
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
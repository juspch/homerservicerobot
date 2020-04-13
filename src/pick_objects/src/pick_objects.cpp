#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
// #include <vector>
#include <visualization_msgs/Marker.h>
// #include <std_msgs/String.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::Publisher pickup_pub;

void move_to(const visualization_msgs::Marker marker) {

  //check if marker message is a real target (designated by pickup, dropoff, or starting
  if (marker.ns!= "Pick-up" && marker.ns!= "Drop-off" && marker.ns!= "Starting" ) {
    //do nothing
  }

  //if real target, then move robot to target location
  else {
  MoveBaseClient ac("move_base", true);
  // MoveBaseClient ac("move_base", true);
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  //Defining movebasemessages using target information from marker
  move_base_msgs::MoveBaseGoal target;
  target.target_pose.header.frame_id = "map";
  target.target_pose.header.stamp = ros::Time::now();
  target.target_pose.pose.position.x = marker.pose.position.x;
  target.target_pose.pose.position.y = marker.pose.position.y;
  target.target_pose.pose.orientation.w = marker.pose.orientation.w;

  //send command to move robot
  ac.sendGoal(target);
  ROS_INFO_STREAM("Going to " << marker.ns << " zone");
  ac.waitForResult();

  //if robot reaches location, do corresponding action depending on label of target
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO_STREAM("Robot reached " << marker.ns << " zone");
    //IF TARGET WAS TO MOVE TO PICKUP ZONE
    if (marker.ns == "Pick-up") {
      visualization_msgs::Marker atpickup;
      atpickup = marker;
      atpickup.ns = "Reached Pick-up";
      ROS_INFO_STREAM("Picking up object...");
      ros::Duration(5.0).sleep();
      pickup_pub.publish(atpickup);
      ROS_INFO_STREAM("Object picked up");
    }
    //IF TARGET WAS TO MOVE TO DROPOFF ZONE
    else if (marker.ns == "Drop-off") {
      visualization_msgs::Marker atdropoff;
      atdropoff = marker;
      atdropoff.ns = "Reached Drop-off";
      ROS_INFO_STREAM("Dropping off object...");
      ros::Duration(5.0).sleep();
      pickup_pub.publish(atdropoff);
      ROS_INFO_STREAM("Object dropped off");
    }
    //IF TARGET WAS TO MOVE TO ORIGIN
    else if (marker.ns == "Starting") {
      visualization_msgs::Marker atorigin;
      atorigin.ns = "Done";
      pickup_pub.publish(atorigin);
      ROS_INFO_STREAM("Waiting for object to pick-up...");
    }
  }
  else {
    ROS_INFO_STREAM("The base failed to reach " << marker.ns << " zone");
 }

}
}

int main(int argc, char** argv){

  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;
  ros::Subscriber marker_sub;

  marker_sub = n.subscribe<visualization_msgs::Marker>("/pick_up", 10, move_to);
  pickup_pub = n.advertise<visualization_msgs::Marker>("/pick_up",10);
  ROS_INFO_STREAM("Waiting for object to pick-up...");

  ros::spin();

}

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <vector>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){


  //define pick-up zone, [0] = x coords, [1] = y coords, [2] = w orientation
  std::vector <int> pick_up_coords(3,0);
  pick_up_coords[0] = -7;
  pick_up_coords[1] = -0.5;
  pick_up_coords[2] = 1;


  //define drop-off zone
  std::vector <int> drop_off_coords(3,0);
  drop_off_coords[0] = 5;
  drop_off_coords[1] = 3;
  drop_off_coords[2] = 1;


  // Initialize the simple_navigation_goals node
  // ros::init(argc, argv, "simple_navigation_goals");
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  //Defining movebasemessages
  move_base_msgs::MoveBaseGoal pick_up, drop_off;

  // set up the frame parameters
  // goal.target_pose.header.frame_id = "base_link";
  pick_up.target_pose.header.frame_id = "map";
  pick_up.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  pick_up.target_pose.pose.position.x = pick_up_coords[0];
  pick_up.target_pose.pose.position.y = pick_up_coords[1];
  pick_up.target_pose.pose.orientation.w = pick_up_coords[2];

  // set up the frame parameters
  // goal.target_pose.header.frame_id = "base_link";
  drop_off.target_pose.header.frame_id = "map";
  drop_off.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  drop_off.target_pose.pose.position.x = drop_off_coords[0];
  drop_off.target_pose.pose.position.y = drop_off_coords[1];
  drop_off.target_pose.pose.orientation.w = drop_off_coords[2];




  //GOING TO PICK-UP
   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Going to Pick-up zone");
  ac.sendGoal(pick_up);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Robot reached Pick-up zone");
    ros::Duration(5.0).sleep();
  }
  else {
    ROS_INFO("The base failed to reach Pick-up zone");
    return 0;
 }

  //GOING TO drop_off_coords
  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Going to Drop-off zone");
  ac.sendGoal(drop_off);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Robot reached Drop-off zone");
  }
  else {
    ROS_INFO("The base failed to reach Drop-off zone");
    return 0;
  }

  return 0;
}

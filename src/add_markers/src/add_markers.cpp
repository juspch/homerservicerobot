#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// #include <std_msgs/String.h>
//defining global publishers
ros::Publisher rviz_pub;
ros::Publisher pickup_pub;
bool mission_accomplished = false;

uint32_t sphere = visualization_msgs::Marker::SPHERE;
uint32_t cube = visualization_msgs::Marker::CUBE;


//DEFINE MARKER, PICKUP AND DROPOFF ZONES
uint32_t   marker_shape = cube;
float pickupx = -6.75;
float pickupy = -0.6;
float dropoffx = 5.25;
float dropoffy = 2.5;

void marker_actions(const visualization_msgs::Marker marker) {

  if (marker.ns == "Reached Pick-up") {
    //REMOVING PICKUP MARKER MSG
    visualization_msgs::Marker remove_pickup;
    remove_pickup.ns = "Pick-up";
    remove_pickup.id = 0;
    remove_pickup.action = visualization_msgs::Marker::DELETE;

    //REMOVE PICKUP MARKER FROM RVIZ
    rviz_pub.publish(remove_pickup);
    ROS_INFO_STREAM("Object picked-up");

    //SEND MSG TO PICK OBJECT TO MOVE TO DROP OFF COORDINATES
    visualization_msgs::Marker drop_off;
    drop_off = marker;
    drop_off.ns = "Drop-off";
    drop_off.pose.position.x = dropoffx;
    drop_off.pose.position.y = dropoffy;
    drop_off.pose.orientation.w = 1.0;
    pickup_pub.publish(drop_off);

  }

  else if (marker.ns == "Reached Drop-off") {
    //ADDING DROPOFF MARKER MSG
    visualization_msgs::Marker add_dropoff;
    add_dropoff = marker;
    add_dropoff.ns = "Drop-off";
    add_dropoff.id = 1;
    add_dropoff.lifetime = ros::Duration(30);

    //REMOVE PICKUP MARKER FROM RVIZ
    rviz_pub.publish(add_dropoff);
    ROS_INFO_STREAM("Object dropped off");

    //SEND MSG TO PICK OBJECT TO MOVE TO DROP OFF COORDINATES
    visualization_msgs::Marker move_to_origin;
    move_to_origin.ns = "Starting";
    move_to_origin.pose.position.x = 0;
    move_to_origin.pose.position.y = 0;
    move_to_origin.pose.orientation.w = 1;
    rviz_pub.publish(move_to_origin);
    ROS_INFO_STREAM("Moving back to origin");
    pickup_pub.publish(move_to_origin);
  }

  else if (marker.ns == "Done") {
    mission_accomplished = true;
  }
}

int main( int argc, char** argv )
{

  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(10);

  rviz_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
  pickup_pub = n.advertise<visualization_msgs::Marker>("/pick_up", 10);

  ros::Subscriber pickup_sub = n.subscribe<visualization_msgs::Marker>("/pick_up", 10, marker_actions);


  //DEFINING THE MARKER MSGS
  visualization_msgs::Marker pickup_marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  pickup_marker.header.frame_id = "map";
  pickup_marker.header.stamp = ros::Time::now();
  // Set the namespace and id for this pickup_marker.  This serves to create a unique ID
  // Any pickup_marker sent with the same namespace and id will overwrite the old one
  pickup_marker.ns = "Pick-up";
  pickup_marker.id = 0;
  // Set the pickup_marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  pickup_marker.type = marker_shape;
  // Set the pickup_marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  pickup_marker.action = visualization_msgs::Marker::ADD;
  // Set the pose of the pickup_marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  pickup_marker.pose.position.x = pickupx;
  pickup_marker.pose.position.y = pickupy;
  // pickup_marker.pose.position.z = 0;
  pickup_marker.pose.orientation.x = 0.0;
  pickup_marker.pose.orientation.y = 0.0;
  pickup_marker.pose.orientation.z = 0.0;
  pickup_marker.pose.orientation.w = 1.0;
  // Set the scale of the pickup_marker -- 1x1x1 here means 1m on a side
  pickup_marker.scale.x = .3;
  pickup_marker.scale.y = .3;
  pickup_marker.scale.z = .3;
  // Set the color -- be sure to set alpha to something non-zero!
  pickup_marker.color.r = 0.6f;
  pickup_marker.color.g = 0.5f;
  pickup_marker.color.b = 0.7f;
  pickup_marker.color.a = 1.0;
  pickup_marker.lifetime = ros::Duration();


  //give time to set-up subs and pubs
  ros::Duration(0.5).sleep();
  
  ROS_INFO("Publishing Pick-up Marker to Rviz and /pick_up topic...");
  //PUBLISH MARKER TO RVIZ
  rviz_pub.publish(pickup_marker);
  //SEND FIRST GOAL TO PICK OBJECT NODE
  pickup_pub.publish(pickup_marker);

  while(mission_accomplished!=true) {
  ros::spinOnce();
  }

  return 0;

}

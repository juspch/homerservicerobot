#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(10);
  // ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10, true);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  // Set our initial shape type to be a cube
  uint32_t cube = visualization_msgs::Marker::CUBE;
  uint32_t sphere = visualization_msgs::Marker::SPHERE;
  uint32_t cylinder = visualization_msgs::Marker::CYLINDER;

  visualization_msgs::Marker pick_upmarker, drop_offmarker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  pick_upmarker.header.frame_id = "map";
  pick_upmarker.header.stamp = ros::Time::now();
  // Set the namespace and id for this pick_upmarker.  This serves to create a unique ID
  // Any pick_upmarker sent with the same namespace and id will overwrite the old one
  pick_upmarker.ns = "pick_up_marker";
  pick_upmarker.id = 0;
  // Set the pick_upmarker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  pick_upmarker.type = sphere;
  // Set the pick_upmarker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  pick_upmarker.action = visualization_msgs::Marker::ADD;
  // Set the pose of the pick_upmarker.  This is a full 6DOF pose relative to the frame/time specified in the header
  pick_upmarker.pose.position.x = -7;
  pick_upmarker.pose.position.y = 0;
  // pick_upmarker.pose.position.z = 0;
  pick_upmarker.pose.orientation.x = 0.0;
  pick_upmarker.pose.orientation.y = 0.0;
  pick_upmarker.pose.orientation.z = 0.0;
  pick_upmarker.pose.orientation.w = 1.0;
  // Set the scale of the pick_upmarker -- 1x1x1 here means 1m on a side
  pick_upmarker.scale.x = .3;
  pick_upmarker.scale.y = .3;
  pick_upmarker.scale.z = .3;
  // Set the color -- be sure to set alpha to something non-zero!
  pick_upmarker.color.r = 0.6f;
  pick_upmarker.color.g = 0.5f;
  pick_upmarker.color.b = 0.7f;
  pick_upmarker.color.a = 1.0;
  pick_upmarker.lifetime = ros::Duration(5);

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  drop_offmarker.header.frame_id = "map";
  drop_offmarker.header.stamp = ros::Time::now();
  // Set the namespace and id for this drop_offmarker.  This serves to create a unique ID
  // Any drop_offmarker sent with the same namespace and id will overwrite the old one
  drop_offmarker.ns = "drop_off_marker";
  drop_offmarker.id = 1;
  // Set the drop_offmarker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  drop_offmarker.type = sphere;
  // Set the drop_offmarker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  drop_offmarker.action = visualization_msgs::Marker::ADD;
  // Set the pose of the drop_offmarker.  This is a full 6DOF pose relative to the frame/time specified in the header
  drop_offmarker.pose.position.x = 5;
  drop_offmarker.pose.position.y = 3;
  // drop_offmarker.pose.position.z = 0;
  drop_offmarker.pose.orientation.x = 0.0;
  drop_offmarker.pose.orientation.y = 0.0;
  drop_offmarker.pose.orientation.z = 0.0;
  drop_offmarker.pose.orientation.w = 1.0;
  // Set the scale of the drop_offmarker -- 1x1x1 here means 1m on a side
  drop_offmarker.scale.x = .3;
  drop_offmarker.scale.y = .3;
  drop_offmarker.scale.z = .3;
  // Set the color -- be sure to set alpha to something non-zero!
  drop_offmarker.color.r = 0.6f;
  drop_offmarker.color.g = 0.5f;
  drop_offmarker.color.b = 0.7f;
  drop_offmarker.color.a = 1.0;
  drop_offmarker.lifetime = ros::Duration(5);

  //REMOVED AFTER ADDING LATCH to pub
  // ros::Duration(0.0001).sleep();
  // marker_pub.publish(pick_upmarker);
  // // ros::Duration(5.0).sleep();
  // marker_pub.publish(drop_offmarker);

  // // Publish the marker
  //   while (marker_pub.getNumSubscribers() < 1)
  //   {
  //     if (!ros::ok())
  //     {
  //       return 0;
  //     }
  //     else if (marker_pub.getNumSubscribers() < 1) {
  //     ROS_WARN_ONCE("Please create a subscriber to the marker");
  //     return 0;
  //    }
  //    else
  //    sleep(1);
  //   }
    ros::Duration(0.005).sleep();
    ROS_INFO("Pick-Up Published");
    marker_pub.publish(pick_upmarker);
    //wait for marker 1 to dissapear after 5 seconds + 5 seconds invisible time = 10
    ros::Duration(10.0).sleep();
    ROS_INFO("Drop-Off Published");
    marker_pub.publish(drop_offmarker);
    ros::Duration(5.0).sleep();
  return 0;
  // while (ros::ok())
  // {
  //   visualization_msgs::Marker marker;
  //   // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  //   marker.header.frame_id = "frame_id";
  //   marker.header.stamp = ros::Time::now();
  //
  //   // Set the namespace and id for this marker.  This serves to create a unique ID
  //   // Any marker sent with the same namespace and id will overwrite the old one
  //   marker.ns = "pick_up_marker";
  //   marker.id = 0;
  //
  //   // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  //   marker.type = sphere;
  //
  //   // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  //   marker.action = visualization_msgs::Marker::ADD;
  //
  //   // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  //   marker.pose.position.x = -7;
  //   marker.pose.position.y = 0;
  //   // marker.pose.position.z = 0;
  //   marker.pose.orientation.x = 0.0;
  //   marker.pose.orientation.y = 0.0;
  //   marker.pose.orientation.z = 0.0;
  //   marker.pose.orientation.w = 1.0;
  //
  //   // Set the scale of the marker -- 1x1x1 here means 1m on a side
  //   marker.scale.x = 1.0;
  //   marker.scale.y = 1.0;
  //   marker.scale.z = 1.0;
  //
  //   // Set the color -- be sure to set alpha to something non-zero!
  //   marker.color.r = 0.0f;
  //   marker.color.g = 1.0f;
  //   marker.color.b = 0.0f;
  //   marker.color.a = 1.0;
  //
  //   marker.lifetime = ros::Duration();
  //
  //   // Publish the marker
  //   while (marker_pub.getNumSubscribers() < 1)
  //   {
  //     if (!ros::ok())
  //     {
  //       return 0;
  //     }
  //     ROS_WARN_ONCE("Please create a subscriber to the marker");
  //     sleep(1);
  //   }
  //   marker_pub.publish(marker);
  //
  //   // Cycle between different shapes
  //   switch (shape)
  //   {
  //   case visualization_msgs::Marker::CUBE:
  //     shape = visualization_msgs::Marker::SPHERE;
  //     break;
  //   case visualization_msgs::Marker::SPHERE:
  //     shape = visualization_msgs::Marker::ARROW;
  //     break;
  //   case visualization_msgs::Marker::ARROW:
  //     shape = visualization_msgs::Marker::CYLINDER;
  //     break;
  //   case visualization_msgs::Marker::CYLINDER:
  //     shape = visualization_msgs::Marker::CUBE;
  //     break;
  //   }
  //
  //   r.sleep();
  // }
}

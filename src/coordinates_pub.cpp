#include <ros/ros.h>
#include "geometry_msgs/Point.h" 

void Callback(const geometry_msgs::Point& coordinates) {
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  ros::Publisher pub = nh.advertise<geometry_msgs::Point>("/pole_coordinates", 100);
  geometry_msgs::Point msg;

  while(ros::ok()) {
    msg = coordinates;
    pub.publish(msg);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
}

// Main ROS method
int main(int argc, char **argv) {
    
  // Initialize the node and set the name
  ros::init(argc, argv, "point_of_pole");
   
  ros::NodeHandle nh;

  // Create the service and advertise it to the ROS computational network
  ros::Subscriber sub = nh.subscribe("/point_of_pole", 1, Callback);
   
  // Keep processing information over and over again
  ros::spin();
 
  // Program completed successfully
  return 0;
}
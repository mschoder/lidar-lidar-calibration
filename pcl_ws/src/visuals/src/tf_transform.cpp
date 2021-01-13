#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>

void botCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // static tf2_ros::TransformBroadcaster br;
    // geometry_msgs::TransformStamped transformStamped;

    // ros::NodeHandle n("~");
    // std::string sensor;
    // n.getParam("sensor", sensor);

    // transformStamped.header.stamp = ros::Time::now();
    // transformStamped.header.frame_id = "husky4/map";

    // if (sensor == "main"){ transformStamped.child_frame_id = "husky4/velodyne";}
    // if (sensor == "front"){ transformStamped.child_frame_id = "husky4/velodyne_front";}
    // if (sensor == "rear"){ transformStamped.child_frame_id = "husky4/velodyne_rear";}

    // transformStamped.transform.translation.x = msg->pose.pose.position.x;
    // transformStamped.transform.translation.y = msg->pose.pose.position.y;
    // transformStamped.transform.translation.z = msg->pose.pose.position.z;

    // transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
    // transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
    // transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
    // transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;

    // br.sendTransform(transformStamped);

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped1, transformStamped2;

    ros::NodeHandle n("~");
    std::string sensor1, sensor2;
    n.getParam("sensor1", sensor1);
    n.getParam("sensor2", sensor2);

    // Transform sensor 1 to husky4/map
    transformStamped1.header.stamp = ros::Time::now();
    transformStamped1.header.frame_id = "husky4/map";

    if (sensor1 == "main"){ transformStamped1.child_frame_id = "husky4/velodyne";}
    if (sensor1 == "front"){ transformStamped1.child_frame_id = "husky4/velodyne_front";}
    if (sensor1 == "rear"){ transformStamped1.child_frame_id = "husky4/velodyne_rear";}

    transformStamped1.transform.translation.x = msg->pose.pose.position.x;
    transformStamped1.transform.translation.y = msg->pose.pose.position.y;
    transformStamped1.transform.translation.z = msg->pose.pose.position.z;

    transformStamped1.transform.rotation.x = msg->pose.pose.orientation.x;
    transformStamped1.transform.rotation.y = msg->pose.pose.orientation.y;
    transformStamped1.transform.rotation.z = msg->pose.pose.orientation.z;
    transformStamped1.transform.rotation.w = msg->pose.pose.orientation.w;

    br.sendTransform(transformStamped1);

    // Transform sensor 2 to husky4/map
    transformStamped2.header.stamp = ros::Time::now();
    transformStamped2.header.frame_id = "husky4/map";

    if (sensor2 == "main"){ transformStamped2.child_frame_id = "husky4/velodyne";}
    if (sensor2 == "front"){ transformStamped2.child_frame_id = "husky4/velodyne_front";}
    if (sensor2 == "rear"){ transformStamped2.child_frame_id = "husky4/velodyne_rear";}

    transformStamped2.transform.translation.x = msg->pose.pose.position.x;
    transformStamped2.transform.translation.y = msg->pose.pose.position.y;
    transformStamped2.transform.translation.z = msg->pose.pose.position.z;

    transformStamped2.transform.rotation.x = msg->pose.pose.orientation.x;
    transformStamped2.transform.rotation.y = msg->pose.pose.orientation.y;
    transformStamped2.transform.rotation.z = msg->pose.pose.orientation.z;
    transformStamped2.transform.rotation.w = msg->pose.pose.orientation.w;

    br.sendTransform(transformStamped2);

}
int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_broadcaster");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/husky4/lo_frontend/odometry", 10, botCallback);
  ros::spin();
  return 0;
}
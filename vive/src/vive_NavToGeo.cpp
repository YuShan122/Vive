#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

ros::Subscriber vive_sub;
ros::Publisher vive_pub;

void vive_callback(const nav_msgs::Odometry::ConstPtr& msg){
    geometry_msgs::PoseWithCovarianceStamped geo_pose;
    geo_pose.pose.pose.position.x = msg->pose.pose.position.x;
    geo_pose.pose.pose.position.y = msg->pose.pose.position.y;
    geo_pose.pose.pose.position.z = msg->pose.pose.position.z;
    geo_pose.pose.pose.orientation.w = msg->pose.pose.orientation.w;
    geo_pose.pose.pose.orientation.x = msg->pose.pose.orientation.x;
    geo_pose.pose.pose.orientation.y = msg->pose.pose.orientation.y;
    geo_pose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
    vive_pub.publish(geo_pose);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vive_NavToGeo");
    ros::NodeHandle nh;
    vive_sub = nh.subscribe("rival1/odom", 10, vive_callback);
    vive_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("Lidar_bonbonbon",10);
    ros::Rate rate(20);
    printf("%s\n",argv[0]);
    while(true){
        ros::spinOnce();
        rate.sleep();
    }
}
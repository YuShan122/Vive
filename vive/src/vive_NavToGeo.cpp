#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

ros::Subscriber vive_sub;
ros::Publisher vive_pub;
geometry_msgs::PoseWithCovarianceStamped geo_pose;

void vive_callback(const nav_msgs::Odometry::ConstPtr& msg){

    geo_pose.header.frame_id = "robot1/map";
    geo_pose.header.stamp = msg->header.stamp;

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
    ros::NodeHandle nh_("~");
    vive_sub = nh.subscribe("rival1/odom", 10, vive_callback);
    vive_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(argv[1],10);
    ros::Rate rate(20);

    double covariance_0, covariance_7, covariance_35;
    nh_.getParam("covariance_0", covariance_0);
    nh_.getParam("covariance_7", covariance_7);
    nh_.getParam("covariance_35", covariance_35);
    geo_pose.pose.covariance[0] = covariance_0;
    geo_pose.pose.covariance[7] = covariance_7;
    geo_pose.pose.covariance[35] = covariance_35;

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
}
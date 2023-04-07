#include <costmap_converter/ObstacleArrayMsg.h>
#include <costmap_converter/ObstacleMsg.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>

typedef struct vive {
    double x, y, z;
    double W, X, Y, Z;
    double yaw, roll, pitch;
    double Vx, Vy, Vz;
    double VX, VY, VZ;
}VIVE;

class RivalMulti{
    public:
        RivalMulti(ros::NodeHandle nh_g, ros::NodeHandle nh_p);
        void Rival1_callback(const nav_msgs::Odometry::ConstPtr &rival1_msg);
        void Rival2_callback(const nav_msgs::Odometry::ConstPtr &rival2_msg);
        // void Lidar_callback(const  &lidar_msg);
        double distance(tf::Vector3 a, tf::Vector3 b);

    private:
        ros::NodeHandle nh;
        ros::NodeHandle nh_local;
        ros::Subscriber rival1_sub;
        ros::Subscriber rival2_sub;
        ros::Subscriber lidar_sub;
        ros::Publisher rival1_pub;
        ros::Publisher rival2_pub;

        VIVE rival1_odom,rival2_odom;
};

RivalMulti::RivalMulti(ros::NodeHandle nh_g, ros::NodeHandle nh_p){
    nh = nh_g;
    nh_local = nh_p;

    rival1_sub = nh.subscribe("tracker_vel",10, &RivalMulti::Rival1_callback, this);
    rival2_sub = nh.subscribe("tracker_vel",10, &RivalMulti::Rival2_callback, this);
    // lidar_sub = nh.subscribe("tracker_vel",10, &RivalMulti::Lidar_callback, this);
    rival1_pub = nh.advertise<nav_msgs::Odometry>("/rival1/odom",10);
    rival2_pub = nh.advertise<nav_msgs::Odometry>("/rival2/odom",10);
}

void RivalMulti::Rival1_callback(const nav_msgs::Odometry::ConstPtr &rival1_msg){
    rival1_odom.x = rival1_msg->pose.pose.position.x;
    rival1_odom.y = rival1_msg->pose.pose.position.y;
    rival1_odom.z = rival1_msg->pose.pose.position.z;
    rival1_odom.W = rival1_msg->pose.pose.orientation.w;
    rival1_odom.X = rival1_msg->pose.pose.orientation.x;
    rival1_odom.Y = rival1_msg->pose.pose.orientation.y;
    rival1_odom.Z = rival1_msg->pose.pose.orientation.z;
    rival1_odom.Vx = rival1_msg->twist.twist.linear.x;
    rival1_odom.Vy = rival1_msg->twist.twist.linear.y;
    rival1_odom.Vz = rival1_msg->twist.twist.linear.z;
    rival1_odom.VX = rival1_msg->twist.twist.angular.x;
    rival1_odom.VY = rival1_msg->twist.twist.angular.y;
    rival1_odom.VZ = rival1_msg->twist.twist.angular.z;
}

double distance(tf::Vector3 a_point, tf::Vector3 b_point){
    return sqrt( pow((a_point[0]-b_point[0]),2) + pow((a_point[1]-b_point[1]),2) );
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_rival_detector");
    ros::NodeHandle nh; 
    ros::NodeHandle nh_("~");
    RivalMulti RivalMulti(nh, nh_);

}
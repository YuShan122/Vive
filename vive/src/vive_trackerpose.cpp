#include <cstdio>
#include <string>
#include <iostream>

#include <ros/ros.h>
// #include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


typedef struct vivePose {
    double x, y, z;
    double W, X, Y, Z;
    double yaw, roll, pitch;
}VIVEPOSE;

class Robot {
public:
    Robot(ros::NodeHandle nh_g, ros::NodeHandle nh_l);
    void lookup_transform_from_map();
    void publish_vive_pose();
    void print_pose();
private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_;
    ros::Publisher pose_pub;
    geometry_msgs::PoseWithCovarianceStamped pose;
    tf::TransformListener listener;
    tf::StampedTransform transform_from_map;
    std::string tracker_frame;
    bool active;
    double covariance[36];
    VIVEPOSE poseV;
};
Robot::Robot(ros::NodeHandle nh_g, ros::NodeHandle nh_l) {
    nh = nh_g;
    nh_ = nh_l;
    // std::string node_name = ros::this_node::getName();
    pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("lidar_bonbonbon", 10);
    nh_.getParam("tracker", tracker_frame);
    nh_.getParam("covariance0", covariance[0]);
    nh_.getParam("covariance7", covariance[7]);
    nh_.getParam("covariance14", covariance[14]);
    nh_.getParam("covariance21", covariance[21]);
    nh_.getParam("covariance28", covariance[28]);
    nh_.getParam("covariance35", covariance[35]);
    // ROS_INFO("test local nh: %s", tracker_frame);
    std::cout << "tracker: " << tracker_frame << std::endl;

}
void Robot::lookup_transform_from_map() {
    // std::string node_name = ros::this_node::getName();
    // if (listener.canTransform("map", tracker_frame, ros::Time::now())) {
    try {
        listener.lookupTransform("map", tracker_frame, ros::Time(0), transform_from_map);
    }
    catch (tf::TransformException& ex) {
        printf("%s", ex.what());
        std::cout << tracker_frame << " connot transform" << std::endl;
    }
    // }
    // else {
        // printf("%s cannot transform\n", tracker_frame);
    // }
}
void Robot::publish_vive_pose() {
    pose.pose.pose.orientation.w = transform_from_map.getRotation().getW();
    pose.pose.pose.orientation.x = transform_from_map.getRotation().getX();
    pose.pose.pose.orientation.y = transform_from_map.getRotation().getY();
    pose.pose.pose.orientation.z = transform_from_map.getRotation().getZ();
    pose.pose.pose.position.x = transform_from_map.getOrigin().getX();
    pose.pose.pose.position.y = transform_from_map.getOrigin().getY();
    pose.pose.pose.position.z = transform_from_map.getOrigin().getZ();
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = tracker_frame;
    pose.pose.covariance[0] = covariance[0];
    pose.pose.covariance[7] = covariance[7];
    pose.pose.covariance[14] = covariance[14];
    pose.pose.covariance[21] = covariance[21];
    pose.pose.covariance[28] = covariance[28];
    pose.pose.covariance[35] = covariance[35];
    pose_pub.publish(pose);
}
void Robot::print_pose() {
    // std::string node_name = ros::this_node::getName();
    poseV.W = transform_from_map.getRotation().getW();
    poseV.X = transform_from_map.getRotation().getX();
    poseV.Y = transform_from_map.getRotation().getY();
    poseV.Z = transform_from_map.getRotation().getZ();
    poseV.x = transform_from_map.getOrigin().getX();
    poseV.y = transform_from_map.getOrigin().getY();
    poseV.z = transform_from_map.getOrigin().getZ();
    printf("(x y z W X Y Z)\n");
    printf("%f, %f, %f, %f, %f, %f, %f\n",
        poseV.x, poseV.y, poseV.z, poseV.W, poseV.X, poseV.Y, poseV.Z);
}

int freq = 21;
int unit;
int tracker_num;
// std::string name_space;
// std::string node_name;

void initialize(ros::NodeHandle nh_) {
    // std::string node_name = ros::this_node::getName();
    nh_.getParam("freq", freq);
    nh_.getParam("unit", unit);
    printf("test local nh: %d, %d\n", freq, unit);
}
void deleteParam() {
    std::string node_name = ros::this_node::getName();
    std::string deleteparam = "rosparam delete " + node_name;
    system(deleteparam.c_str());
    std::cout << node_name << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vive_trackerpose");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");
    // ros::Publisher pose_pub_1;
    // ros::Publisher pose_pub_2;
    // ros::Publisher pose_pub_2 = nh.advertise<geometry_msgs::PoseStamped>("vive_pose_2", 10);
    // geometry_msgs::PoseStamped vive_pose;
    // tf::TransformBroadcaster br;
    // tf::TransformListener listener;
    initialize(nh_);
    ros::Rate rate(freq);

    Robot robot(nh, nh_);
    // std::string node_name;
    // std::string name_space;
    // node_name = ros::this_node::getName();
    // name_space = ros::this_node::getNamespace();

    while (ros::ok()) {

        robot.lookup_transform_from_map();
        robot.publish_vive_pose();
        robot.print_pose();
        // printf("%s\n", node_name);
        // printf("%s\n", name_space);

        rate.sleep();
    }

    deleteParam();
    printf("closed\n");

    return(0);
}
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
    void print_pose(int unit_);
private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_;
    ros::Publisher pose_pub;
    geometry_msgs::PoseWithCovarianceStamped pose;
    tf::TransformListener listener;
    tf::StampedTransform transform_from_map;
    std::string node_name_;
    std::string robot_name;
    std::string tracker_frame;
    std::string map_frame;
    bool has_tf;
    double covariance[36];
    VIVEPOSE poseV;
};
Robot::Robot(ros::NodeHandle nh_g, ros::NodeHandle nh_l) {
    nh = nh_g;
    nh_ = nh_l;
    pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("lidar_bonbonbon", 10);   //topic name: [ns of <group>]/lidar_bonbonbon
    node_name_ = ros::this_node::getName();
    bool ok = true;
    ok &= nh_.getParam("tracker", tracker_frame);       //path: under this node
    ok &= nh_.getParam("map", map_frame);
    ok &= nh_.getParam("robot_name", robot_name);
    ok &= nh_.getParam("covariance0", covariance[0]);
    ok &= nh_.getParam("covariance7", covariance[7]);
    ok &= nh_.getParam("covariance14", covariance[14]);
    ok &= nh_.getParam("covariance21", covariance[21]);
    ok &= nh_.getParam("covariance28", covariance[28]);
    ok &= nh_.getParam("covariance35", covariance[35]);

    std::cout << "node: " << node_name_ << " getting parameters of the robot..." << std::endl;
    std::cout << "robot name: " << robot_name << std::endl;
    std::cout << "map frame: " << map_frame << std::endl;
    std::cout << "tracker: " << tracker_frame << std::endl;

    if (ok) {
        std::cout << "node: " << node_name_ << " get parameters of the robot sucessed." << std::endl;
    }
    else {
        std::cout << "node: " << node_name_ << " get parameters of robot failed." << std::endl;
    }
}
void Robot::lookup_transform_from_map() {
    has_tf = listener.canTransform(map_frame, tracker_frame, ros::Time(0));
    try {
        listener.lookupTransform(map_frame, tracker_frame, ros::Time(0), transform_from_map);
    }
    catch (tf::TransformException& ex) {
        printf("%s", ex.what());
        std::cout << "connot transform from " << map_frame << " to " << tracker_frame << std::endl;
    }
}
void Robot::publish_vive_pose() {
    if (has_tf) {
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
}
void Robot::print_pose(int unit_) {
    poseV.x = transform_from_map.getOrigin().getX() * unit_;
    poseV.y = transform_from_map.getOrigin().getY() * unit_;
    poseV.z = transform_from_map.getOrigin().getZ() * unit_;
    poseV.W = transform_from_map.getRotation().getW() * unit_;
    poseV.X = transform_from_map.getRotation().getX() * unit_;
    poseV.Y = transform_from_map.getRotation().getY() * unit_;
    poseV.Z = transform_from_map.getRotation().getZ() * unit_;
    if (has_tf) {
        std::cout << robot_name << "/" << "trackerpose: " << map_frame << "->" << tracker_frame << " (x y z W X Y Z)" << std::endl;
        std::cout << poseV.x << " " << poseV.y << " " << poseV.z << " "
            << poseV.W << " " << poseV.X << " " << poseV.Y << " " << poseV.Z << std::endl;
    }
    else {
        std::cout << robot_name << "/" << map_frame << "->" << tracker_frame << " do not have tf." << std::endl;
    }
}

int freq = 20;
int unit = 1;
std::string name_space;
std::string node_name;

void initialize(ros::NodeHandle nh_) {
    bool ok = true;
    node_name = ros::this_node::getName();
    name_space = ros::this_node::getNamespace();
    ok &= nh_.getParam("freq", freq);
    ok &= nh_.getParam("unit", unit);
    std::cout << "param: freq= " << freq << std::endl;
    std::cout << "param: unit= " << unit << std::endl;
    if (ok) {
        std::cout << "node: " << node_name << " get parameters of node sucessed." << std::endl;
    }
    else {
        std::cout << "node: " << node_name << " get parameters of node failed." << std::endl;
    }
    std::cout << "node: " << node_name << " initialized." << "(in namespace: " << name_space << ")" << std::endl;
}
void deleteParam() {
    std::string deleteparam = "rosparam delete " + node_name;
    system(deleteparam.c_str());
    std::cout << "node: " << node_name << " parameters deleted" << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vive_trackerpose");
    ros::NodeHandle nh; //path: the ns of <group> of the launch file
    ros::NodeHandle nh_("~");   //path: this node
    initialize(nh_);
    ros::Rate rate(freq);

    Robot robot(nh, nh_);

    while (ros::ok()) {

        robot.lookup_transform_from_map();
        robot.publish_vive_pose();
        robot.print_pose(unit);

        rate.sleep();
    }

    deleteParam();
    std::cout << "node: " << node_name << " closed." << std::endl;

    return(0);
}
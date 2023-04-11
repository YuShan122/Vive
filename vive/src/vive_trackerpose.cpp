#include <cstdio>
#include <string>
#include <iostream>
#include <cmath>

#include <ros/ros.h>
// #include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/SetBool.h>


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
    void ekf_sub_callback(const geometry_msgs::PoseWithCovarianceStamped& msg_);
    bool in_boundry(double x, double y);
private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_;
    ros::Publisher pose_pub;
    ros::Subscriber ekf_sub;
    geometry_msgs::PoseWithCovarianceStamped pose;
    tf::TransformListener listener;
    tf::StampedTransform transform_from_map;
    std::string node_name_;
    std::string robot_name;
    std::string tracker_frame;
    std::string map_frame;
    bool has_tf;
    bool match_ekf;
    double covariance[36];
    VIVEPOSE poseV;
    double tole_with_ekf;
    bool ekf_active;
    bool in_boundry_;
};
Robot::Robot(ros::NodeHandle nh_g, ros::NodeHandle nh_l) {
    nh = nh_g;
    nh_ = nh_l;
    pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("vive_bonbonbon", 10);   //topic name: [ns of <group>]/vive_bonbonbon
    ekf_sub = nh.subscribe("ekf_pose", 10, &Robot::ekf_sub_callback, this);
    node_name_ = ros::this_node::getName();
    bool ok = true;
    ok &= nh_.getParam("tracker", tracker_frame);       //path: under this node
    ok &= nh_.getParam("map", map_frame);
    ok &= nh_.getParam("robot_name", robot_name);
    ok &= nh_.getParam("tole_with_ekf", tole_with_ekf);
    ok &= nh_.getParam("ekf_active", ekf_active);
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

    match_ekf = true;
    in_boundry_ = true;
}
void Robot::lookup_transform_from_map() {
    has_tf = true;
    try {
        listener.lookupTransform(map_frame, tracker_frame, ros::Time(0), transform_from_map);
    }
    catch (tf::TransformException& ex) {
        // printf("%s", ex.what());
        has_tf = false;
        std::cout << "connot lookup transform from " << map_frame << " to " << tracker_frame << std::endl;
    }
    poseV.x = transform_from_map.getOrigin().getX();
    poseV.y = transform_from_map.getOrigin().getY();
    poseV.z = transform_from_map.getOrigin().getZ();
    poseV.W = transform_from_map.getRotation().getW();
    poseV.X = transform_from_map.getRotation().getX();
    poseV.Y = transform_from_map.getRotation().getY();
    poseV.Z = transform_from_map.getRotation().getZ();
    in_boundry_ = in_boundry(poseV.x, poseV.y);
}
void Robot::publish_vive_pose() {
    if (has_tf && match_ekf && in_boundry_) {
        pose.pose.pose.orientation.w = poseV.W;
        pose.pose.pose.orientation.x = poseV.X;
        pose.pose.pose.orientation.y = poseV.Y;
        pose.pose.pose.orientation.z = poseV.Z;
        pose.pose.pose.position.x = poseV.x;
        pose.pose.pose.position.y = poseV.y;
        pose.pose.pose.position.z = poseV.z;
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
    else {
        std::cout << robot_name << " did not publish vive_pose ";
        if (!has_tf) std::cout << "(has no tf) ";
        if (!match_ekf) std::cout << "(dose not match ekf) ";
        if (!in_boundry_) std::cout << "(dose not in boundry) ";
        std::cout << std::endl;
    }
}
void Robot::print_pose(int unit_) {
    if (has_tf) {
        std::cout << robot_name << "/" << "trackerpose: " << map_frame << "->" << tracker_frame << " (x y z W X Y Z) "
            << poseV.x * unit_ << " " << poseV.y * unit_ << " " << poseV.z * unit_ << " "
            << poseV.W << " " << poseV.X << " " << poseV.Y << " " << poseV.Z << std::endl;
    }
    else {
        std::cout << robot_name << "/" << map_frame << "->" << tracker_frame << " do not have tf." << std::endl;
    }
}
void Robot::ekf_sub_callback(const geometry_msgs::PoseWithCovarianceStamped& msg_) {
    match_ekf = true;
    if (ekf_active) {
        VIVEPOSE ekf_p;
        ekf_p.x = msg_.pose.pose.position.x;
        ekf_p.y = msg_.pose.pose.position.y;
        // ekf_p.z = msg_.pose.pose.position.z;
        // ekf_p.W = msg_.pose.pose.orientation.w;
        // ekf_p.X = msg_.pose.pose.orientation.x;
        // ekf_p.Y = msg_.pose.pose.orientation.y;
        // ekf_p.Z = msg_.pose.pose.orientation.z;

        double d = (double)sqrt(pow(ekf_p.x - poseV.x, 2) + pow(ekf_p.y - poseV.y, 2));
        if (d > tole_with_ekf) {
            match_ekf = false;
            std::cout << robot_name << ": vivepose dose not match ekf. ";
            std::cout << "ekf_pose(x y z)= (" << ekf_p.x << " " << ekf_p.y << " " << ekf_p.z << ") " << "d= " << d << std::endl;
        }
    }
}
bool Robot::in_boundry(double x, double y) {
    bool in = false;
    if (x < 3 && x > 0 && y < 2 && y > 0) in = true;
    return in;
}

int freq = 20;
int unit = 1;
std::string name_space;
std::string node_name;
bool world_is_running = true;

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
bool run_srv_func(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    world_is_running = req.data;
    res.success = true;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vive_trackerpose");
    ros::NodeHandle nh; //path: the ns of <group> of the launch file
    ros::NodeHandle nh_("~");   //path: this node
    initialize(nh_);
    ros::Rate rate(freq);
    ros::ServiceServer run_srv = nh.advertiseService("survive_world_is_running", run_srv_func);

    Robot robot(nh, nh_);

    while (ros::ok()) {
        if (world_is_running) {
            robot.lookup_transform_from_map();
            robot.publish_vive_pose();
            robot.print_pose(unit);
        }
        ros::spinOnce();
        rate.sleep();
    }

    deleteParam();
    std::cout << "node: " << node_name << " closed." << std::endl;

    return(0);
}
#include <cstdio>
#include <string>
#include <iostream>

#include <ros/ros.h>
// #include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

bool status = true; //tracker status 

typedef struct vivePose {
    double x, y, z;
    double W, X, Y, Z;
    double yaw, roll, pitch;
}VIVEPOSE;

class Rival {
public:
    Rival(ros::NodeHandle nh_g, ros::NodeHandle nh_p);
    void vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
    void lookup_transform_from_map();
    void publish_vive_pose(bool status);
    void print_pose(int unit_);
    void trans_vel();
    tf::Vector3 lowpass_filter(tf::Vector3 in_vel);
    bool check_status(tf::Vector3 in_vel);

private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_local;
    ros::Subscriber vel_sub;
    ros::Publisher pose_pub;
    ros::Publisher error_pub;
    // geometry_msgs::PoseWithCovarianceStamped pose;
    nav_msgs::Odometry pose;
    tf::TransformListener listener;
    tf::StampedTransform transform_from_map;
    tf::StampedTransform transform_SurviveWorldTomap;

    std::string topic_name;
    std::string node_name_;
    std::string robot_name;
    std::string tracker_frame;
    std::string map_frame;
    std::string world_frame;
    
    tf::Vector3 twist_vel;
    tf::Vector3 twist_rot;
    tf::Vector3 in_vel;
    tf::Vector3 in_rot;
    tf::Vector3 out_vel;
    tf::Vector3 last_out_vel;

    double alpha;
    double del_vel;
    double tole;
    double error_tole;

    bool has_tf;
    VIVEPOSE poseV;
};
Rival::Rival(ros::NodeHandle nh_g, ros::NodeHandle nh_p) {
    nh = nh_g;
    nh_local = nh_p;
    node_name_ = ros::this_node::getName();
    robot_name = ros::this_node::getNamespace();
    bool ok = true;
    ok &= nh_local.getParam("tracker", tracker_frame);       //path: under this node
    ok &= nh_local.getParam("map", map_frame);
    ok &= nh_local.getParam("world", world_frame);
    ok &= nh_local.getParam("topic_name", topic_name);
    ok &= nh_local.getParam("alpha", alpha);
    ok &= nh_local.getParam("del_vel", del_vel);
    ok &= nh_local.getParam("tole", tole);
    ok &= nh_local.getParam("error_tole", error_tole);

    vel_sub = nh.subscribe("tracker_vel",10, &Rival::vel_callback, this);
    pose_pub = nh.advertise<nav_msgs::Odometry>(topic_name, 10);
    error_pub = nh.advertise<std_msgs::Float64>(robot_name + "/error", 10);    

    std::cout << node_name_ << " getting parameters of the robot...\n";
    std::cout << "robot name: " << robot_name << "\n";
    std::cout << "map frame: " << map_frame << "\n";
    std::cout << "tracker: " << tracker_frame << "\n";

    if (ok) std::cout << node_name_ << " get parameters of the robot sucessed.\n";
    else std::cout << node_name_ << " get parameters of robot failed.\n";
}


void Rival::vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
    twist_vel.setValue(msg->linear.x, msg->linear.y, msg->linear.z);
    twist_rot.setValue(msg->angular.x, msg->angular.y, msg->angular.z);
}
void Rival::trans_vel(){
    try{
        listener.lookupTransform(map_frame, world_frame, ros::Time(0), transform_SurviveWorldTomap);
    }
    catch(tf::TransformException& ex) {ROS_ERROR("%s", ex.what());}
    in_rot = transform_SurviveWorldTomap.getBasis() * twist_rot;
    in_vel = transform_SurviveWorldTomap.getBasis() * twist_vel;

    out_vel = lowpass_filter(in_vel);
    status = check_status(in_vel);
}
tf::Vector3 Rival::lowpass_filter(tf::Vector3 in_vel){
    for(int i = 0; i<3; i++)
    {
        // if(abs(in_vel[i]-last_out_vel[i]) > del_vel){
        //     in_vel[i] = last_out_vel[i];
        // }
        out_vel[i] = (1-alpha)*last_out_vel[i] + alpha*in_vel[i];
        last_out_vel[i] = out_vel[i];
    }
    return out_vel;
}
bool Rival::check_status(tf::Vector3 in_vel){
    double error;
    error = sqrt(pow(abs(in_vel[0]-last_out_vel[0]),2) + pow(abs(in_vel[1]-last_out_vel[1]), 2));
    error_pub.publish(error);
    // if(error >= error_tole)
    //     return false;
    // else
    //     return true; 
    return true;
}

void Rival::lookup_transform_from_map() {
    has_tf = listener.canTransform(map_frame, tracker_frame, ros::Time(0));
    try {
        listener.lookupTransform(map_frame, tracker_frame, ros::Time(0), transform_from_map);
    }
    catch (tf::TransformException& ex) {
        printf("%s", ex.what());
        std::cout << "connot transform from " << map_frame << " to " << tracker_frame <<"\n";
    }
}
void Rival::publish_vive_pose(bool status) {
    if (has_tf) {
        pose.pose.pose.orientation.w = transform_from_map.getRotation().getW();
        pose.pose.pose.orientation.x = transform_from_map.getRotation().getX();
        pose.pose.pose.orientation.y = transform_from_map.getRotation().getY();
        pose.pose.pose.orientation.z = transform_from_map.getRotation().getZ();
        pose.pose.pose.position.x = transform_from_map.getOrigin().getX();
        pose.pose.pose.position.y = transform_from_map.getOrigin().getY();
        pose.pose.pose.position.z = transform_from_map.getOrigin().getZ();
        pose.twist.twist.linear.x = abs(out_vel[0]) > tole ? out_vel[0] : 0.0;
        pose.twist.twist.linear.y = abs(out_vel[1]) > tole ? out_vel[1] : 0.0;
        pose.twist.twist.linear.z = abs(out_vel[2]) > tole ? out_vel[2] : 0.0;
        pose.twist.twist.angular.x = 0;
        pose.twist.twist.angular.y = 0;
        pose.twist.twist.angular.z = abs(in_rot[2]) > tole ? in_rot[2] : 0.0;

        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = tracker_frame;
        if(status)
            pose_pub.publish(pose);
        else
            ROS_WARN_STREAM("Stop to publish!!");    
    }
}
void Rival::print_pose(int unit_) {
    poseV.x = transform_from_map.getOrigin().getX() * unit_;
    poseV.y = transform_from_map.getOrigin().getY() * unit_;
    poseV.z = transform_from_map.getOrigin().getZ() * unit_;
    poseV.W = transform_from_map.getRotation().getW() * unit_;
    poseV.X = transform_from_map.getRotation().getX() * unit_;
    poseV.Y = transform_from_map.getRotation().getY() * unit_;
    poseV.Z = transform_from_map.getRotation().getZ() * unit_;
    if (has_tf) {
        std::cout << robot_name << "/" << "trackerpose: " << map_frame << "->" << tracker_frame << " (x y z W X Y Z)\n";
        std::cout << poseV.x << " " << poseV.y << " " << poseV.z << " "
            << poseV.W << " " << poseV.X << " " << poseV.Y << " " << poseV.Z << "\n";
    }
    else std::cout << robot_name << "/" << map_frame << "->" << tracker_frame << " do not have tf.\n";
    printf("%s tracker vel\n",robot_name.c_str());
    printf("%5.1f, %5.1f, %5.1f\n", pose.twist.twist.linear.x, pose.twist.twist.linear.y, pose.twist.twist.linear.z);
    printf("%s tracker rota\n",robot_name.c_str());
    printf("%5.4f\n", pose.twist.twist.angular.z);
}


int freq = 20;
int unit = 1;
std::string node_name;
void initialize(ros::NodeHandle nh_) {
    bool ok = true;
    node_name = ros::this_node::getName();
    ok &= nh_.getParam("freq", freq);
    ok &= nh_.getParam("unit", unit);
    std::cout << "param: freq= " << freq << std::endl;
    std::cout << "param: unit= " << unit << std::endl;

    if (ok) std::cout << "node: " << node_name << " get parameters of node sucessed.\n";
    else std::cout << "node: " << node_name << " get parameters of node failed.\n";
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

    Rival rival(nh, nh_);

    while (ros::ok()) {
        ros::spinOnce();
        rival.trans_vel();
        rival.lookup_transform_from_map();
        rival.publish_vive_pose(status);
        rival.print_pose(unit);
        rate.sleep();
    }

    deleteParam();
    std::cout << "node: " << node_name << " closed.\n";

    return(0);
}
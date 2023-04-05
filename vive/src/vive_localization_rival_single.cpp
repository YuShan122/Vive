#include <stdio.h>
#include <string.h>
#include <survive_api.h>
#include <os_generic.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
// #include <Eigen/Eigen>
// #include <Eigen/Dense>
// #include <Eigen/Eigenvalues>
// #include <Eigen/Geometry>

static volatile int keepRunning = 1;

#ifdef __linux__

#include <assert.h>
#include <signal.h>
#include <stdlib.h>

std::string survive_frame;
std::string tracker_frame;
std::string tracker_avg_frame;
std::string map_frame;
std::string map0_frame;
std::string map1_frame;
std::string map2_frame;
std::string lh0_frame;
std::string lh1_frame;
std::string lh2_frame;

std::string tracker_SN_A = "LHR-94135635";
std::string tracker_SN_B = "LHR-15565625";
std::string tracker_SN_C = "LHR-662B1E75";
std::string tracker_SN_D = "LHR-38203A4C";
std::string tracker_SN_E = "LHR-E833C29B";
std::string tracker_serial_number;
std::string tracker_name;
std::string robot_name;
std::string topic_name;

bool status = true;

void intHandler(int dummy) {
    if (keepRunning == 0)
        exit(-1);
    keepRunning = 0;
}
#endif

typedef struct vivePose {
    double x, y, z;
    double W, X, Y, Z;
    double yaw, roll, pitch;
}VIVEPOSE;
VIVEPOSE LH0, LH1, LH2, tracker_vel, tracker_last_pose;

tf::StampedTransform transform_LH0ToMap0;
tf::StampedTransform transform_LH1ToMap1;
tf::StampedTransform transform_LH2ToMap2;
tf::StampedTransform transform_surviveWorldToLH0;
tf::StampedTransform transform_surviveWorldToLH1;
tf::StampedTransform transform_surviveWorldToLH2;
tf::StampedTransform transform_surviveWorldToTracker;
tf::StampedTransform transform_map0ToTracker;
tf::StampedTransform transform_map1ToTracker;
tf::StampedTransform transform_map2ToTracker;
tf::StampedTransform transform_map0ToSurviveWorld;
tf::StampedTransform transform_SurviveWorldTomap;
tf::StampedTransform transform_mapToTracker;

static void log_fn(SurviveSimpleContext* actx, SurviveLogLevel logLevel, const char* msg) {
    fprintf(stderr, "(%7.3f) SimpleApi: %s\n", survive_simple_run_time(actx), msg);
}

int freq = 21;
int unit = 1;
double alpha=0.01;
double del_vel=0.3;
void initialize(ros::NodeHandle nh_) {
    bool get_param_ok;
    auto node_name = ros::this_node::getName();
    robot_name = ros::this_node::getNamespace();
    nh_.getParam(node_name + "/LH0_x", LH0.x);
    nh_.getParam(node_name + "/LH0_y", LH0.y);
    nh_.getParam(node_name + "/LH0_z", LH0.z);
    nh_.getParam(node_name + "/LH0_W", LH0.W);
    nh_.getParam(node_name + "/LH0_X", LH0.X);
    nh_.getParam(node_name + "/LH0_Y", LH0.Y);
    nh_.getParam(node_name + "/LH0_Z", LH0.Z);

    nh_.getParam(node_name + "/LH1_x", LH1.x);
    nh_.getParam(node_name + "/LH1_y", LH1.y);
    nh_.getParam(node_name + "/LH1_z", LH1.z);
    nh_.getParam(node_name + "/LH1_W", LH1.W);
    nh_.getParam(node_name + "/LH1_X", LH1.X);
    nh_.getParam(node_name + "/LH1_Y", LH1.Y);
    nh_.getParam(node_name + "/LH1_Z", LH1.Z);

    nh_.getParam(node_name + "/LH2_x", LH2.x);
    nh_.getParam(node_name + "/LH2_y", LH2.y);
    nh_.getParam(node_name + "/LH2_z", LH2.z);
    nh_.getParam(node_name + "/LH2_W", LH2.W);
    nh_.getParam(node_name + "/LH2_X", LH2.X);
    nh_.getParam(node_name + "/LH2_Y", LH2.Y);
    nh_.getParam(node_name + "/LH2_Z", LH2.Z);
    nh_.getParam(node_name + "/freq", freq);
    nh_.getParam(node_name + "/unit", unit);
    nh_.getParam(node_name + "/alpha", alpha);
    nh_.getParam(node_name + "/del_vel", del_vel);
    nh_.getParam(node_name + "/tracker_name", tracker_name);
    nh_.getParam(node_name + "/topic_name", topic_name);

    survive_frame = robot_name + "/survive_world";
    tracker_frame = robot_name + "/tracker";
    tracker_avg_frame = robot_name + "/tracker_avg";
    map_frame = robot_name + "/map";
    map0_frame = robot_name + "/map0";
    map1_frame = robot_name + "/map1";
    map2_frame = robot_name + "/map2";
    lh0_frame = robot_name + "/LH0";
    lh1_frame = robot_name + "/LH1";
    lh2_frame = robot_name + "/LH2";
    
    if(strcmp("tracker_A", tracker_name.c_str()) == 0) tracker_serial_number = "LHR-94135635" ;
    if(strcmp("tracker_B", tracker_name.c_str()) == 0) tracker_serial_number = "LHR-15565625" ;
    if(strcmp("tracker_C", tracker_name.c_str()) == 0) tracker_serial_number = "LHR-662B1E75" ;
    if(strcmp("tracker_D", tracker_name.c_str()) == 0) tracker_serial_number = "LHR-38203A4C" ;
    if(strcmp("tracker_E", tracker_name.c_str()) == 0) tracker_serial_number = "LHR-E833C29B" ;

    transform_LH0ToMap0.setOrigin(tf::Vector3(LH0.x, LH0.y, LH0.z));
    transform_LH0ToMap0.setRotation(tf::Quaternion(LH0.X, LH0.Y, LH0.Z, LH0.W));
    transform_LH1ToMap1.setOrigin(tf::Vector3(LH1.x, LH1.y, LH1.z));
    transform_LH1ToMap1.setRotation(tf::Quaternion(LH1.X, LH1.Y, LH1.Z, LH1.W));
    transform_LH2ToMap2.setOrigin(tf::Vector3(LH2.x, LH2.y, LH2.z));
    transform_LH2ToMap2.setRotation(tf::Quaternion(LH2.X, LH2.Y, LH2.Z, LH2.W));
}

void deleteParam(ros::NodeHandle nh_)
{
    auto node_name = ros::this_node::getName();
    std::string deleteparam = "rosparam delete " + node_name;
    system(deleteparam.c_str());
    printf("delete param\n");
}

ros::Time last_time;
double dt;
nav_msgs::Odometry avg_pose(tf::TransformBroadcaster br_) {
    nav_msgs::Odometry vive_pose;
    vive_pose.header.frame_id = robot_name + "/tracker_frame";
    vive_pose.child_frame_id = "map";
    tf::Quaternion q;
    q = transform_map0ToTracker.getRotation().operator/(3).operator+
        (transform_map1ToTracker.getRotation().operator/(3)).operator+
        (transform_map2ToTracker.getRotation().operator/(3));
    vive_pose.pose.pose.orientation.w = q.getW();
    vive_pose.pose.pose.orientation.x = q.getX();
    vive_pose.pose.pose.orientation.y = q.getY();
    vive_pose.pose.pose.orientation.z = q.getZ();
    vive_pose.pose.pose.position.x = (double)(
        transform_map0ToTracker.getOrigin().getX() +
        transform_map1ToTracker.getOrigin().getX() +
        transform_map2ToTracker.getOrigin().getX()) / 3;
    vive_pose.pose.pose.position.y = (double)(
        transform_map0ToTracker.getOrigin().getY() +
        transform_map1ToTracker.getOrigin().getY() +
        transform_map2ToTracker.getOrigin().getY()) / 3;
    vive_pose.pose.pose.position.z = (double)(
        transform_map0ToTracker.getOrigin().getZ() +
        transform_map1ToTracker.getOrigin().getZ() +
        transform_map2ToTracker.getOrigin().getZ()) / 3;
    vive_pose.pose.pose.position.z = 0;
    vive_pose.header.stamp = ros::Time::now();
    vive_pose.header.frame_id = map_frame;
    
    transform_mapToTracker.setOrigin(tf::Vector3(
        vive_pose.pose.pose.position.x, vive_pose.pose.pose.position.y, vive_pose.pose.pose.position.z));
    transform_mapToTracker.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform_mapToTracker, ros::Time::now(), map_frame, tracker_avg_frame));

    
    printf("%s avg_pose (x y z W X Y Z)\n", robot_name.c_str());
    printf("%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n",
        vive_pose.pose.pose.position.x * unit, vive_pose.pose.pose.position.y * unit, vive_pose.pose.pose.position.z * unit,
        vive_pose.pose.pose.orientation.w, vive_pose.pose.pose.orientation.x,
        vive_pose.pose.pose.orientation.y, vive_pose.pose.pose.orientation.z);
    
    // differential
    // dt = (ros::Time::now() - last_time).toSec();
    // tracker_vel.x = (vive_pose.pose.pose.position.x - tracker_last_pose.x)/ dt;
    // tracker_vel.y = (vive_pose.pose.pose.position.y - tracker_last_pose.y)/ dt;
    // tracker_vel.z = (vive_pose.pose.pose.position.z - tracker_last_pose.z)/ dt;
    // tracker_last_pose.x = vive_pose.pose.pose.position.x;
    // tracker_last_pose.y = vive_pose.pose.pose.position.y;
    // tracker_last_pose.z = vive_pose.pose.pose.position.z;
    // last_time = ros::Time::now();
    
    return vive_pose;
}

tf::Vector3 last_out_vel;
tf::Vector3 lowpass_filter(tf::Vector3 in_vel)
{
    tf::Vector3 out_vel;
    for(int i = 0; i<3; i++)
    {
        // if(abs(in_vel[i]-last_out_vel[i])>del_vel){
        //     in_vel[i] = last_out_vel[i];
        // }
        out_vel[i] = (1-alpha)*last_out_vel[i] + alpha*in_vel[i];
        last_out_vel[i] = out_vel[i];
    }
    return out_vel;
}

int main(int argc, char** argv) {
#ifdef __linux__
    signal(SIGINT, intHandler);
    signal(SIGTERM, intHandler);
    signal(SIGKILL, intHandler);
#endif

    ros::init(argc, argv, "vive_localization_rival_single");
    ros::NodeHandle nh;
    tf::TransformBroadcaster br;
    tf::TransformListener listener;

    initialize(nh);
    ros::Publisher pose_pub = nh.advertise<nav_msgs::Odometry>(robot_name + topic_name, 10);
    ros::Rate rate(freq);
    nav_msgs::Odometry pose_;
    last_time = ros::Time::now();
    SurviveSimpleContext* actx = survive_simple_init_with_logger(argc, argv, log_fn);
    if (actx == 0) // implies -help or similiar
        return 0;

    double start_time = OGGetAbsoluteTime();
    survive_simple_start_thread(actx);

    for (const SurviveSimpleObject* it = survive_simple_get_first_object(actx); it != 0;
        it = survive_simple_get_next_object(actx, it)) {
        printf("Found '%s'\n", survive_simple_object_name(it));
    }

    struct SurviveSimpleEvent event = {};
    while (keepRunning && survive_simple_wait_for_event(actx, &event) != SurviveSimpleEventType_Shutdown && ros::ok()) {
        SurvivePose pose;
        SurviveVelocity velocity;
        VIVEPOSE vel;
        switch (event.event_type) {
        case SurviveSimpleEventType_PoseUpdateEvent: {
            for (const SurviveSimpleObject* it = survive_simple_get_first_object(actx);
                it != 0; it = survive_simple_get_next_object(actx, it)) {

                survive_simple_object_get_latest_pose(it, &pose);
                if (survive_simple_object_get_type(it) == SurviveSimpleObject_OBJECT && strcmp(survive_simple_serial_number(it), tracker_serial_number.c_str()) == 0) {
                    survive_simple_object_get_latest_velocity(it, &velocity);
                    // printf("%s velocity : \nx : %f\ny : %f\nz : %f\n", survive_simple_object_name(it), velocity.Pos[0], velocity.Pos[1], velocity.Pos[2]);
                    transform_surviveWorldToTracker.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                    transform_surviveWorldToTracker.setRotation(tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));

                    vel.x = velocity.Pos[0];
                    vel.y = velocity.Pos[1];
                    vel.z = velocity.Pos[2];
                    vel.pitch = velocity.AxisAngleRot[0];
                    vel.roll = velocity.AxisAngleRot[1];
                    vel.yaw = velocity.AxisAngleRot[2];
                    printf("api velocity\n");
                    printf("%8.4f %8.4f %8.4f\n",vel.x,vel.y,vel.z);
                }
                else if (survive_simple_object_get_type(it) == SurviveSimpleObject_LIGHTHOUSE) {
                    if (strcmp(survive_simple_serial_number(it), "LHB-400B1A3E") == 0) {
                        transform_surviveWorldToLH0.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                        transform_surviveWorldToLH0.setRotation(tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                    }
                    else if (strcmp(survive_simple_serial_number(it), "LHB-D4EEE18") == 0) {
                        transform_surviveWorldToLH1.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                        transform_surviveWorldToLH1.setRotation(tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                    }
                    else if (strcmp(survive_simple_serial_number(it), "LHB-2BEE096A") == 0) {
                        transform_surviveWorldToLH2.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                        transform_surviveWorldToLH2.setRotation(tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                    }
                }
            }
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToTracker, ros::Time::now(), survive_frame, tracker_frame));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH0, ros::Time::now(), survive_frame, lh0_frame));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH1, ros::Time::now(), survive_frame, lh1_frame));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH2, ros::Time::now(), survive_frame, lh2_frame));
            br.sendTransform(tf::StampedTransform(transform_LH0ToMap0, ros::Time::now(), lh0_frame, map0_frame));
            br.sendTransform(tf::StampedTransform(transform_LH1ToMap1, ros::Time::now(), lh1_frame, map1_frame));
            br.sendTransform(tf::StampedTransform(transform_LH2ToMap2, ros::Time::now(), lh2_frame, map2_frame));
            try {
                listener.lookupTransform(map0_frame, tracker_frame, ros::Time(0), transform_map0ToTracker);
                listener.lookupTransform(map1_frame, tracker_frame, ros::Time(0), transform_map1ToTracker);
                listener.lookupTransform(map2_frame, tracker_frame, ros::Time(0), transform_map2ToTracker);
                listener.lookupTransform(map0_frame, survive_frame, ros::Time(0), transform_map0ToSurviveWorld);
            }
            catch (tf::TransformException& ex) {
                ROS_ERROR("%s", ex.what());
            }
            // printf("transform: map0 to tracker\n");
            // printf("%7.4f, %7.4f, %7.4f\n", transform_map0ToTracker.getOrigin().x() * unit,
            //     transform_map0ToTracker.getOrigin().y() * unit, transform_map0ToTracker.getOrigin().z() * unit);
            // printf("transform: map1 to tracker\n");
            // printf("%7.4f, %7.4f, %7.4f\n", transform_map1ToTracker.getOrigin().x() * unit,
            //     transform_map1ToTracker.getOrigin().y() * unit, transform_map1ToTracker.getOrigin().z() * unit);
            // printf("transform: map2 to tracker\n");
            // printf("%7.4f, %7.4f, %7.4f\n", transform_map2ToTracker.getOrigin().x() * unit,
            //     transform_map2ToTracker.getOrigin().y() * unit, transform_map2ToTracker.getOrigin().z() * unit);

            break;
        }
        }
        pose_ = avg_pose(br);
        
        try{
            listener.lookupTransform(map0_frame,survive_frame, ros::Time(0), transform_SurviveWorldTomap);
        }
        catch(tf::TransformException& ex){ ROS_ERROR("%s", ex.what()); }
        double tole = 0.1;
        tf::Vector3 twist_vel(vel.x, vel.y, vel.z);
        tf::Vector3 twist_rot(vel.roll, vel.pitch, vel.yaw);

        tf::Vector3 out_rot = transform_SurviveWorldTomap.getBasis() * twist_rot;
        // tf::Vector3 out_vel = transform_SurviveWorldTomap.getBasis() * twist_vel + transform_SurviveWorldTomap.getOrigin().cross(out_rot);
        tf::Vector3 out_vel = transform_SurviveWorldTomap.getBasis() * twist_vel ;

        out_vel = lowpass_filter(out_vel);
        pose_.twist.twist.linear.x = abs(out_vel[0]) > tole ? out_vel[0] : 0.0;
        pose_.twist.twist.linear.y = abs(out_vel[1]) > tole ? out_vel[1] : 0.0;
        pose_.twist.twist.linear.z = abs(out_vel[2]) > tole ? out_vel[2] : 0.0;
        pose_.twist.twist.angular.x = 0;
        pose_.twist.twist.angular.y = 0;
        pose_.twist.twist.angular.z = abs(out_rot[2]) > tole ? out_rot[2] : 0.0;
        
        // differential the pose to get the velocity 
        // int resolution = 1;
        // pose_.twist.twist.linear.x = abs(tracker_vel.x) > tole ? int(tracker_vel.x/resolution)*resolution : 0.0;
        // pose_.twist.twist.linear.y = abs(tracker_vel.y) > tole ? int(tracker_vel.y/resolution)*resolution : 0.0;
        // pose_.twist.twist.linear.z = abs(tracker_vel.z) > tole ? int(tracker_vel.z/resolution)*resolution : 0.0;

        printf("%s tracker vel\n",robot_name.c_str());
        printf("%5.1f, %5.1f, %5.1f\n", pose_.twist.twist.linear.x, pose_.twist.twist.linear.y, pose_.twist.twist.linear.z);
        printf("%s tracker rota\n",robot_name.c_str());
        printf("%5.4f\n", pose_.twist.twist.angular.z);

        pose_pub.publish(pose_);
        rate.sleep();
    }

    deleteParam(nh);
    survive_simple_close(actx);
    printf("Cleaning up\n");
    return 0;
}



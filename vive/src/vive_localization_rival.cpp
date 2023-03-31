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

static volatile int keepRunning = 1;

#ifdef __linux__

#include <assert.h>
#include <signal.h>
#include <stdlib.h>


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

VIVEPOSE LH0, LH1, LH2;
int freq = 21;
int unit = 1;
double tole = 0.1;
double alpha = 0.1;
bool debug = false;

std::string tracker_SN_A = "LHR-94135635";
std::string tracker_SN_B = "LHR-15565625";
std::string tracker_SN_C = "LHR-662B1E75";
std::string tracker_SN_D = "LHR-38203A4C";
std::string tracker_SN_E = "LHR-E833C29B";
std::string tracker_SerialName_rival1 = tracker_SN_A;
std::string tracker_SerialName_rival2 = tracker_SN_B;
std::string tracker_name_rival1 = "tracker_rival1";
std::string tracker_name_rival2 = "tracker_rival2";

tf::StampedTransform transform_LH0ToMap0;
tf::StampedTransform transform_LH1ToMap1;
tf::StampedTransform transform_LH2ToMap2;
tf::StampedTransform transform_surviveWorldToLH0;
tf::StampedTransform transform_surviveWorldToLH1;
tf::StampedTransform transform_surviveWorldToLH2;
tf::StampedTransform transform_surviveWorldToTracker_A;
tf::StampedTransform transform_surviveWorldToTracker_B;
tf::StampedTransform transform_map0ToTracker_A;
tf::StampedTransform transform_map1ToTracker_A;
tf::StampedTransform transform_map2ToTracker_A;
tf::StampedTransform transform_map0ToTracker_B;
tf::StampedTransform transform_map1ToTracker_B;
tf::StampedTransform transform_map2ToTracker_B;
tf::StampedTransform transform_mapToTracker;
tf::StampedTransform transform_SurviveWorldTomap;

static void log_fn(SurviveSimpleContext* actx, SurviveLogLevel logLevel, const char* msg) {
    fprintf(stderr, "(%7.3f) SimpleApi: %s\n", survive_simple_run_time(actx), msg);
}

void initialize(ros::NodeHandle nh_) {
    bool get_param_ok;
    auto node_name = ros::this_node::getName();
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

    nh_.getParam(node_name + "/tole", tole);
    nh_.getParam(node_name + "/alpha", alpha);
    nh_.getParam(node_name + "/debug", debug);

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

tf::StampedTransform transform_map0ToTracker;
tf::StampedTransform transform_map1ToTracker;
tf::StampedTransform transform_map2ToTracker;
nav_msgs::Odometry avg_pose(tf::TransformBroadcaster br_,std::string name) {
    nav_msgs::Odometry vive_pose;
    vive_pose.header.frame_id = "tracker_frame" + name;
    vive_pose.child_frame_id = "map";
    if(strcmp(name.c_str(),tracker_name_rival1.c_str() ) == 0){
        transform_map0ToTracker = transform_map0ToTracker_A;
        transform_map1ToTracker = transform_map1ToTracker_A;
        transform_map2ToTracker = transform_map2ToTracker_A;
    }
    else if (strcmp(name.c_str(), tracker_name_rival2.c_str()) == 0){
        transform_map0ToTracker = transform_map0ToTracker_B;
        transform_map1ToTracker = transform_map1ToTracker_B;
        transform_map2ToTracker = transform_map2ToTracker_B;
    }
     
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
    vive_pose.header.frame_id = "map";
    
    transform_mapToTracker.setOrigin(tf::Vector3(
        vive_pose.pose.pose.position.x, vive_pose.pose.pose.position.y, vive_pose.pose.pose.position.z));
    transform_mapToTracker.setRotation(q);
    // br_.sendTransform(tf::StampedTransform(transform_mapToTracker, ros::Time::now(), "map", name));

    printf("transform: (x y z W X Y Z)\n");
    printf("%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n",
        vive_pose.pose.pose.position.x * unit, vive_pose.pose.pose.position.y * unit, vive_pose.pose.pose.position.z * unit,
        vive_pose.pose.pose.orientation.w, vive_pose.pose.pose.orientation.x,
        vive_pose.pose.pose.orientation.y, vive_pose.pose.pose.orientation.z);
    
    return vive_pose;
}

tf::Vector3 rival1_last_out_vel, rival2_last_out_vel;
tf::Vector3 lowpass_filter(tf::Vector3 in_vel, tf::Vector3 last_out_vel)
{
    tf::Vector3 out_vel;
    for(int i = 0; i<3; i++)
    {
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

    ros::init(argc, argv, "vive_localization_rival");
    ros::NodeHandle nh;
    ros::Publisher rival1_pub = nh.advertise<nav_msgs::Odometry>("rival1/odom", 10);
    ros::Publisher rival2_pub = nh.advertise<nav_msgs::Odometry>("rival2/odom", 10);
    tf::TransformBroadcaster br;
    tf::TransformListener listener;
    nav_msgs::Odometry rival1_pose;
    nav_msgs::Odometry rival2_pose;

    initialize(nh);
    ros::Rate rate(freq);
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
        VIVEPOSE rival1_vel, rival2_vel;
        switch (event.event_type) {
        case SurviveSimpleEventType_PoseUpdateEvent: {
            for (const SurviveSimpleObject* it = survive_simple_get_first_object(actx);
                it != 0; it = survive_simple_get_next_object(actx, it)) {
                
                survive_simple_object_get_latest_pose(it, &pose);
                if (survive_simple_object_get_type(it) == SurviveSimpleObject_OBJECT) {
                    survive_simple_object_get_latest_velocity(it, &velocity);
                    // printf("%s velocity : \nx : %f\ny : %f\nz : %f\n", survive_simple_object_name(it), velocity.Pos[0], velocity.Pos[1], velocity.Pos[2]);
                    if(strcmp(survive_simple_serial_number(it), tracker_SerialName_rival1.c_str()) == 0){
                        transform_surviveWorldToTracker_A.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                        transform_surviveWorldToTracker_A.setRotation(tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                        rival1_vel.x = velocity.Pos[0];
                        rival1_vel.y = velocity.Pos[1];
                        rival1_vel.z = velocity.Pos[2];
                        rival1_vel.pitch = velocity.AxisAngleRot[0];
                        rival1_vel.roll = velocity.AxisAngleRot[1];
                        rival1_vel.yaw = velocity.AxisAngleRot[2];
                        if(debug){
                            printf("API rival1 velocity\n");
                            printf("%8.4f %8.4f %8.4f\n",rival1_vel.x,rival1_vel.y,rival1_vel.z);
                        } 
                    }
                    else if(strcmp(survive_simple_serial_number(it), tracker_SerialName_rival2.c_str()) == 0){
                        transform_surviveWorldToTracker_B.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                        transform_surviveWorldToTracker_B.setRotation(tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                        rival2_vel.x = velocity.Pos[0];
                        rival2_vel.y = velocity.Pos[1];
                        rival2_vel.z = velocity.Pos[2];
                        rival2_vel.pitch = velocity.AxisAngleRot[0];
                        rival2_vel.roll = velocity.AxisAngleRot[1];
                        rival2_vel.yaw = velocity.AxisAngleRot[2];
                        if(debug){
                            printf("API rival1 velocity\n");
                            printf("%8.4f %8.4f %8.4f\n",rival2_vel.x,rival2_vel.y,rival2_vel.z);
                        }
                    }
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
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToTracker_A, ros::Time::now(), "survive_world", tracker_name_rival1));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToTracker_B, ros::Time::now(), "survive_world", tracker_name_rival2));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH0, ros::Time::now(), "survive_world", "LH0"));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH1, ros::Time::now(), "survive_world", "LH1"));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH2, ros::Time::now(), "survive_world", "LH2"));
            br.sendTransform(tf::StampedTransform(transform_LH0ToMap0, ros::Time::now(), "LH0", "map0"));
            br.sendTransform(tf::StampedTransform(transform_LH1ToMap1, ros::Time::now(), "LH1", "map1"));
            br.sendTransform(tf::StampedTransform(transform_LH2ToMap2, ros::Time::now(), "LH2", "map2"));
            try {
                listener.lookupTransform("map0", tracker_name_rival1, ros::Time(0), transform_map0ToTracker_A);
                listener.lookupTransform("map1", tracker_name_rival1, ros::Time(0), transform_map1ToTracker_A);
                listener.lookupTransform("map2", tracker_name_rival1, ros::Time(0), transform_map2ToTracker_A);
                listener.lookupTransform("map0", tracker_name_rival2, ros::Time(0), transform_map0ToTracker_B);
                listener.lookupTransform("map1", tracker_name_rival2, ros::Time(0), transform_map1ToTracker_B);
                listener.lookupTransform("map2", tracker_name_rival2, ros::Time(0), transform_map2ToTracker_B);
            }
            catch (tf::TransformException& ex) {
                ROS_ERROR("%s", ex.what());
            }

            if(debug){
                printf("transform: map0 to tracker_A\n");
                printf("%f, %f, %f\n", transform_map0ToTracker_A.getOrigin().x() * unit,
                    transform_map0ToTracker_A.getOrigin().y() * unit, transform_map0ToTracker_A.getOrigin().z() * unit);
                printf("transform: map1 to tracker_A\n");
                printf("%f, %f, %f\n", transform_map1ToTracker_A.getOrigin().x() * unit,
                    transform_map1ToTracker_A.getOrigin().y() * unit, transform_map1ToTracker_A.getOrigin().z() * unit);
                printf("transform: map2 to tracker_A\n");
                printf("%f, %f, %f\n", transform_map2ToTracker_A.getOrigin().x() * unit,
                    transform_map2ToTracker_A.getOrigin().y() * unit, transform_map2ToTracker_A.getOrigin().z() * unit);
                
                printf("transform: map0 to tracker_B\n");
                printf("%f, %f, %f\n", transform_map0ToTracker_B.getOrigin().x() * unit,
                    transform_map0ToTracker_B.getOrigin().y() * unit, transform_map0ToTracker_B.getOrigin().z() * unit);
                printf("transform: map1 to tracker_B\n");
                printf("%f, %f, %f\n", transform_map1ToTracker_B.getOrigin().x() * unit,
                    transform_map1ToTracker_B.getOrigin().y() * unit, transform_map1ToTracker_B.getOrigin().z() * unit);
                printf("transform: map2 to tracker_B\n");
                printf("%f, %f, %f\n", transform_map2ToTracker_B.getOrigin().x() * unit,
                    transform_map2ToTracker_B.getOrigin().y() * unit, transform_map2ToTracker_B.getOrigin().z() * unit);
            }
            break;
        }
        }
        
        rival1_pose = avg_pose(br, tracker_name_rival1);
        rival2_pose = avg_pose(br, tracker_name_rival2);
        try{
            listener.lookupTransform("map0","survive_world", ros::Time(0), transform_SurviveWorldTomap);
        }
        catch(tf::TransformException& ex){ ROS_ERROR("%s", ex.what()); }

        // set up rotation matrix equation & trans
        tf::Vector3 twist_rival1_vel(rival1_vel.x, rival1_vel.y, rival1_vel.z);
        tf::Vector3 twist_rival1_rot(rival1_vel.roll, rival1_vel.pitch, rival1_vel.yaw);
        tf::Vector3 out_rival1_rot = transform_SurviveWorldTomap.getBasis() * twist_rival1_rot;
        tf::Vector3 out_rival1_vel = transform_SurviveWorldTomap.getBasis() * twist_rival1_vel ;

        tf::Vector3 twist_rival2_vel(rival2_vel.x, rival2_vel.y, rival2_vel.z);
        tf::Vector3 twist_rival2_rot(rival2_vel.roll, rival2_vel.pitch, rival2_vel.yaw);
        tf::Vector3 out_rival2_rot = transform_SurviveWorldTomap.getBasis() * twist_rival2_rot;
        tf::Vector3 out_rival2_vel = transform_SurviveWorldTomap.getBasis() * twist_rival2_vel ;

        out_rival1_vel = lowpass_filter(out_rival1_vel,rival1_last_out_vel);
        out_rival2_vel = lowpass_filter(out_rival2_vel,rival2_last_out_vel);

        rival1_pose.twist.twist.linear.x = abs(out_rival1_vel[0]) > tole ? out_rival1_vel[0] : 0.0;
        rival1_pose.twist.twist.linear.y = abs(out_rival1_vel[1]) > tole ? out_rival1_vel[1] : 0.0;
        rival1_pose.twist.twist.linear.z = 0.0;
        rival1_pose.twist.twist.angular.x = 0.0;
        rival1_pose.twist.twist.angular.y = 0.0;
        rival1_pose.twist.twist.angular.z = out_rival1_rot[2];

        rival2_pose.twist.twist.linear.x = abs(out_rival2_vel[0]) > tole ? out_rival2_vel[0] : 0.0;
        rival2_pose.twist.twist.linear.y = abs(out_rival2_vel[1]) > tole ? out_rival2_vel[1] : 0.0;
        rival2_pose.twist.twist.linear.z = 0.0;
        rival2_pose.twist.twist.angular.x = 0.0;
        rival2_pose.twist.twist.angular.y = 0.0;
        rival2_pose.twist.twist.angular.z = out_rival2_rot[2];

        printf("rival1 vel\n");
        printf("%4.2f, %4.2f, %4.2f, %4.2f\n", rival1_pose.twist.twist.linear.x, rival1_pose.twist.twist.linear.y, rival1_pose.twist.twist.linear.z, rival1_pose.twist.twist.angular.z);
        printf("rival2 vel\n");
        printf("%4.2f, %4.2f, %4.2f, %4.2f\n", rival2_pose.twist.twist.linear.x, rival2_pose.twist.twist.linear.y, rival2_pose.twist.twist.linear.z, rival2_pose.twist.twist.angular.z);
        
        rival1_pub.publish(rival1_pose);
        rival2_pub.publish(rival2_pose);
        rate.sleep();
    }

    deleteParam(nh);
    survive_simple_close(actx);
    printf("Cleaning up\n");
    return 0;
}



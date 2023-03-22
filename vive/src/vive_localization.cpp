#include <stdio.h>
#include <string.h>
#include <survive_api.h>
#include <os_generic.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
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
int freq = 21;
int unit = 1;

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

ros::Time last_time;
double dt;

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

nav_msgs::Odometry avg_pose(tf::TransformBroadcaster br_) {
    nav_msgs::Odometry vive_pose;
    vive_pose.header.frame_id = "tracker_frame";
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
    vive_pose.header.frame_id = "map";
    
    transform_mapToTracker.setOrigin(tf::Vector3(
        vive_pose.pose.pose.position.x, vive_pose.pose.pose.position.y, vive_pose.pose.pose.position.z));
    transform_mapToTracker.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform_mapToTracker, ros::Time::now(), "map", "tracker_avg"));

    printf("transform: avg_pose (x y z W X Y Z)\n");
    printf("%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n",
        vive_pose.pose.pose.position.x * unit, vive_pose.pose.pose.position.y * unit, vive_pose.pose.pose.position.z * unit,
        vive_pose.pose.pose.orientation.w, vive_pose.pose.pose.orientation.x,
        vive_pose.pose.pose.orientation.y, vive_pose.pose.pose.orientation.z);
    
    // differential
    dt = (ros::Time::now() - last_time).toSec();
    tracker_vel.x = (vive_pose.pose.pose.position.x - tracker_last_pose.x)/ dt;
    tracker_vel.y = (vive_pose.pose.pose.position.y - tracker_last_pose.y)/ dt;
    tracker_vel.z = (vive_pose.pose.pose.position.z - tracker_last_pose.z)/ dt;
    tracker_last_pose.x = vive_pose.pose.pose.position.x;
    tracker_last_pose.y = vive_pose.pose.pose.position.y;
    tracker_last_pose.z = vive_pose.pose.pose.position.z;
    last_time = ros::Time::now();
    
    return vive_pose;
}

int main(int argc, char** argv) {
#ifdef __linux__
    signal(SIGINT, intHandler);
    signal(SIGTERM, intHandler);
    signal(SIGKILL, intHandler);
#endif

    ros::init(argc, argv, "vive_localization");
    ros::NodeHandle nh;
    ros::Publisher pose_pub = nh.advertise<nav_msgs::Odometry>("RivalOdom_1", 10);
    tf::TransformBroadcaster br;
    tf::TransformListener listener;

    initialize(nh);
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
        // VIVEPOSE vel, vel_, vel_minus;
        VIVEPOSE vel;
        switch (event.event_type) {
        case SurviveSimpleEventType_PoseUpdateEvent: {
            for (const SurviveSimpleObject* it = survive_simple_get_first_object(actx);
                it != 0; it = survive_simple_get_next_object(actx, it)) {

                survive_simple_object_get_latest_pose(it, &pose);
                if (survive_simple_object_get_type(it) == SurviveSimpleObject_OBJECT) {
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
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToTracker, ros::Time::now(), "survive_world", "tracker"));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH0, ros::Time::now(), "survive_world", "LH0"));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH1, ros::Time::now(), "survive_world", "LH1"));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH2, ros::Time::now(), "survive_world", "LH2"));
            br.sendTransform(tf::StampedTransform(transform_LH0ToMap0, ros::Time::now(), "LH0", "map0"));
            br.sendTransform(tf::StampedTransform(transform_LH1ToMap1, ros::Time::now(), "LH1", "map1"));
            br.sendTransform(tf::StampedTransform(transform_LH2ToMap2, ros::Time::now(), "LH2", "map2"));
            try {
                listener.lookupTransform("map0", "tracker", ros::Time(0), transform_map0ToTracker);
                listener.lookupTransform("map1", "tracker", ros::Time(0), transform_map1ToTracker);
                listener.lookupTransform("map2", "tracker", ros::Time(0), transform_map2ToTracker);
                listener.lookupTransform("map0", "survive_world", ros::Time(0), transform_map0ToSurviveWorld);
            }
            catch (tf::TransformException& ex) {
                ROS_ERROR("%s", ex.what());
            }
            printf("transform: map0 to tracker\n");
            printf("%7.4f, %7.4f, %7.4f\n", transform_map0ToTracker.getOrigin().x() * unit,
                transform_map0ToTracker.getOrigin().y() * unit, transform_map0ToTracker.getOrigin().z() * unit);
            printf("transform: map1 to tracker\n");
            printf("%7.4f, %7.4f, %7.4f\n", transform_map1ToTracker.getOrigin().x() * unit,
                transform_map1ToTracker.getOrigin().y() * unit, transform_map1ToTracker.getOrigin().z() * unit);
            printf("transform: map2 to tracker\n");
            printf("%7.4f, %7.4f, %7.4f\n", transform_map2ToTracker.getOrigin().x() * unit,
                transform_map2ToTracker.getOrigin().y() * unit, transform_map2ToTracker.getOrigin().z() * unit);

            break;
        }
        }

        // vel_minus.x = transform_map0ToSurviveWorld.getOrigin().x();
        // vel_minus.y = sin(2);


        pose_ = avg_pose(br);
        
        try{
            listener.lookupTransform("survive_world","map0", ros::Time(0), transform_SurviveWorldTomap);
        }
        catch(tf::TransformException& ex){ ROS_ERROR("%s", ex.what()); }
        double tole = 0.0005;
        tf::Matrix3x3 R(transform_SurviveWorldTomap.getRotation());

        printf("trans matrix\n%8.4f, %8.4f, %8.4f\n%8.4f, %8.4f, %8.4f\n%8.4f, %8.4f, %8.4f\n",R[0][0],R[0][1],R[0][2],R[1][0],R[1][1],R[1][2],R[2][0],R[2][1],R[2][2]);
        // survive_world frame trans to map to get the tracker velocity
        // tf::Vector3 vel_(vel.x, vel.y, vel.z);
        // tf::Vector3 vector_vel = R * vel_;
        // pose_.twist.twist.linear.x = vector_vel[0] > tole ? vector_vel[0] : 0.0;
        // pose_.twist.twist.linear.y = vector_vel[1] > tole ? vector_vel[0] : 0.0;
        // pose_.twist.twist.linear.z = vector_vel[2] > tole ? vector_vel[0] : 0.0;
        
        // differential the pose to get the velocity 
        int resolution = 50;
        pose_.twist.twist.linear.x = abs(tracker_vel.x) > tole ? int(tracker_vel.x/resolution)*resolution : 0.0;
        pose_.twist.twist.linear.y = abs(tracker_vel.y) > tole ? int(tracker_vel.y/resolution)*resolution : 0.0;
        pose_.twist.twist.linear.z = abs(tracker_vel.z) > tole ? int(tracker_vel.z/resolution)*resolution : 0.0;
        
        // survive_world frame vel
        // pose_.twist.twist.linear.x = vel.x;
        // pose_.twist.twist.linear.y = vel.y;
        // pose_.twist.twist.linear.z = vel.z;

        tf::Vector3 rot_(vel.roll, vel.pitch, vel.yaw);
        tf::Vector3 vector_rot = R * rot_;
        pose_.twist.twist.angular.x = 0;
        pose_.twist.twist.angular.y = 0;
        pose_.twist.twist.angular.z = abs(vector_rot[2]) > tole ? vector_rot[2] : 0.0;
        
        printf("tracker vel\n");
        printf("%5.4f, %5.4f, %5.4f\n", pose_.twist.twist.linear.x*unit, pose_.twist.twist.linear.y*unit, pose_.twist.twist.linear.z*unit);
        printf("tracker rota\n");
        printf("%5.4f\n", pose_.twist.twist.angular.z);

        pose_pub.publish(pose_);
        rate.sleep();
    }

    deleteParam(nh);
    survive_simple_close(actx);
    printf("Cleaning up\n");
    return 0;
}



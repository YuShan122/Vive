#include <stdio.h>
#include <string.h>
#include <survive_api.h>
#include <os_generic.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

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

std::string tracker_SN_A = "LHR-94135635";
std::string tracker_SN_B = "LHR-15565625";
std::string tracker_SN_C = "LHR-662B1E75";
std::string tracker_SN_D = "LHR-38203A4C";
std::string tracker_SN_E = "LHR-E833C29B";

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

geometry_msgs::PoseStamped avg_pose(tf::TransformBroadcaster br_) {
    geometry_msgs::PoseStamped pose_avg;
    tf::Quaternion q;
    q = transform_map0ToTracker_A.getRotation().operator/(3).operator+
        (transform_map1ToTracker_A.getRotation().operator/(3)).operator+
        (transform_map2ToTracker_A.getRotation().operator/(3));
    pose_avg.pose.orientation.w = q.getW();
    pose_avg.pose.orientation.x = q.getX();
    pose_avg.pose.orientation.y = q.getY();
    pose_avg.pose.orientation.z = q.getZ();
    pose_avg.pose.position.x = (double)(
        transform_map0ToTracker_A.getOrigin().getX() +
        transform_map1ToTracker_A.getOrigin().getX() +
        transform_map2ToTracker_A.getOrigin().getX()) / 3;
    pose_avg.pose.position.y = (double)(
        transform_map0ToTracker_A.getOrigin().getY() +
        transform_map1ToTracker_A.getOrigin().getY() +
        transform_map2ToTracker_A.getOrigin().getY()) / 3;
    pose_avg.pose.position.z = (double)(
        transform_map0ToTracker_A.getOrigin().getZ() +
        transform_map1ToTracker_A.getOrigin().getZ() +
        transform_map2ToTracker_A.getOrigin().getZ()) / 3;
    pose_avg.header.stamp = ros::Time::now();
    pose_avg.header.frame_id = "map";

    transform_mapToTracker.setOrigin(tf::Vector3(
        pose_avg.pose.position.x, pose_avg.pose.position.y, pose_avg.pose.position.z));
    transform_mapToTracker.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform_mapToTracker, ros::Time::now(), "map", "tracker_avg"));

    printf("transform: avg_pose (x y z W X Y Z)\n");
    printf("%f, %f, %f, %f, %f, %f, %f\n",
        pose_avg.pose.position.x, pose_avg.pose.position.y, pose_avg.pose.position.z,
        pose_avg.pose.orientation.w, pose_avg.pose.orientation.x,
        pose_avg.pose.orientation.y, pose_avg.pose.orientation.z);
    return pose_avg;
}

int main(int argc, char** argv) {
#ifdef __linux__
    signal(SIGINT, intHandler);
    signal(SIGTERM, intHandler);
    signal(SIGKILL, intHandler);
#endif

    ros::init(argc, argv, "vive_localization");
    ros::NodeHandle nh;
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("vive_pose", 10);
    tf::TransformBroadcaster br;
    tf::TransformListener listener;

    initialize(nh);
    ros::Rate rate(freq);
    geometry_msgs::PoseStamped vive_pose;
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
        switch (event.event_type) {
        case SurviveSimpleEventType_PoseUpdateEvent: {
            for (const SurviveSimpleObject* it = survive_simple_get_first_object(actx);
                it != 0; it = survive_simple_get_next_object(actx, it)) {
                SurvivePose pose;
                SurviveVelocity velocity;
                survive_simple_object_get_latest_pose(it, &pose);
                printf("%s, %s, %f, %f, %f, %f, %f, %f, %f\n",
                    survive_simple_object_name(it), survive_simple_serial_number(it),
                    pose.Pos[0], pose.Pos[1], pose.Pos[2],
                    pose.Rot[0], pose.Rot[1], pose.Rot[2], pose.Rot[3]);
                if (survive_simple_object_get_type(it) == SurviveSimpleObject_OBJECT) {
                    survive_simple_object_get_latest_velocity(it, &velocity);
                    // printf("%s velocity : \nx : %f\ny : %f\nz : %f\n", survive_simple_object_name(it), velocity.Pos[0], velocity.Pos[1], velocity.Pos[2]);
                    if(strcmp(survive_simple_serial_number(it), tracker_SN_A.c_str()) == 0)
                    {
                        transform_surviveWorldToTracker_A.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                        transform_surviveWorldToTracker_A.setRotation(tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                    }
                    else if(strcmp(survive_simple_serial_number(it), tracker_SN_B.c_str()) == 0)
                    {
                        transform_surviveWorldToTracker_B.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                        transform_surviveWorldToTracker_B.setRotation(tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
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
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToTracker_A, ros::Time::now(), "survive_world", "tracker_A"));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToTracker_B, ros::Time::now(), "survive_world", "tracker_B"));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH0, ros::Time::now(), "survive_world", "LH0"));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH1, ros::Time::now(), "survive_world", "LH1"));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH2, ros::Time::now(), "survive_world", "LH2"));
            br.sendTransform(tf::StampedTransform(transform_LH0ToMap0, ros::Time::now(), "LH0", "map0"));
            br.sendTransform(tf::StampedTransform(transform_LH1ToMap1, ros::Time::now(), "LH1", "map1"));
            br.sendTransform(tf::StampedTransform(transform_LH2ToMap2, ros::Time::now(), "LH2", "map2"));
            try {
                listener.lookupTransform("map0", "tracker_A", ros::Time(0), transform_map0ToTracker_A);
                listener.lookupTransform("map1", "tracker_A", ros::Time(0), transform_map1ToTracker_A);
                listener.lookupTransform("map2", "tracker_A", ros::Time(0), transform_map2ToTracker_A);
                listener.lookupTransform("map0", "tracker_B", ros::Time(0), transform_map0ToTracker_B);
                listener.lookupTransform("map1", "tracker_B", ros::Time(0), transform_map1ToTracker_B);
                listener.lookupTransform("map2", "tracker_B", ros::Time(0), transform_map2ToTracker_B);
            }
            catch (tf::TransformException& ex) {
                ROS_ERROR("%s", ex.what());
            }
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
            
            break;
        }
        }
        vive_pose.pose.orientation.w = transform_map0ToTracker_A.getRotation().getW();
        vive_pose.pose.orientation.x = transform_map0ToTracker_A.getRotation().getX();
        vive_pose.pose.orientation.y = transform_map0ToTracker_A.getRotation().getY();
        vive_pose.pose.orientation.z = transform_map0ToTracker_A.getRotation().getZ();
        vive_pose.pose.position.x = transform_map0ToTracker_A.getOrigin().getX();
        vive_pose.pose.position.y = transform_map0ToTracker_A.getOrigin().getY();
        vive_pose.pose.position.z = transform_map0ToTracker_A.getOrigin().getZ();
        vive_pose.header.stamp = ros::Time::now();
        vive_pose.header.frame_id = "map";
        // pose_pub.publish(vive_pose);
        pose_pub.publish(avg_pose(br));
        rate.sleep();
    }

    deleteParam(nh);
    survive_simple_close(actx);
    printf("Cleaning up\n");
    return 0;
}



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

VIVEPOSE LH0, LH1;
int freq = 100;
int unit = 1;

tf::StampedTransform transform_LH0ToMap0;
tf::StampedTransform transform_LH1ToMap1;
tf::StampedTransform transform_surviveWorldToLH0;
tf::StampedTransform transform_surviveWorldToLH1;
tf::StampedTransform transform_surviveWorldToTracker;
tf::StampedTransform transform_map0ToTracker;
tf::StampedTransform transform_map1ToTracker;

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
    nh_.getParam(node_name + "/freq",freq);
    nh_.getParam(node_name + "/unit", unit);

    transform_LH0ToMap0.setOrigin(tf::Vector3(LH0.x, LH0.y, LH0.z));
    transform_LH0ToMap0.setRotation(tf::Quaternion(LH0.X, LH0.Y, LH0.Z, LH0.W));
    transform_LH1ToMap1.setOrigin(tf::Vector3(LH1.x, LH1.y, LH1.z));
    transform_LH1ToMap1.setRotation(tf::Quaternion(LH1.X, LH1.Y, LH1.Z, LH1.W));
}

void deleteParam(ros::NodeHandle nh_)
{
    auto node_name = ros::this_node::getName();
    std::string deleteparam = "rosparam delete " + node_name;
    system(deleteparam.c_str());
    printf("delete param\n");

}

int main(int argc, char** argv) {
#ifdef __linux__
    signal(SIGINT, intHandler);
    signal(SIGTERM, intHandler);
    signal(SIGKILL, intHandler);
#endif

    ros::init(argc, argv, "api_simple_test");
    ros::NodeHandle nh;
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("vive_pose",10);
    tf::TransformBroadcaster br;
    tf::TransformListener listener;
    ros::Rate rate(freq);

    initialize(nh);
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
                survive_simple_object_get_latest_pose(it, &pose);

                if (survive_simple_object_get_type(it) == SurviveSimpleObject_OBJECT) {
                    // printf("tracker tf\n");
                    transform_surviveWorldToTracker.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                    transform_surviveWorldToTracker.setRotation(tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                }
                else if (survive_simple_object_get_type(it) == SurviveSimpleObject_LIGHTHOUSE) {
                    if (strcmp(survive_simple_serial_number(it), "LHB-400B1A3E")) {
                        // printf("LH0 tf\n");
                        transform_surviveWorldToLH0.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                        transform_surviveWorldToLH0.setRotation(tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                    }
                    if (strcmp(survive_simple_serial_number(it), "LHB-D4EEE18")) {
                        // printf("LH1 tf\n");
                        transform_surviveWorldToLH1.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                        transform_surviveWorldToLH1.setRotation(tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                    }
                }

            }
            // printf("send system tf\n");
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToTracker, ros::Time::now(), "survive_world", "tracker"));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH0, ros::Time::now(), "survive_world", "LH0"));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH1, ros::Time::now(), "survive_world", "LH1"));
            br.sendTransform(tf::StampedTransform(transform_LH0ToMap0, ros::Time::now(), "LH0", "map0"));
            br.sendTransform(tf::StampedTransform(transform_LH1ToMap1, ros::Time::now(), "LH1", "map1"));
            // printf("lookup and print\n");
            try {
                listener.lookupTransform("map0", "tracker", ros::Time(0), transform_map0ToTracker);
                listener.lookupTransform("map1", "tracker", ros::Time(0), transform_map1ToTracker);
            }
            catch (tf::TransformException& ex) {
                ROS_ERROR("%s", ex.what());
            }
            printf("transform: map0 to tracker\n");
            printf("%f, %f, %f\n", transform_map0ToTracker.getOrigin().x()*unit,
                transform_map0ToTracker.getOrigin().y()*unit, transform_map0ToTracker.getOrigin().z()*unit);
            printf("transform: map1 to tracker\n");
            printf("%f, %f, %f\n", transform_map1ToTracker.getOrigin().x()*unit,
                transform_map1ToTracker.getOrigin().y()*unit, transform_map1ToTracker.getOrigin().z()*unit);
            break;
            }
        }
        vive_pose.pose.orientation.w = transform_map0ToTracker.getRotation().getW();
        vive_pose.pose.orientation.x = transform_map0ToTracker.getRotation().getX();
        vive_pose.pose.orientation.y = transform_map0ToTracker.getRotation().getY();
        vive_pose.pose.orientation.z = transform_map0ToTracker.getRotation().getZ();
        vive_pose.pose.position.x = transform_map0ToTracker.getOrigin().getX();
        vive_pose.pose.position.y = transform_map0ToTracker.getOrigin().getY();
        vive_pose.pose.position.z = transform_map0ToTracker.getOrigin().getZ();
        vive_pose.header.stamp = ros::Time::now();
        vive_pose.header.frame_id = "map";
        pose_pub.publish(vive_pose);
        rate.sleep();
    }
    
    deleteParam(nh);
    survive_simple_close(actx);
    printf("Cleaning up\n");
    return 0;
}



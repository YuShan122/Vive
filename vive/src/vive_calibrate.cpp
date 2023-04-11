#include <stdio.h>
#include <string.h>
#include <survive_api.h>
#include <os_generic.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

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

VIVEPOSE tracker_abs;
std::string LH0_serial_number = "LHB-400B1A3E";
std::string LH1_serial_number = "LHB-D4EEE18";
std::string LH2_serial_number = "LHB-2BEE096A";
std::string tracker_serial_number;
std::string tracker_name;

tf::StampedTransform transform_LH0ToMap;
tf::StampedTransform transform_LH1ToMap;
tf::StampedTransform transform_LH2ToMap;
tf::StampedTransform transform_MapToLH0;
tf::StampedTransform transform_MapToLH1;
tf::StampedTransform transform_MapToLH2;
tf::StampedTransform transform_surviveWorldToLH0;
tf::StampedTransform transform_surviveWorldToLH1;
tf::StampedTransform transform_surviveWorldToLH2;
tf::StampedTransform transform_surviveWorldToTracker;
tf::StampedTransform transform_trackerAbsToMap;

void initialize(ros::NodeHandle);
void reloadParam(ros::NodeHandle);
void deleteParam(ros::NodeHandle);

static void log_fn(SurviveSimpleContext* actx, SurviveLogLevel logLevel, const char* msg) {
    fprintf(stderr, "(%7.3f) SimpleApi: %s\n", survive_simple_run_time(actx), msg);
}

int main(int argc, char** argv) {
#ifdef __linux__
    signal(SIGINT, intHandler);
    signal(SIGTERM, intHandler);
    signal(SIGKILL, intHandler);
#endif

    ros::init(argc, argv, "vive_calibrate");
    ros::NodeHandle nh;
    tf::TransformBroadcaster br;
    tf::TransformListener listener;

    initialize(nh);

    SurviveSimpleContext* actx = survive_simple_init_with_logger(argc, argv, log_fn);
    if (actx == 0) // implies -help or similiar
        return 0;

    double start_time = OGGetAbsoluteTime();
    survive_simple_start_thread(actx);

    // print all devices
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

                if (survive_simple_object_get_type(it) == SurviveSimpleObject_OBJECT && strcmp(survive_simple_serial_number(it), tracker_serial_number.c_str()) == 0) {
                    // printf("tracker tf\n");
                    transform_surviveWorldToTracker.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                    transform_surviveWorldToTracker.setRotation(tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                }
                else if (survive_simple_object_get_type(it) == SurviveSimpleObject_LIGHTHOUSE) {
                    if (strcmp(survive_simple_serial_number(it), LH0_serial_number.c_str()) == 0) {
                        // printf("set LH0 tf\n");
                        transform_surviveWorldToLH0.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                        transform_surviveWorldToLH0.setRotation(tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                    }
                    else if (strcmp(survive_simple_serial_number(it), LH1_serial_number.c_str()) == 0) {
                        // printf("set LH1 tf\n");
                        transform_surviveWorldToLH1.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                        transform_surviveWorldToLH1.setRotation(tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                    }
                    else if (strcmp(survive_simple_serial_number(it), LH2_serial_number.c_str()) == 0) {
                        // printf("set LH2 tf\n");
                        transform_surviveWorldToLH2.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                        transform_surviveWorldToLH2.setRotation(tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                    }
                }
            }
            // printf("send\n");
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToTracker, ros::Time::now(), "survive_world", "tracker"));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH0, ros::Time::now(), "survive_world", "LH0"));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH1, ros::Time::now(), "survive_world", "LH1"));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH2, ros::Time::now(), "survive_world", "LH2"));
            br.sendTransform(tf::StampedTransform(transform_trackerAbsToMap, ros::Time::now(), "tracker", "map"));

            // printf("lookup and print\n");
            try {
                listener.lookupTransform("LH0", "map", ros::Time(0), transform_LH0ToMap);
                listener.lookupTransform("LH1", "map", ros::Time(0), transform_LH1ToMap);
                listener.lookupTransform("LH2", "map", ros::Time(0), transform_LH2ToMap);
                listener.lookupTransform("map", "LH0", ros::Time(0), transform_MapToLH0);
                listener.lookupTransform("map", "LH1", ros::Time(0), transform_MapToLH1);
                listener.lookupTransform("map", "LH2", ros::Time(0), transform_MapToLH2);
            }
            catch (tf::TransformException& ex) {
                ROS_ERROR("%s", ex.what());
            }
            // printf("transform: LH0 to map\n");
            // printf("%f, %f, %f\n", transform_LH0ToMap.getOrigin().x(),
            //     transform_LH0ToMap.getOrigin().y(), transform_LH0ToMap.getOrigin().z());
            // printf("transform: LH1 to map\n");
            // printf("%f, %f, %f\n", transform_LH1ToMap.getOrigin().x(),
            //     transform_LH1ToMap.getOrigin().y(), transform_LH1ToMap.getOrigin().z());
            printf("transform: map to LH0\n");
            printf("%f, %f, %f\n", transform_MapToLH0.getOrigin().x(),
                transform_MapToLH0.getOrigin().y(), transform_MapToLH0.getOrigin().z());
            printf("transform: map to LH1\n");
            printf("%f, %f, %f\n", transform_MapToLH1.getOrigin().x(),
                transform_MapToLH1.getOrigin().y(), transform_MapToLH1.getOrigin().z());
            printf("transform: map to LH2\n");
            printf("%f, %f, %f\n", transform_MapToLH2.getOrigin().x(),
                transform_MapToLH2.getOrigin().y(), transform_MapToLH2.getOrigin().z());
            break;
        }
        }
    }

    reloadParam(nh);

    survive_simple_close(actx);
    printf("close the thread\n");
    return 0;
}

void initialize(ros::NodeHandle nh_) {

    auto node_name = ros::this_node::getName();
    nh_.getParam(node_name + "/tracker_abs_x", tracker_abs.x);
    nh_.getParam(node_name + "/tracker_abs_y", tracker_abs.y);
    nh_.getParam(node_name + "/tracker_abs_z", tracker_abs.z);
    nh_.getParam(node_name + "/tracker_abs_W", tracker_abs.W);
    nh_.getParam(node_name + "/tracker_abs_X", tracker_abs.X);
    nh_.getParam(node_name + "/tracker_abs_Y", tracker_abs.Y);
    nh_.getParam(node_name + "/tracker_abs_Z", tracker_abs.Z);
    nh_.getParam(node_name + "/tracker_name", tracker_name);

    if (strcmp("tracker_A", tracker_name.c_str()) == 0) tracker_serial_number = "LHR-94135635";
    if (strcmp("tracker_B", tracker_name.c_str()) == 0) tracker_serial_number = "LHR-15565625";
    if (strcmp("tracker_C", tracker_name.c_str()) == 0) tracker_serial_number = "LHR-662B1E75";
    if (strcmp("tracker_D", tracker_name.c_str()) == 0) tracker_serial_number = "LHR-38203A4C";
    if (strcmp("tracker_E", tracker_name.c_str()) == 0) tracker_serial_number = "LHR-E833C29B";

    transform_trackerAbsToMap.setOrigin(tf::Vector3(tracker_abs.x, tracker_abs.y, tracker_abs.z));
    transform_trackerAbsToMap.setRotation(tf::Quaternion(tracker_abs.X, tracker_abs.Y, tracker_abs.Z, tracker_abs.W));
}

void reloadParam(ros::NodeHandle nh_)
{
    auto node_name = ros::this_node::getName();
    auto ss_prefix = "rosparam set " + node_name;
    auto ss = ss_prefix + "/LH0_x " + std::to_string(transform_LH0ToMap.getOrigin().x());
    system(ss.c_str());
    ss = ss_prefix + "/LH0_y " + std::to_string(transform_LH0ToMap.getOrigin().y());
    system(ss.c_str());
    ss = ss_prefix + "/LH0_z " + std::to_string(transform_LH0ToMap.getOrigin().z());
    system(ss.c_str());
    ss = ss_prefix + "/LH0_W " + std::to_string(transform_LH0ToMap.getRotation().getW());
    system(ss.c_str());
    ss = ss_prefix + "/LH0_X " + std::to_string(transform_LH0ToMap.getRotation().getX());
    system(ss.c_str());
    ss = ss_prefix + "/LH0_Y " + std::to_string(transform_LH0ToMap.getRotation().getY());
    system(ss.c_str());
    ss = ss_prefix + "/LH0_Z " + std::to_string(transform_LH0ToMap.getRotation().getZ());
    system(ss.c_str());

    ss = ss_prefix + "/LH1_x " + std::to_string(transform_LH1ToMap.getOrigin().x());
    system(ss.c_str());
    ss = ss_prefix + "/LH1_y " + std::to_string(transform_LH1ToMap.getOrigin().y());
    system(ss.c_str());
    ss = ss_prefix + "/LH1_z " + std::to_string(transform_LH1ToMap.getOrigin().z());
    system(ss.c_str());
    ss = ss_prefix + "/LH1_W " + std::to_string(transform_LH1ToMap.getRotation().getW());
    system(ss.c_str());
    ss = ss_prefix + "/LH1_X " + std::to_string(transform_LH1ToMap.getRotation().getX());
    system(ss.c_str());
    ss = ss_prefix + "/LH1_Y " + std::to_string(transform_LH1ToMap.getRotation().getY());
    system(ss.c_str());
    ss = ss_prefix + "/LH1_Z " + std::to_string(transform_LH1ToMap.getRotation().getZ());
    system(ss.c_str());

    ss = ss_prefix + "/LH2_x " + std::to_string(transform_LH2ToMap.getOrigin().x());
    system(ss.c_str());
    ss = ss_prefix + "/LH2_y " + std::to_string(transform_LH2ToMap.getOrigin().y());
    system(ss.c_str());
    ss = ss_prefix + "/LH2_z " + std::to_string(transform_LH2ToMap.getOrigin().z());
    system(ss.c_str());
    ss = ss_prefix + "/LH2_W " + std::to_string(transform_LH2ToMap.getRotation().getW());
    system(ss.c_str());
    ss = ss_prefix + "/LH2_X " + std::to_string(transform_LH2ToMap.getRotation().getX());
    system(ss.c_str());
    ss = ss_prefix + "/LH2_Y " + std::to_string(transform_LH2ToMap.getRotation().getY());
    system(ss.c_str());
    ss = ss_prefix + "/LH2_Z " + std::to_string(transform_LH2ToMap.getRotation().getZ());
    system(ss.c_str());
    auto path = "rosparam dump ~/eurobot_localization_ws/src/Eurobot-Localization/vive/param/vive_calibrate.yaml vive_calibrate";
    printf("dump param to vive_calibrate.yaml\n");
    system(path);
    deleteParam(nh_);
}

void deleteParam(ros::NodeHandle nh_)
{
    auto node_name = ros::this_node::getName();
    nh_.deleteParam(node_name + "/tracker_abs_x");
    nh_.deleteParam(node_name + "/tracker_abs_y");
    nh_.deleteParam(node_name + "/tracker_abs_z");
    nh_.deleteParam(node_name + "/tracker_abs_W");
    nh_.deleteParam(node_name + "/tracker_abs_X");
    nh_.deleteParam(node_name + "/tracker_abs_Y");
    nh_.deleteParam(node_name + "/tracker_abs_Z");
    auto path = "rosparam dump ~/eurobot_localization_ws/src/Eurobot-Localization/vive/param/vive_localization.yaml vive_calibrate";
    printf("load param to vive_localization.yaml\n");
    system(path);

    std::string deleteparam = "rosparam delete " + node_name;
    system(deleteparam.c_str());
    printf("cleaning up Param\n");
}
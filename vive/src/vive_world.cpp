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

class Tracker {
public:
    Tracker() {}
    Tracker(char lt, std::string sn) {
        letter = lt;
        serial_num = sn;
        frame = "tracker";
        frame.push_back(letter);
    }
    std::string serial_num;
    std::string frame;
    char letter;
    tf::StampedTransform transform_from_map;
    tf::StampedTransform transform_from_world;
    void set_and_send_transform_from_world(tf::TransformBroadcaster br_, SurvivePose pose_) {
        transform_from_world.setOrigin(tf::Vector3(pose_.Pos[0], pose_.Pos[1], pose_.Pos[2]));
        transform_from_world.setRotation(tf::Quaternion(pose_.Rot[1], pose_.Rot[2], pose_.Rot[3], pose_.Rot[0]));
        br_.sendTransform(tf::StampedTransform(
            transform_from_world, ros::Time::now(), "survive_world", frame));
    }
private:
};

class Lighthouse {
public:
    Lighthouse() {}
    Lighthouse(int od, std::string sn) {
        order = od;
        serial_num = sn;
        frame = "LH" + std::to_string(order);
    }
    std::string serial_num;
    std::string frame;
    int order;
    VIVEPOSE p_to_map;
    tf::StampedTransform transform_to_map;
    tf::StampedTransform transform_from_world;
    void set_transform_form_world(SurvivePose pose_) {
        transform_from_world.setOrigin(
            tf::Vector3(pose_.Pos[0], pose_.Pos[1], pose_.Pos[2]));
        transform_from_world.setRotation(
            tf::Quaternion(pose_.Rot[1], pose_.Rot[2], pose_.Rot[3], pose_.Rot[0]));
    }
    void set_transform_to_map() {
        transform_to_map.setOrigin(tf::Vector3(p_to_map.x, p_to_map.y, p_to_map.z));
        transform_to_map.setRotation(tf::Quaternion(p_to_map.X, p_to_map.Y, p_to_map.Z, p_to_map.W));
    }
    void send_transform_from_world(tf::TransformBroadcaster br_) {
        br_.sendTransform(tf::StampedTransform(transform_from_world,
            ros::Time::now(), "survive_world", frame));
    }
    void send_transform_to_map(tf::TransformBroadcaster br_) {
        br_.sendTransform(tf::StampedTransform(transform_to_map,
            ros::Time::now(), frame, "map" + std::to_string(order)));
    }
private:
};

class Map {
public:
    Map() {
        frame = "map";
    }
    Map(int od) {
        order = od;
        frame = "map" + std::to_string(order);
    }
    int order;
    std::string frame;
    tf::StampedTransform transform_from_world;
private:
};

// VIVEPOSE LH0, LH1, LH2;
Tracker trackerA('A', "LHR-94135635");
Tracker trackerB('B', "LHR-15565625");
Tracker trackerC('C', "LHR-662B1E75");
Tracker trackerD('D', "LHR-38203A4C");
Tracker trackerE('E', "LHR-E833C29B");
Lighthouse lh0(0, "LHB-400B1A3E");
Lighthouse lh1(1, "LHB-D4EEE18");
Lighthouse lh2(2, "LHB-2BEE096A");
Map map;
Map map0(0);
Map map1(1);
Map map2(2);
int freq = 21;
int unit = 1;

static void log_fn(SurviveSimpleContext* actx, SurviveLogLevel logLevel, const char* msg) {
    fprintf(stderr, "(%7.3f) SimpleApi: %s\n", survive_simple_run_time(actx), msg);
}

void initialize(ros::NodeHandle nh_) {
    bool get_param_ok;
    auto node_name = ros::this_node::getName();
    nh_.getParam(node_name + "/LH0_x", lh0.p_to_map.x);
    nh_.getParam(node_name + "/LH0_y", lh0.p_to_map.y);
    nh_.getParam(node_name + "/LH0_z", lh0.p_to_map.z);
    nh_.getParam(node_name + "/LH0_W", lh0.p_to_map.W);
    nh_.getParam(node_name + "/LH0_X", lh0.p_to_map.X);
    nh_.getParam(node_name + "/LH0_Y", lh0.p_to_map.Y);
    nh_.getParam(node_name + "/LH0_Z", lh0.p_to_map.Z);

    nh_.getParam(node_name + "/LH1_x", lh1.p_to_map.x);
    nh_.getParam(node_name + "/LH1_y", lh1.p_to_map.y);
    nh_.getParam(node_name + "/LH1_z", lh1.p_to_map.z);
    nh_.getParam(node_name + "/LH1_W", lh1.p_to_map.W);
    nh_.getParam(node_name + "/LH1_X", lh1.p_to_map.X);
    nh_.getParam(node_name + "/LH1_Y", lh1.p_to_map.Y);
    nh_.getParam(node_name + "/LH1_Z", lh1.p_to_map.Z);

    nh_.getParam(node_name + "/LH2_x", lh2.p_to_map.x);
    nh_.getParam(node_name + "/LH2_y", lh2.p_to_map.y);
    nh_.getParam(node_name + "/LH2_z", lh2.p_to_map.z);
    nh_.getParam(node_name + "/LH2_W", lh2.p_to_map.W);
    nh_.getParam(node_name + "/LH2_X", lh2.p_to_map.X);
    nh_.getParam(node_name + "/LH2_Y", lh2.p_to_map.Y);
    nh_.getParam(node_name + "/LH2_Z", lh2.p_to_map.Z);

    nh_.getParam(node_name + "/freq", freq);
    nh_.getParam(node_name + "/unit", unit);

    lh0.set_transform_to_map();
    lh1.set_transform_to_map();
    lh2.set_transform_to_map();
}

void deleteParam(ros::NodeHandle nh_)
{
    auto node_name = ros::this_node::getName();
    std::string deleteparam = "rosparam delete " + node_name;
    system(deleteparam.c_str());
    printf("delete param\n");

}

/*
Map map_avg(Map map_0, Map map_1, Map map_2) {
    Map map_avg;
    VIVEPOSE pose_avg;
    tf::Quaternion q;
    q = map_0.transform_from_world.getRotation().operator/(3).operator+
        (map_1.transform_from_world.getRotation().operator/(3)).operator+
        (map_2.transform_from_world.getRotation().operator/(3));
    pose_avg.W = q.getW();
    pose_avg.X = q.getX();
    pose_avg.Y = q.getY();
    pose_avg.Z = q.getZ();
    pose_avg.x = (double)(
        map_0.transform_from_world.getOrigin().getX() +
        map_1.transform_from_world.getOrigin().getX() +
        map_2.transform_from_world.getOrigin().getX()) / 3;
    pose_avg.y = (double)(
        map_0.transform_from_world.getOrigin().getY() +
        map_1.transform_from_world.getOrigin().getY() +
        map_2.transform_from_world.getOrigin().getY()) / 3;
    pose_avg.z = (double)(
        map_0.transform_from_world.getOrigin().getZ() +
        map_1.transform_from_world.getOrigin().getZ() +
        map_2.transform_from_world.getOrigin().getZ()) / 3;

    map_avg.transform_from_world.setOrigin(tf::Vector3(
        pose_avg.x, pose_avg.y, pose_avg.z));
    map_avg.transform_from_world.setRotation(q);

    printf("survive_world to map_avg (x y z W X Y Z)\n");
    printf("%f, %f, %f, %f, %f, %f, %f\n",
        pose_avg.x, pose_avg.y, pose_avg.z,
        pose_avg.W, pose_avg.X,
        pose_avg.Y, pose_avg.Z);
    return map_avg;
};
*/

int main(int argc, char** argv) {
#ifdef __linux__
    signal(SIGINT, intHandler);
    signal(SIGTERM, intHandler);
    signal(SIGKILL, intHandler);
#endif

    ros::init(argc, argv, "vive_world");
    ros::NodeHandle nh;
    tf::TransformBroadcaster br;
    tf::TransformListener listener;

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
        for (const SurviveSimpleObject* it = survive_simple_get_first_object(actx);
            it != 0; it = survive_simple_get_next_object(actx, it)) {
            SurvivePose pose;
            survive_simple_object_get_latest_pose(it, &pose);
            printf("%s, %s, %f, %f, %f, %f, %f, %f, %f\n",
                survive_simple_object_name(it), survive_simple_serial_number(it),
                pose.Pos[0], pose.Pos[1], pose.Pos[2],
                pose.Rot[0], pose.Rot[1], pose.Rot[2], pose.Rot[3]);
            if (survive_simple_object_get_type(it) == SurviveSimpleObject_OBJECT) {
                SurviveVelocity velocity;
                survive_simple_object_get_latest_velocity(it, &velocity);
                // printf("%s velocity : \nx : %f\ny : %f\nz : %f\n", survive_simple_object_name(it),
                //     velocity.Pos[0], velocity.Pos[1], velocity.Pos[2]);
                if (strcmp(survive_simple_serial_number(it), trackerA.serial_num.c_str()) == 0)
                {
                    trackerA.set_and_send_transform_from_world(br, pose);
                }
                else if (strcmp(survive_simple_serial_number(it), trackerB.serial_num.c_str()) == 0)
                {
                    trackerB.set_and_send_transform_from_world(br, pose);
                }
                else if (strcmp(survive_simple_serial_number(it), trackerC.serial_num.c_str()) == 0)
                {
                    trackerC.set_and_send_transform_from_world(br, pose);
                }
                else if (strcmp(survive_simple_serial_number(it), trackerD.serial_num.c_str()) == 0)
                {
                    trackerD.set_and_send_transform_from_world(br, pose);
                }
                else if (strcmp(survive_simple_serial_number(it), trackerE.serial_num.c_str()) == 0)
                {
                    trackerE.set_and_send_transform_from_world(br, pose);
                }

            }
            else if (survive_simple_object_get_type(it) == SurviveSimpleObject_LIGHTHOUSE) {
                if (strcmp(survive_simple_serial_number(it), lh0.serial_num.c_str()) == 0) {
                    lh0.set_transform_form_world(pose);
                    lh0.send_transform_from_world(br);
                    lh0.send_transform_to_map(br);
                }
                else if (strcmp(survive_simple_serial_number(it), lh1.serial_num.c_str()) == 0) {
                    lh1.set_transform_form_world(pose);
                    lh1.send_transform_from_world(br);
                    lh1.send_transform_to_map(br);
                }
                else if (strcmp(survive_simple_serial_number(it), lh2.serial_num.c_str()) == 0) {
                    lh2.set_transform_form_world(pose);
                    lh2.send_transform_from_world(br);
                    lh2.send_transform_to_map(br);
                }
            }
        }
        // try {
        //     listener.lookupTransform("survive_world", map0.frame, ros::Time(0), map0.transform_from_world);
        //     listener.lookupTransform("survive_world", map1.frame, ros::Time(0), map1.transform_from_world);
        //     listener.lookupTransform("survive_world", map2.frame, ros::Time(0), map2.transform_from_world);
        // }
        // catch (tf::TransformException& ex) {
        //     ROS_ERROR("%s", ex.what());
        // }

        // map = map_avg(map0, map1, map2);
        // br.sendTransform(tf::StampedTransform(map.transform_from_world, ros::Time::now(), "survive_world", "map"));


    }
    rate.sleep();

    deleteParam(nh);
    survive_simple_close(actx);
    printf("Cleaning up\n");
    return 0;
}



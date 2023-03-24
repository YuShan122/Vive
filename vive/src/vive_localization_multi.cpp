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
    Tracker() {};
    Tracker(char lt, std::string sn) {
        letter = lt;
        serial_num = sn;
        frame = "tracker";
        frame.push_back(letter);
    };
    std::string serial_num;
    std::string frame;
    char letter;
    tf::StampedTransform transform_from_map;
    tf::StampedTransform transform_from_map0;
    tf::StampedTransform transform_from_map1;
    tf::StampedTransform transform_from_map2;
    tf::StampedTransform transform_from_world;
private:
};

class Lighthouse {
public:
    Lighthouse() {};
    Lighthouse(int od, std::string sn) {
        order = od;
        serial_num = sn;
    };
    std::string serial_num;
    int order;
    VIVEPOSE p_to_map;
    tf::StampedTransform transform_to_map;
    tf::StampedTransform transform_from_world;
    void setTransform_form_world(SurvivePose pose_) {
        transform_from_world.setOrigin(
            tf::Vector3(pose_.Pos[0], pose_.Pos[1], pose_.Pos[2]));
        transform_from_world.setRotation(
            tf::Quaternion(pose_.Rot[1], pose_.Rot[2], pose_.Rot[3], pose_.Rot[0]));

    }
private:
};

class Map {
public:
    Map() {};
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
Map map0, map1, map2, map;
std::string tracker_prefix = "tracker";
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

    lh0.transform_to_map.setOrigin(tf::Vector3(
        lh0.p_to_map.x, lh0.p_to_map.y, lh0.p_to_map.z));
    lh0.transform_to_map.setRotation(tf::Quaternion(
        lh0.p_to_map.X, lh0.p_to_map.Y, lh0.p_to_map.Z, lh0.p_to_map.W));
    lh1.transform_to_map.setOrigin(tf::Vector3(
        lh1.p_to_map.x, lh1.p_to_map.y, lh1.p_to_map.z));
    lh1.transform_to_map.setRotation(tf::Quaternion(
        lh1.p_to_map.X, lh1.p_to_map.Y, lh1.p_to_map.Z, lh1.p_to_map.W));
    lh2.transform_to_map.setOrigin(tf::Vector3(
        lh2.p_to_map.x, lh2.p_to_map.y, lh2.p_to_map.z));
    lh2.transform_to_map.setRotation(tf::Quaternion(
        lh2.p_to_map.X, lh2.p_to_map.Y, lh2.p_to_map.Z, lh2.p_to_map.W));
}

void deleteParam(ros::NodeHandle nh_)
{
    auto node_name = ros::this_node::getName();
    std::string deleteparam = "rosparam delete " + node_name;
    system(deleteparam.c_str());
    printf("delete param\n");

}


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

    printf("map_avg (x y z W X Y Z)\n");
    printf("%f, %f, %f, %f, %f, %f, %f\n",
        pose_avg.x, pose_avg.y, pose_avg.z,
        pose_avg.W, pose_avg.X,
        pose_avg.Y, pose_avg.Z);
    return map_avg;
};

// geometry_msgs::PoseStamped avg_pose(tf::TransformBroadcaster br_) {
//     geometry_msgs::PoseStamped pose_avg;
//     tf::Quaternion q;
//     q = trackerA.transform_from_map0.getRotation().operator/(3).operator+
//         (trackerA.transform_from_map1.getRotation().operator/(3)).operator+
//         (trackerA.transform_from_map2.getRotation().operator/(3));
//     pose_avg.pose.orientation.w = q.getW();
//     pose_avg.pose.orientation.x = q.getX();
//     pose_avg.pose.orientation.y = q.getY();
//     pose_avg.pose.orientation.z = q.getZ();
//     pose_avg.pose.position.x = (double)(
//         trackerA.transform_from_map0.getOrigin().getX() +
//         trackerA.transform_from_map1.getOrigin().getX() +
//         trackerA.transform_from_map2.getOrigin().getX()) / 3;
//     pose_avg.pose.position.y = (double)(
//         trackerA.transform_from_map0.getOrigin().getY() +
//         trackerA.transform_from_map1.getOrigin().getY() +
//         trackerA.transform_from_map2.getOrigin().getY()) / 3;
//     pose_avg.pose.position.z = (double)(
//         trackerA.transform_from_map0.getOrigin().getZ() +
//         trackerA.transform_from_map1.getOrigin().getZ() +
//         trackerA.transform_from_map2.getOrigin().getZ()) / 3;
//     pose_avg.header.stamp = ros::Time::now();
//     pose_avg.header.frame_id = "map";

//     transform_mapToTracker.setOrigin(tf::Vector3(
//         pose_avg.pose.position.x, pose_avg.pose.position.y, pose_avg.pose.position.z));
//     transform_mapToTracker.setRotation(q);
//     br_.sendTransform(tf::StampedTransform(transform_mapToTracker, ros::Time::now(), "map", "tracker_avg"));

//     printf("transform: avg_pose (x y z W X Y Z)\n");
//     printf("%f, %f, %f, %f, %f, %f, %f\n",
//         pose_avg.pose.position.x, pose_avg.pose.position.y, pose_avg.pose.position.z,
//         pose_avg.pose.orientation.w, pose_avg.pose.orientation.x,
//         pose_avg.pose.orientation.y, pose_avg.pose.orientation.z);
//     return pose_avg;
// }

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
                    Tracker tracker_;
                    if (strcmp(survive_simple_serial_number(it), trackerA.serial_num.c_str()) == 0)
                    {
                        tracker_ = trackerA;
                    }
                    else if (strcmp(survive_simple_serial_number(it), trackerB.serial_num.c_str()) == 0)
                    {
                        tracker_ = trackerB;
                    }
                    tracker_.transform_from_world.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                    tracker_.transform_from_world.setRotation(tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                    br.sendTransform(tf::StampedTransform(
                        tracker_.transform_from_world, ros::Time::now(), "survive_world", tracker_.frame));
                }
                else if (survive_simple_object_get_type(it) == SurviveSimpleObject_LIGHTHOUSE) {
                    if (strcmp(survive_simple_serial_number(it), lh0.serial_num.c_str()) == 0) {
                        lh0.setTransform_form_world(pose);
                    }
                    else if (strcmp(survive_simple_serial_number(it), lh1.serial_num.c_str()) == 0) {
                        lh1.setTransform_form_world(pose);
                    }
                    else if (strcmp(survive_simple_serial_number(it), lh2.serial_num.c_str()) == 0) {
                        lh2.setTransform_form_world(pose);
                    }
                }
            }
            br.sendTransform(tf::StampedTransform(lh0.transform_from_world, ros::Time::now(), "survive_world", "LH0"));
            br.sendTransform(tf::StampedTransform(lh1.transform_from_world, ros::Time::now(), "survive_world", "LH1"));
            br.sendTransform(tf::StampedTransform(lh2.transform_from_world, ros::Time::now(), "survive_world", "LH2"));
            br.sendTransform(tf::StampedTransform(lh0.transform_to_map, ros::Time::now(), "LH0", "map0"));
            br.sendTransform(tf::StampedTransform(lh1.transform_to_map, ros::Time::now(), "LH1", "map1"));
            br.sendTransform(tf::StampedTransform(lh2.transform_to_map, ros::Time::now(), "LH2", "map2"));
            try {
                listener.lookupTransform("survive_world", "map0", ros::Time(0), map0.transform_from_world);
                listener.lookupTransform("survive_world", "map1", ros::Time(0), map1.transform_from_world);
                listener.lookupTransform("survive_world", "map2", ros::Time(0), map2.transform_from_world);
            }
            catch (tf::TransformException& ex) {
                ROS_ERROR("%s", ex.what());
            }

            map = map_avg(map0, map1, map2);
            br.sendTransform(tf::StampedTransform(map.transform_from_world, ros::Time::now(), "survive_world", "map"));

            try {
                listener.lookupTransform("map", "trackerA", ros::Time(0), trackerA.transform_from_map);
                listener.lookupTransform("map0", "trackerA", ros::Time(0), trackerA.transform_from_map0);
                listener.lookupTransform("map1", "trackerA", ros::Time(0), trackerA.transform_from_map1);
                listener.lookupTransform("map2", "trackerA", ros::Time(0), trackerA.transform_from_map2);
                listener.lookupTransform("map", "trackerB", ros::Time(0), trackerB.transform_from_map);
                listener.lookupTransform("map0", "trackerB", ros::Time(0), trackerB.transform_from_map0);
                listener.lookupTransform("map1", "trackerB", ros::Time(0), trackerB.transform_from_map1);
                listener.lookupTransform("map2", "trackerB", ros::Time(0), trackerB.transform_from_map2);
            }
            catch (tf::TransformException& ex) {
                ROS_ERROR("%s", ex.what());
            }
            printf("transform: map0 to tracker_A\n");
            printf("%f, %f, %f\n", trackerA.transform_from_map0.getOrigin().x() * unit,
                trackerA.transform_from_map0.getOrigin().y() * unit, trackerA.transform_from_map0.getOrigin().z() * unit);
            printf("transform: map1 to tracker_A\n");
            printf("%f, %f, %f\n", trackerA.transform_from_map1.getOrigin().x() * unit,
                trackerA.transform_from_map1.getOrigin().y() * unit, trackerA.transform_from_map1.getOrigin().z() * unit);
            printf("transform: map2 to tracker_A\n");
            printf("%f, %f, %f\n", trackerA.transform_from_map2.getOrigin().x() * unit,
                trackerA.transform_from_map2.getOrigin().y() * unit, trackerA.transform_from_map2.getOrigin().z() * unit);

            printf("transform: map0 to trackerB\n");
            printf("%f, %f, %f\n", trackerB.transform_from_map0.getOrigin().x() * unit,
                trackerB.transform_from_map0.getOrigin().y() * unit, trackerB.transform_from_map0.getOrigin().z() * unit);
            printf("transform: map1 to trackerB\n");
            printf("%f, %f, %f\n", trackerB.transform_from_map1.getOrigin().x() * unit,
                trackerB.transform_from_map1.getOrigin().y() * unit, trackerB.transform_from_map1.getOrigin().z() * unit);
            printf("transform: map2 to trackerB\n");
            printf("%f, %f, %f\n", trackerB.transform_from_map2.getOrigin().x() * unit,
                trackerB.transform_from_map2.getOrigin().y() * unit, trackerB.transform_from_map2.getOrigin().z() * unit);

            break;
        }
        }
        vive_pose.pose.orientation.w = trackerA.transform_from_map0.getRotation().getW();
        vive_pose.pose.orientation.x = trackerA.transform_from_map0.getRotation().getX();
        vive_pose.pose.orientation.y = trackerA.transform_from_map0.getRotation().getY();
        vive_pose.pose.orientation.z = trackerA.transform_from_map0.getRotation().getZ();
        vive_pose.pose.position.x = trackerA.transform_from_map0.getOrigin().getX();
        vive_pose.pose.position.y = trackerA.transform_from_map0.getOrigin().getY();
        vive_pose.pose.position.z = trackerA.transform_from_map0.getOrigin().getZ();
        vive_pose.header.stamp = ros::Time::now();
        vive_pose.header.frame_id = "map";
        // pose_pub.publish(vive_pose);
        // pose_pub.publish(avg_pose(br));
        rate.sleep();
    }

    deleteParam(nh);
    survive_simple_close(actx);
    printf("Cleaning up\n");
    return 0;
}



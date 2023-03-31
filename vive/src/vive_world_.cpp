#include <stdio.h>
#include <string.h>
#include <iostream>
#include <cmath>

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
    // double yaw, roll, pitch;
}VIVEPOSE;

class ViveDevice {
public:
    ViveDevice();
    ViveDevice(std::string pf_, const char* sn_);
    void send_tf_from_world(SurvivePose p_, std::string wf_);
private:
    std::string frame;
    const char* serial_num;
    tf::TransformBroadcaster br;
    // tf::TransformListener listener;
};
ViveDevice::ViveDevice(std::string pf_, const char* sn_) {
    serial_num = sn_;
    if (strcmp(serial_num, "LHR-94135635") == 0) frame = "trackerA";
    if (strcmp(serial_num, "LHR-15565625") == 0) frame = "trackerB";
    if (strcmp(serial_num, "LHR-662B1E75") == 0) frame = "trackerC";
    if (strcmp(serial_num, "LHR-38203A4C") == 0) frame = "trackerD";
    if (strcmp(serial_num, "LHR-E833C29B") == 0) frame = "trackerE";
    if (strcmp(serial_num, "LHB-400B1A3E") == 0) frame = pf_ + "LH0";
    if (strcmp(serial_num, "LHB-D4EEE18") == 0) frame = pf_ + "LH1";
    if (strcmp(serial_num, "LHB-2BEE096A") == 0) frame = pf_ + "LH2";
}
void ViveDevice::send_tf_from_world(SurvivePose p_, std::string wf_) {
    tf::StampedTransform tf_from_world;
    tf_from_world.setOrigin(tf::Vector3(p_.Pos[0], p_.Pos[1], p_.Pos[2]));
    tf_from_world.setRotation(tf::Quaternion(p_.Rot[1], p_.Rot[2], p_.Rot[3], p_.Rot[0]));
    br.sendTransform(tf::StampedTransform(tf_from_world, ros::Time::now(), wf_, frame));
}

class ViveMap {
public:
    ViveMap();
    ViveMap(std::string f_);
    ViveMap(std::string pf_, int od_);
    void set_tf_from_lh(ros::NodeHandle nh_);
    void send_tf_from_lh();
    void lookup_tf_to_world(std::string wf_);
    VIVEPOSE get_p_to_world(bool ok);
    tf::Quaternion get_q_from_world_devide_by(int dvs, bool ok);
private:
    std::string frame;
    std::string lh_frame;
    int order;
    tf::StampedTransform tf_from_lh;
    tf::StampedTransform tf_to_world;
    VIVEPOSE p_from_lh;
    VIVEPOSE p_to_world;
    tf::TransformBroadcaster br;
    tf::TransformListener listener;
};
ViveMap::ViveMap(std::string f_) {
    frame = f_;
}
ViveMap::ViveMap(std::string pf_, int od_) {
    order = od_;
    frame = pf_ + "map" + std::to_string(order);
    lh_frame = pf_ + "LH" + std::to_string(order);
}
void ViveMap::set_tf_from_lh(ros::NodeHandle nh_) {
    bool ok = true;
    ok &= nh_.getParam("LH" + std::to_string(order) + "_x", p_from_lh.x);
    ok &= nh_.getParam("LH" + std::to_string(order) + "_y", p_from_lh.y);
    ok &= nh_.getParam("LH" + std::to_string(order) + "_z", p_from_lh.z);
    ok &= nh_.getParam("LH" + std::to_string(order) + "_W", p_from_lh.W);
    ok &= nh_.getParam("LH" + std::to_string(order) + "_X", p_from_lh.X);
    ok &= nh_.getParam("LH" + std::to_string(order) + "_Y", p_from_lh.Y);
    ok &= nh_.getParam("LH" + std::to_string(order) + "_Z", p_from_lh.Z);
    if (ok) {
        std::cout << "set tf: lh" << order << " to map" << order << " successed." << std::endl;
    }
    else {
        std::cout << "set tf: lh" << order << " to map" << order << " failed." << std::endl;
    }
    tf_from_lh.setOrigin(tf::Vector3(p_from_lh.x, p_from_lh.y, p_from_lh.z));
    tf_from_lh.setRotation(tf::Quaternion(p_from_lh.X, p_from_lh.Y, p_from_lh.Z, p_from_lh.W));
}
void ViveMap::send_tf_from_lh() {
    br.sendTransform(tf::(StampedTransform(tf_from_lh, ros::Time::now(), lh_frame, frame)));
}
void ViveMap::lookup_tf_to_world(std::string wf_) {
    try {
        listener.lookupTransform(frame, wf_, ros::Time(0), tf_to_world);
    }
    catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
    }
    p_to_world.x = tf_to_world.getOrigin().getX();
    p_to_world.y = tf_to_world.getOrigin().getY();
    p_to_world.z = tf_to_world.getOrigin().getZ();
    p_to_world.W = tf_to_world.getRotation().getW();
    p_to_world.X = tf_to_world.getRotation().getX();
    p_to_world.Y = tf_to_world.getRotation().getY();
    p_to_world.Z = tf_to_world.getRotation().getZ();
}
VIVEPOSE ViveMap::get_p_to_world(bool ok) {
    if (!ok) {
        VIVEPOSE p_zero;
        p_zero.x = 0;
        p_zero.y = 0;
        p_zero.z = 0;
        p_zero.W = 1;
        p_zero.X = 0;
        p_zero.Y = 0;
        p_zero.Z = 0;
        return p_zero;
    }
    return p_to_world;
}
tf::Quaternion ViveMap::get_q_to_world_devide_by(int dvs, bool ok) {
    tf::Quaternion q;
    if (!ok) {
        tf::Quaternion q_zero(0, 0, 0, 1);
        return q_zero;
    }
    q = tf_to_world.getRotation().operator/(dvs);
    return q;
}

double find_distance(ViveMap map1, ViveMap map2) {
    double distance;
    double x = map1.get_p_to_world().x - map2.get_p_to_world().x;
    double y = map1.get_p_to_world().y - map2.get_p_to_world().y;
    double z = map1.get_p_to_world().z - map2.get_p_to_world().z;

    distance = sqrt(x * x + y * y + z * z);
    return distance;
}
ViveMap find_avg_map(ViveMap map0, ViveMap map1, ViveMap map2) {
    ViveMap avg_map;
    tf::Quaternion avg_q;
    tf::StampedTransform avg_tf;
    VIVEPOSE avg_p;
    bool okall = true;
    bool ok0 = true;
    bool ok1 = true;
    bool ok2 = true;
    int divisor = 3;

    double d01 = find_distance(map0, map1);
    double d12 = find_distance(map1, map2);
    double d20 = find_distance(map2, map0);
    if (d01 + d12 + d20 > 0.1) okall = false;
    if (d01 + d20 > 0.1) ok0 = false;
    if (d12 + d01 > 0.1) ok1 = false;
    if (d20 + d12 > 0.1) ok2 = false;

    if (okall) divisor = 3;
    if (!ok0) divisor--;
    if (!ok1) divisor--;
    if (!ok2) divisor--;

    avg_p.x = (double)
        map0.get_p_to_world(ok0).x / divisor +
        map1.get_p_to_world(ok1).x / divisor +
        map2.get_p_to_world(ok2).x / divisor;
    avg_p.y = (double)
        map0.get_p_to_world(ok0).y / divisor +
        map1.get_p_to_world(ok1).y / divisor +
        map2.get_p_to_world(ok2).y / divisor;
    avg_p.z = (double)
        map0.get_p_to_world(ok0).z / divisor +
        map1.get_p_to_world(ok1).z / divisor +
        map2.get_p_to_world(ok2).z / divisor;
    avg_q =
        map0.get_q_from_world_devide_by(divisor, ok0).operator+(
            map1.get_q_from_world_devide_by(divisor, ok1)).operator+(
                map2.get_q_from_world_devide_by(divisor, ok2));


    return avg_map;
}

/*globle variables*/
std::string survive_prefix;
std::string wold_frame;
std::string node_name;
std::string name_space;
int freq;
int unit;
int tracker_num;

/*functions*/
void initialize(ros::NodeHandle nh_) {
    bool ok = true;
    node_name = ros::this_node::getName();
    name_space = ros::this_node::getNamespace();

    ok &= nh_.getParam("freq", freq);
    ok &= nh_.getParam("unit", unit);
    ok &= nh_.getParam("survive_prefix", survive_prefix);
    ok &= nh_.getParam("tracker_num", tracker_num);

    std::cout << "param: freq= " << freq << std::endl;
    std::cout << "param: unit= " << unit << std::endl;
    std::cout << "param: survive_prefix= " << survive_prefix << std::endl;
    std::cout << "param: tracker_num= " << tracker_num << std::endl;
    if (ok) {
        std::cout << "node: " << node_name << " get parameters of node sucessed." << std::endl;
    }
    else {
        std::cout << "node: " << node_name << " get parameters of node failed." << std::endl;
    }
    std::cout << "node: " << node_name << " initialized." << "(in namespace: " << name_space << ")" << std::endl;

    world_frame = survive_prefix + "world";
}

static void log_fn(SurviveSimpleContext* actx, SurviveLogLevel logLevel, const char* msg) {
    fprintf(stderr, "(%7.3f) SimpleApi: %s\n", survive_simple_run_time(actx), msg);
}

int main(int argc, char** argv) {
#ifdef __linux__
    signal(SIGINT, intHandler);
    signal(SIGTERM, intHandler);
    signal(SIGKILL, intHandler);
#endif

    ros::init(argc, argv, "vive_world");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");
    tf::TransformBroadcaster br;
    tf::TransformListener listener;

    SurviveSimpleContext* actx = survive_simple_init_with_logger(argc, argv, log_fn);
    if (actx == 0) // implies -help or similiar
        return 0;
    double start_time = OGGetAbsoluteTime();
    survive_simple_start_thread(actx);
    for (const SurviveSimpleObject* it = survive_simple_get_first_object(actx); it != 0;
        it = survive_simple_get_next_object(actx, it)) {
        printf("Found '%s'\n", survive_simple_object_name(it));
    }

    initialize(nh_);
    ros::Rate rate(freq);
    ViveMap map(survive_prefix + "map");
    ViveMap map0(survive_prefix, 0);
    ViveMap map1(survive_prefix, 1);
    ViveMap map2(survive_prefix, 2);
    map0.set_tf_from_lh(nh_);
    map1.set_tf_from_lh(nh_);
    map2.set_tf_from_lh(nh_);

    struct SurviveSimpleEvent event = {};
    while (keepRunning && survive_simple_wait_for_event(actx, &event) != SurviveSimpleEventType_Shutdown && ros::ok()) {
        for (const SurviveSimpleObject* it = survive_simple_get_first_object(actx);
            it != 0; it = survive_simple_get_next_object(actx, it)) {
            SurvivePose pose;
            SurviveVelocity velocity;
            survive_simple_object_get_latest_pose(it, &pose);
            survive_simple_object_get_latest_velocity(it, &velocity);
            printf("%s, %s, %f, %f, %f, %f, %f, %f, %f\n",
                survive_simple_object_name(it), survive_simple_serial_number(it),
                pose.Pos[0], pose.Pos[1], pose.Pos[2],
                pose.Rot[0], pose.Rot[1], pose.Rot[2], pose.Rot[3]);
            // printf("%s velocity : \nx : %f\ny : %f\nz : %f\n", survive_simple_object_name(it),
            //     velocity.Pos[0], velocity.Pos[1], velocity.Pos[2]);

            ViveDevice device(survive_prefix, survive_simple_serial_number(it));
            device.send_tf_from_world(pose, world_frame);
            // if (survive_simple_object_get_type(it) == SurviveSimpleObject_OBJECT) {
            // }
            // else if (survive_simple_object_get_type(it) == SurviveSimpleObject_LIGHTHOUSE) {
            // }
        }

        map0.send_tf_from_lh();
        map1.send_tf_from_lh();
        map2.send_tf_from_lh();
        map0.lookup_tf_to_world(world_frame);
        map1.lookup_tf_to_world(world_frame);
        map2.lookup_tf_to_world(world_frame);

        rate.sleep();
    }

    survive_simple_close(actx);
    printf("Cleaning up\n");
    return 0;
}



#include <stdio.h>
#include <string.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>


typedef struct vivePose {
    double x, y, z;
    double W, X, Y, Z;
    double yaw, roll, pitch;
}VIVEPOSE;

class theNode {
public:
    theNode(ros::NodeHandle nh);
    void initialize();
    void deleteParam();
    int freq;
    std::string robot_name;
private:
    ros::NodeHandle nh_;
    std::string node_name;
    int unit;
    int tracker_num;
};
theNode::theNode(ros::NodeHandle nh) {
    nh_ = nh;
    freq = 21;
    unit = 1;
}
void theNode::initialize() {
    node_name = ros::this_node::getName();
    nh_.getParam(node_name + "/freq", freq);
    nh_.getParam(node_name + "/unit", unit);
    nh_.getParam(node_name + "/tracker_num", tracker_num);
    nh_.getParam(node_name + "/robot_name", robot_name);
}
void theNode::deleteParam()
{
    std::string deleteparam = "rosparam delete " + node_name;
    system(deleteparam.c_str());
}

class Tracker {
public:
    Tracker();
    Tracker(char lt, std::string sn);
    // void initTracker(char lt, std::string sn) {
    //     letter = lt;
    //     serial_num = sn;
    //     frame = "tracker";
    //     frame.push_back(letter);
    // }
    void lookup_transform_from_map();
    void print_tracker_pose();
    void publish_tracker_pose();
private:
    std::string serial_num;
    std::string frame;
    char letter;
    bool used;
    tf::StampedTransform transform_from_map;
    tf::StampedTransform transform_from_world;
    tf::TransformListener listener_;
    ros::Publisher pose_pub;
};
Tracker::Tracker() {
    used = false;
}
Tracker::Tracker(char lt, std::string sn) {
    letter = lt;
    serial_num = sn;
    frame = "tracker";
    frame.push_back(letter);
    used = false;
}
void Tracker::lookup_transform_from_map() {
    if (listener_.canTransform("map", frame, ros::Time(0))) {
        used = true;
        try {
            listener_.lookupTransform("map", frame, ros::Time(0), transform_from_map);
        }
        catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
        }
    }
}
void Tracker::print_tracker_pose() {
    if (used) {
        printf("tracker%c pose: (x y z W X Y Z)\n", letter);
        printf("%f, %f, %f, %f, %f, %f, %f\n", transform_from_map.getOrigin().x(),
            transform_from_map.getOrigin().y(), transform_from_map.getOrigin().z(),
            transform_from_map.getRotation().getW(), transform_from_map.getRotation().getX(),
            transform_from_map.getRotation().getY(), transform_from_map.getRotation().getZ());
    }
    else {
        printf("tracker%c not used\n", letter);
    }
}
void Tracker::publish_tracker_pose(ros::Publisher pose_pub_) {

}

// class Lighthouse {
// public:
//     Lighthouse();
//     Lighthouse(int od, std::string sn) {
//         order = od;
//         serial_num = sn;
//         frame = "LH" + std::to_string(order);
//     }
//     std::string serial_num;
//     std::string frame;
//     int order;
//     VIVEPOSE p_to_map;
//     tf::StampedTransform transform_to_map;
//     tf::StampedTransform transform_from_world;
//     void set_transform_to_map() {
//         transform_to_map.setOrigin(tf::Vector3(p_to_map.x, p_to_map.y, p_to_map.z));
//         transform_to_map.setRotation(tf::Quaternion(p_to_map.X, p_to_map.Y, p_to_map.Z, p_to_map.W));
//     }
//     void send_transform_from_world(tf::TransformBroadcaster br_) {
//         br_.sendTransform(tf::StampedTransform(transform_from_world,
//             ros::Time::now(), "survive_world", frame));
//     }
//     void send_transform_to_map(tf::TransformBroadcaster br_) {
//         br_.sendTransform(tf::StampedTransform(transform_to_map,
//             ros::Time::now(), frame, "map" + std::to_string(order)));
//     }
// private:
// };


class Map {
public:
    Map();
    Map(int od);
    void lookup_transform_from_world();
    tf::StampedTransform transform_from_world;
private:
    int order;
    std::string frame;
};
Map::Map() {
    frame = "map";
}
Map::Map(int od) {
    order = od;
    frame = "map" + std::to_string(order);
}
void Map::lookup_transform_from_world() {
    tf::TransformListener listener_;
    // tf::StampedTransform transform_from_world;
    if (listener_.canTransform("survive_world", frame, ros::Time(0))) {
        try {
            listener_.lookupTransform("survive_world", frame, ros::Time(0), transform_from_world);
        }
        catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
        }
    }
}

Map find_map_avg(Map map_0, Map map_1, Map map_2) {
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

int main(int argc, char** argv) {

    ros::init(argc, argv, "vive_trackerpose");
    ros::NodeHandle nh;
    theNode thenode(nh);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("vive_pose", 10);
    tf::TransformBroadcaster br;
    tf::TransformListener listener;
    Tracker trackerA('A', "LHR-94135635");
    Tracker trackerB('B', "LHR-15565625");
    Tracker trackerC('C', "LHR-662B1E75");
    Tracker trackerD('D', "LHR-38203A4C");
    Tracker trackerE('E', "LHR-E833C29B");
    Map map;
    Map map0(0);
    Map map1(1);
    Map map2(2);

    thenode.initialize();
    ros::Rate rate(thenode.freq);
    geometry_msgs::PoseStamped vive_pose;

    while (ros::ok()) {
        map0.lookup_transform_from_world();
        map1.lookup_transform_from_world();
        map2.lookup_transform_from_world();
        map = find_map_avg(map0, map1, map2);
        br.sendTransform(tf::StampedTransform(map.transform_from_world, ros::Time::now(), "survive_world", "map"));

        trackerA.lookup_transform_from_map();
        trackerB.lookup_transform_from_map();
        trackerC.lookup_transform_from_map();
        trackerD.lookup_transform_from_map();
        trackerE.lookup_transform_from_map();

        trackerA.print_tracker_pose();
        trackerB.print_tracker_pose();
        trackerC.print_tracker_pose();
        trackerD.print_tracker_pose();
        trackerE.print_tracker_pose();

        // vive_pose.pose.orientation.w = trackerA.transform_from_map0.getRotation().getW();
        // vive_pose.pose.orientation.x = trackerA.transform_from_map0.getRotation().getX();
        // vive_pose.pose.orientation.y = trackerA.transform_from_map0.getRotation().getY();
        // vive_pose.pose.orientation.z = trackerA.transform_from_map0.getRotation().getZ();
        // vive_pose.pose.position.x = trackerA.transform_from_map0.getOrigin().getX();
        // vive_pose.pose.position.y = trackerA.transform_from_map0.getOrigin().getY();
        // vive_pose.pose.position.z = trackerA.transform_from_map0.getOrigin().getZ();
        // vive_pose.header.stamp = ros::Time::now();
        // vive_pose.header.frame_id = "map";
        // pose_pub.publish(vive_pose);
        // pose_pub.publish(avg_pose(br));
        rate.sleep();
    }

    // thenode.deleteParam();
    printf("delete param\n");
    // survive_simple_close(actx);
    printf("Cleaning up\n");
    return 0;
}



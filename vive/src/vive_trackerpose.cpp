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

// class theNode {
// public:
//     theNode(ros::NodeHandle& nh_): nh(nh_) {};

//     void initialize() {
//         bool get_param_ok;

//         // initTrackers();
//     }
//     void deleteParam()
//     {
//         auto node_name = ros::this_node::getName();


//     }
// private:
//     ros::NodeHandle nh;
// };
// theNode::theNode(ros::NodeHandle nh_) {
//     nh = nh_;
// }

class Tracker {
public:
    Tracker();
    Tracker(char lt, std::string sn) {
        letter = lt;
        serial_num = sn;
        frame = "tracker";
        frame.push_back(letter);
        used = false;
    }
    // void initTracker(char lt, std::string sn) {
    //     letter = lt;
    //     serial_num = sn;
    //     frame = "tracker";
    //     frame.push_back(letter);
    // }
    // void set_and_send_transform_from_world(tf::TransformBroadcaster br_, SurvivePose pose_) {
    //     transform_from_world.setOrigin(tf::Vector3(pose_.Pos[0], pose_.Pos[1], pose_.Pos[2]));
    //     transform_from_world.setRotation(tf::Quaternion(pose_.Rot[1], pose_.Rot[2], pose_.Rot[3], pose_.Rot[0]));
    //     br_.sendTransform(tf::StampedTransform(
    //         transform_from_world, ros::Time::now(), "survive_world", frame));
    // }
    void lookup_transform_from_map() {
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
    void print_tracker_pose() {
        if (used) {
            printf("tracker%c pose: (x y z W X Y Z)\n", letter);
            printf("%f, %f, %f, %f, %f, %f, %f\n", transform_from_map.getOrigin().x(),
                transform_from_map.getOrigin().y(), transform_from_map.getOrigin().z(),
                transform_from_map.getRotation().getW(), transform_from_map.getRotation().getX(),
                transform_from_map.getRotation().getY(), transform_from_map.getRotation().getZ());
        }
        else {
            printf("tracker%c not used", letter);
        }
    }
private:
    std::string serial_num;
    std::string frame;
    char letter;
    bool used;
    tf::StampedTransform transform_from_map;
    tf::StampedTransform transform_from_world;
    tf::TransformListener listener_;
};
Tracker::Tracker() {
    used = false;
}

class Lighthouse {
public:
    Lighthouse();
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
    // void set_transform_form_world(SurvivePose pose_) {
    //     transform_from_world.setOrigin(
    //         tf::Vector3(pose_.Pos[0], pose_.Pos[1], pose_.Pos[2]));
    //     transform_from_world.setRotation(
    //         tf::Quaternion(pose_.Rot[1], pose_.Rot[2], pose_.Rot[3], pose_.Rot[0]));
    // }
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
    Map();
    Map(int od) {
        order = od;
        frame = "map" + std::to_string(order);
    }
    int order;
    std::string frame;
    tf::StampedTransform transform_from_world;
    void lookup_transform_from_world();
private:
};
Map::Map() {
    frame = "map";
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



// VIVEPOSE LH0, LH1, LH2;
// Tracker trackers[5];
// Lighthouse lh0(0, "LHB-400B1A3E");
// Lighthouse lh1(1, "LHB-D4EEE18");
// Lighthouse lh2(2, "LHB-2BEE096A");
int freq = 21;
int unit = 1;

// static void log_fn(SurviveSimpleContext* actx, SurviveLogLevel logLevel, const char* msg) {
//     fprintf(stderr, "(%7.3f) SimpleApi: %s\n", survive_simple_run_time(actx), msg);
// }

// void initTrackers() {
//     trackers[0].initTracker('A', "LHR-94135635");
//     trackers[1].initTracker('B', "LHR-15565625");
//     trackers[2].initTracker('C', "LHR-662B1E75");
//     trackers[3].initTracker('D', "LHR-38203A4C");
//     trackers[4].initTracker('E', "LHR-E833C29B");
// }


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
    // theNode thenode(nh);
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

    // thenode.initialize();
    auto node_name = ros::this_node::getName();
    nh.getParam(node_name + "/freq", freq);
    nh.getParam(node_name + "/unit", unit);
    ros::Rate rate(freq);
    geometry_msgs::PoseStamped vive_pose;
    // SurviveSimpleContext* actx = survive_simple_init_with_logger(argc, argv, log_fn);
    // if (actx == 0) // implies -help or similiar
    //     return 0;

    // double start_time = OGGetAbsoluteTime();
    // survive_simple_start_thread(actx);

    // for (const SurviveSimpleObject* it = survive_simple_get_first_object(actx); it != 0;
    //     it = survive_simple_get_next_object(actx, it)) {
    //     printf("Found '%s'\n", survive_simple_object_name(it));
    // }

    // struct SurviveSimpleEvent event = {};
    // while (keepRunning && survive_simple_wait_for_event(actx, &event) != SurviveSimpleEventType_Shutdown && ros::ok()) {
    //     for (const SurviveSimpleObject* it = survive_simple_get_first_object(actx);
    //         it != 0; it = survive_simple_get_next_object(actx, it)) {
    //         SurvivePose pose;
    //         survive_simple_object_get_latest_pose(it, &pose);
    //         printf("%s, %s, %f, %f, %f, %f, %f, %f, %f\n",
    //             survive_simple_object_name(it), survive_simple_serial_number(it),
    //             pose.Pos[0], pose.Pos[1], pose.Pos[2],
    //             pose.Rot[0], pose.Rot[1], pose.Rot[2], pose.Rot[3]);
    //         if (survive_simple_object_get_type(it) == SurviveSimpleObject_OBJECT) {
    //             SurviveVelocity velocity;
    //             survive_simple_object_get_latest_velocity(it, &velocity);
                // printf("%s velocity : \nx : %f\ny : %f\nz : %f\n", survive_simple_object_name(it),
                //     velocity.Pos[0], velocity.Pos[1], velocity.Pos[2]);
        //         if (strcmp(survive_simple_serial_number(it), trackerA.serial_num.c_str()) == 0)
        //         {
        //             trackerA.set_and_send_transform_from_world(br, pose);
        //         }
        //         else if (strcmp(survive_simple_serial_number(it), trackerB.serial_num.c_str()) == 0)
        //         {
        //             trackerB.set_and_send_transform_from_world(br, pose);
        //         }
        //         else if (strcmp(survive_simple_serial_number(it), trackerC.serial_num.c_str()) == 0)
        //         {
        //             trackerC.set_and_send_transform_from_world(br, pose);
        //         }
        //         else if (strcmp(survive_simple_serial_number(it), trackerD.serial_num.c_str()) == 0)
        //         {
        //             trackerD.set_and_send_transform_from_world(br, pose);
        //         }
        //         else if (strcmp(survive_simple_serial_number(it), trackerE.serial_num.c_str()) == 0)
        //         {
        //             trackerE.set_and_send_transform_from_world(br, pose);
        //         }

        //     }
        //     else if (survive_simple_object_get_type(it) == SurviveSimpleObject_LIGHTHOUSE) {
        //         if (strcmp(survive_simple_serial_number(it), lh0.serial_num.c_str()) == 0) {
        //             lh0.set_transform_form_world(pose);
        //             lh0.send_transform_from_world(br);
        //             lh0.send_transform_to_map(br);
        //         }
        //         else if (strcmp(survive_simple_serial_number(it), lh1.serial_num.c_str()) == 0) {
        //             lh1.set_transform_form_world(pose);
        //             lh1.send_transform_from_world(br);
        //             lh1.send_transform_to_map(br);
        //         }
        //         else if (strcmp(survive_simple_serial_number(it), lh2.serial_num.c_str()) == 0) {
        //             lh2.set_transform_form_world(pose);
        //             lh2.send_transform_from_world(br);
        //             lh2.send_transform_to_map(br);
        //         }
        //     }
        // }

    // try {
    //     listener.lookupTransform("survive_world", map0.frame, ros::Time(0), map0.transform_from_world);
    //     listener.lookupTransform("survive_world", map1.frame, ros::Time(0), map1.transform_from_world);
    //     listener.lookupTransform("survive_world", map2.frame, ros::Time(0), map2.transform_from_world);
    // }
    // catch (tf::TransformException& ex) {
    //     ROS_ERROR("%s", ex.what());
    // }

    while (ros::ok()) {


        map0.lookup_transform_from_world();
        map1.lookup_transform_from_world();
        map2.lookup_transform_from_world();
        map = find_map_avg(map0, map1, map2);
        br.sendTransform(tf::StampedTransform(map.transform_from_world, ros::Time::now(), "survive_world", "map"));

        // for (int t = 0; t < 5; t++) {
        //     if (listener.canTransform("map", trackers[t].frame, ros::Time(0))) {
        //         printf("can transform map to track%c", trackers[t].letter);
        //         trackers[t].used = true;
        //         try {
        //             listener.lookupTransform("map", trackers[t].frame, ros::Time(0), trackers[t].transform_from_map);
        //         }
        //         catch (tf::TransformException& ex) {
        //             ROS_ERROR("%s", ex.what());
        //         }

        //     }
        // }
        trackerA.lookup_transform_from_map();
        trackerB.lookup_transform_from_map();
        trackerC.lookup_transform_from_map();
        trackerD.lookup_transform_from_map();
        trackerE.lookup_transform_from_map();


        // printf("transform: map0 to tracker_A\n");
        // printf("%f, %f, %f\n", trackerA.transform_from_map0.getOrigin().x() * unit,
        //     trackerA.transform_from_map0.getOrigin().y() * unit, trackerA.transform_from_map0.getOrigin().z() * unit);
        // printf("transform: map1 to tracker_A\n");
        // printf("%f, %f, %f\n", trackerA.transform_from_map1.getOrigin().x() * unit,
        //     trackerA.transform_from_map1.getOrigin().y() * unit, trackerA.transform_from_map1.getOrigin().z() * unit);
        // printf("transform: map2 to tracker_A\n");
        // printf("%f, %f, %f\n", trackerA.transform_from_map2.getOrigin().x() * unit,
        //     trackerA.transform_from_map2.getOrigin().y() * unit, trackerA.transform_from_map2.getOrigin().z() * unit);

        // printf("transform: map0 to trackerB\n");
        // printf("%f, %f, %f\n", trackerB.transform_from_map0.getOrigin().x() * unit,
        //     trackerB.transform_from_map0.getOrigin().y() * unit, trackerB.transform_from_map0.getOrigin().z() * unit);
        // printf("transform: map1 to trackerB\n");
        // printf("%f, %f, %f\n", trackerB.transform_from_map1.getOrigin().x() * unit,
        //     trackerB.transform_from_map1.getOrigin().y() * unit, trackerB.transform_from_map1.getOrigin().z() * unit);
        // printf("transform: map2 to trackerB\n");
        // printf("%f, %f, %f\n", trackerB.transform_from_map2.getOrigin().x() * unit,
        //     trackerB.transform_from_map2.getOrigin().y() * unit, trackerB.transform_from_map2.getOrigin().z() * unit);

        trackerA.print_tracker_pose();
        trackerB.print_tracker_pose();
        trackerC.print_tracker_pose();
        trackerD.print_tracker_pose();
        trackerE.print_tracker_pose();

        // }
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
    std::string deleteparam = "rosparam delete " + node_name;
    system(deleteparam.c_str());
    printf("delete param\n");
    // survive_simple_close(actx);
    printf("Cleaning up\n");
    return 0;
}



#include <stdio.h>
#include <string.h>
#include <survive_api.h>
#include <os_generic.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

typedef struct vivePose {
    double x, y, z;
    double W, X, Y, Z;
    double yaw, roll, pitch;
}VIVEPOSE;

class Vive
{
public:
    Vive(ros::NodeHandle nh, ros::NodeHandle nh_local): nh(nh), nh_local(nh_local) {}
    ~Vive();
    void initialize();
    bool survive_start(int argc, char** argv);
    void survive_thread();

private:

    void Publisher_pose(tf::StampedTransform tf_pose);
    void Publisher_vel(SurviveVelocity vel);
    // void timerCallback(const ros::TimerEvent &e);

    //ros tools
    ros::NodeHandle nh;
    ros::NodeHandle nh_local;
    ros::Publisher pose_pub;
    ros::Publisher vel_pub;
    // ros::Timer timer_;

    //tf tools
    tf::TransformBroadcaster br;
    tf::TransformListener listener;

    //tf object
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

    //param
    int freq = 21;
    int unit = 1;
    double start_time;
    SurviveSimpleContext* actx;

};
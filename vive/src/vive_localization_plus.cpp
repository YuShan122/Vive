#include <vive_localization_plus.h>

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



VIVEPOSE LH0, LH1;


Vive::~Vive()
{
    auto node_name = ros::this_node::getName();
    std::string deleteparam = "rosparam delete " + node_name;
    system(deleteparam.c_str());
    printf("delete param\n");

    survive_simple_close(actx);
    printf("cleaning up\n");
}

void Vive::initialize() {
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("vive_pose",10);
    nh_local.getParam("/LH0_x", LH0.x);
    nh_local.getParam("/LH0_y", LH0.y);
    nh_local.getParam("/LH0_z", LH0.z);
    nh_local.getParam("/LH0_W", LH0.W);
    nh_local.getParam("/LH0_X", LH0.X);
    nh_local.getParam("/LH0_Y", LH0.Y);
    nh_local.getParam("/LH0_Z", LH0.Z);

    nh_local.getParam("/LH1_x", LH1.x);
    nh_local.getParam("/LH1_y", LH1.y);
    nh_local.getParam("/LH1_z", LH1.z);
    nh_local.getParam("/LH1_W", LH1.W);
    nh_local.getParam("/LH1_X", LH1.X);
    nh_local.getParam("/LH1_Y", LH1.Y);
    nh_local.getParam("/LH1_Z", LH1.Z);
    nh_local.getParam("/freq", freq);
    nh_local.getParam("/unit", unit);

    transform_LH0ToMap0.setOrigin(tf::Vector3(LH0.x, LH0.y, LH0.z));
    transform_LH0ToMap0.setRotation(tf::Quaternion(LH0.X, LH0.Y, LH0.Z, LH0.W));
    transform_LH1ToMap1.setOrigin(tf::Vector3(LH1.x, LH1.y, LH1.z));
    transform_LH1ToMap1.setRotation(tf::Quaternion(LH1.X, LH1.Y, LH1.Z, LH1.W));

    // timer_ = nh.createTimer(ros::Duration(1.0 / freq), &Vive::timerCallback, this, false, false);
    // timer_.setPeriod(ros::Duration(1.0 / freq), false);
}

void Vive::Publisher_pose(tf::StampedTransform tf_pose)
{
    geometry_msgs::PoseStamped pose_;
    pose_.pose.orientation.x = tf_pose.getRotation().getX();
    pose_.pose.orientation.w = tf_pose.getRotation().getW();
    pose_.pose.orientation.z = tf_pose.getRotation().getZ();
    pose_.pose.orientation.y = tf_pose.getRotation().getY();
    pose_.pose.position.x = tf_pose.getOrigin().getX();
    pose_.pose.position.y = tf_pose.getOrigin().getY();
    pose_.pose.position.z = tf_pose.getOrigin().getZ();
    pose_.header.stamp = ros::Time::now();
    pose_.header.frame_id = "map";
    pose_pub.publish(pose_);
}

bool Vive::survive_start(int argc, char** argv)
{
    actx = survive_simple_init(argc, argv);
    if (actx == 0) // implies -help or similiar
        return 0;

    start_time = OGGetAbsoluteTime();
    survive_simple_start_thread(actx);
    for (const SurviveSimpleObject* it = survive_simple_get_first_object(actx); it != 0;
        it = survive_simple_get_next_object(actx, it)) {
        printf("Found '%s'\n", survive_simple_object_name(it));
    }
    // timer_.start();
    return 1;
}

// void Vive::timerCallback(const ros::TimerEvent &e)

void Vive::survive_thread()
{
    printf("start thread\n");
    ros::Rate rate(freq);
    struct SurviveSimpleEvent event = {};
    while (keepRunning && survive_simple_wait_for_event(actx, &event) != SurviveSimpleEventType_Shutdown && ros::ok()) {
        switch (event.event_type) {
        case SurviveSimpleEventType_PoseUpdateEvent: {
            for (const SurviveSimpleObject* it = survive_simple_get_first_object(actx);
                it != 0; it = survive_simple_get_next_object(actx, it)) {
                SurvivePose pose;
                SurviveVelocity velocity;
                survive_simple_object_get_latest_pose(it, &pose);
                if (survive_simple_object_get_type(it) == SurviveSimpleObject_OBJECT) {
                    survive_simple_object_get_latest_velocity(it, &velocity);
                    printf("%s velocity : \nx : %f\ny : %f\nz : %f\n",survive_simple_object_name(it), velocity.Pos[0], velocity.Pos[1], velocity.Pos[2]);
                    transform_surviveWorldToTracker.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                    transform_surviveWorldToTracker.setRotation(tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                }
                else if (survive_simple_object_get_type(it) == SurviveSimpleObject_LIGHTHOUSE) {
                    if (strcmp(survive_simple_serial_number(it), "LHB-400B1A3E")) {
                        transform_surviveWorldToLH0.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                        transform_surviveWorldToLH0.setRotation(tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                    }
                    if (strcmp(survive_simple_serial_number(it), "LHB-D4EEE18")) {
                        transform_surviveWorldToLH1.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                        transform_surviveWorldToLH1.setRotation(tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                    }
                }
            }
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToTracker, ros::Time::now(), "survive_world", "tracker"));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH0, ros::Time::now(), "survive_world", "LH0"));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH1, ros::Time::now(), "survive_world", "LH1"));
            br.sendTransform(tf::StampedTransform(transform_LH0ToMap0, ros::Time::now(), "LH0", "map0"));
            br.sendTransform(tf::StampedTransform(transform_LH1ToMap1, ros::Time::now(), "LH1", "map1"));
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
        Publisher_pose(transform_map0ToTracker);
        rate.sleep();
    }
    survive_simple_close(actx);
    printf("exit thread\n");
}

int main(int argc, char** argv) {

#ifdef __linux__
    signal(SIGINT, intHandler);
    signal(SIGTERM, intHandler);
    signal(SIGKILL, intHandler);
#endif

    ros::init(argc, argv, "vive_localization_plus");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");
    Vive vive(nh, nh_local);
    vive.initialize();
    bool start_device_ok = vive.survive_start(argc, argv);
    start_device_ok = 1 ? printf("successfully start timer\n") : printf("start error\n") ;

    vive.survive_thread();
    return 0;
}



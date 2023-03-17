#include <predict_enemy_vel.h>

Now_Status now_status;

void VIVE_ENEMY::initialize()
{
    pose_sub = nh_.subscribe<geometry_msgs::Twist>("vive_pose", 20, &VIVE_ENEMY::pose_callback, this);
    SimpleNumber = 5;
    list_index = 0;
    vel_list.clear();
    vel_list.reserve(SimpleNumber);
    now_status.now_vel = 0;
    now_status.max_vel = 0;
    now_status.status = STATUS::STOP;
}

VIVE_ENEMY::~VIVE_ENEMY()
{

}

void VIVE_ENEMY::pose_callback(const geometry_msgs::Twist::ConstPtr& vel_msg)
{
    vel_list[list_index] = sqrt((vel_msg->linear.x) ^ 2 + (vel_msg->linear.y) ^ 2);
}

void VIVE_ENEMY::get_data()
{
    ros::Rate rate(timeInterval);
    float total_vel = 0;
    float avage_vel = 0;
    while (list_index < SimpleNumber)
    {
        ros::spin();
        total_vel += vel_list[list_index];
        list_index++;
        rate.sleep();
    }
    list_index = 0;
    avage_vel = total_vel / SimpleNumber;
    analysis(avage_vel);
}

void VIVE_ENEMY::analysis(float vel_data)
{
    offset = vel_data - now_status.now_vel;
    if (offset >= accel_min_tole && offset <= accel_max_tole)
    {
        now_status.max_vel = vel_data;
        now_status.status = STATUS::ACCEL;
    }
    else if (abs(offset) < accel_min_tole&&)
    {
        now_status.status = STATUS::KEEP_VEL;
    }
    now_status.now_vel = vel_data;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "predict_enemy_vel");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");
    VIVE_ENEMY vive_enemy(nh, nh_local);

}
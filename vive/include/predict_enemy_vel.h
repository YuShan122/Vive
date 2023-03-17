#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <vector>

enum class STATUS
{
    ACCEL,
    DECEL,
    STOP,
    MAX_VEL,
    KEEP_VEL
};

class Now_Status
{
    float now_vel;
    float max_vel;
    STATUS status;
};


class VIVE_ENEMY
{
public:
    VIVE_ENEMY(ros::NodeHandle nh, ros::NodeHandle nh_local) : nh_(nh), nh_local_(nh_local) {}
    ~VIVE_ENEMY();
    void initialize();
    
private:
    void pose_callback(const geometry_msgs::Twist::ConstPtr &vel_msg);
    void analysis(float vel_data);
    void get_data();
    
    ros::Subscriber pose_sub;
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
  
    std::vector<float> vel_list;
    int list_index;
    float offset;

    float timeInterval;
    int SimpleNumber;
    float accel_min_tole;
    float accel_max_tole;
};




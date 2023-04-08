#include <costmap_converter/ObstacleArrayMsg.h>
#include <costmap_converter/ObstacleMsg.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/tf.h>


typedef struct ODOMINfO{
    double x, y;
    double Vx, Vy;
    std_msgs::Header header;
}OdomInfo;

OdomInfo rival1_odom,rival2_odom; 
std::vector<OdomInfo> Lidar_vec;
double freq;
bool fusion_switch;

class RivalMulti{
    public:
        RivalMulti(ros::NodeHandle nh_g, ros::NodeHandle nh_p);
        void Rival1_callback(const nav_msgs::Odometry::ConstPtr &rival1_msg);
        void Rival2_callback(const nav_msgs::Odometry::ConstPtr &rival2_msg);
        void Lidar_callback(const  nav_msgs::Odometry::ConstPtr &lidar_msg);

        bool Rival_match(std::string name, OdomInfo rival_data, std::vector<OdomInfo>& Lidar_vec);
        double distance(double a_x, double a_y, double b_x, double b_y);
        void publish_rival_odom(std::string name, OdomInfo odom_data);
        bool distribute_rival_odom(bool rival1_ok, bool rival2_ok, std::vector<OdomInfo>& Lidar_vec);
        bool boundary(double x, double y);

    private:
        ros::NodeHandle nh;
        ros::NodeHandle nh_local;
        ros::Subscriber rival1_sub;
        ros::Subscriber rival2_sub;
        ros::Subscriber lidar_sub;
        ros::Publisher rival1_pub;
        ros::Publisher rival2_pub;

        std::string node_name;
        std::string rival1_topic_name;
        std::string rival2_topic_name;
        std::string lidar_topic_name;

        double match_tole;
        double boundary_upper_x;
        double boundary_down_x;
        double boundary_upper_y;
        double boundary_down_y;
};

RivalMulti::RivalMulti(ros::NodeHandle nh_g, ros::NodeHandle nh_p){
    nh = nh_g;
    nh_local = nh_p;

    node_name = ros::this_node::getName();

    bool ok = true;
    ok &= nh_local.getParam("freq", freq);
    ok &= nh_local.getParam("rival1_topic_name", rival1_topic_name);
    ok &= nh_local.getParam("rival2_topic_name", rival2_topic_name);
    ok &= nh_local.getParam("lidar_topic_name", lidar_topic_name);
    ok &= nh_local.getParam("match_tole", match_tole);
    ok &= nh_local.getParam("boundary_upper_x", boundary_upper_x);
    ok &= nh_local.getParam("boundary_down_x", boundary_down_x);
    ok &= nh_local.getParam("boundary_upper_y", boundary_upper_y);
    ok &= nh_local.getParam("boundary_down_y", boundary_down_y);
    ok &= nh_local.getParam("fusion_switch", fusion_switch);

    rival1_sub = nh.subscribe("/rival1/odom/tracker",10, &RivalMulti::Rival1_callback, this);
    rival2_sub = nh.subscribe("/rival2/odom/tracker",10, &RivalMulti::Rival2_callback, this);
    lidar_sub = nh.subscribe(lidar_topic_name, 10, &RivalMulti::Lidar_callback, this);
    rival1_pub = nh.advertise<nav_msgs::Odometry>(rival1_topic_name, 10);
    rival2_pub = nh.advertise<nav_msgs::Odometry>(rival2_topic_name, 10);

    std::cout << node_name << " freq : " << freq << "\n";
    if (ok) std::cout << node_name << " get parameters of the robot sucessed.\n";
    else std::cout << node_name << " get parameters of robot failed.\n";
}

void RivalMulti::Rival1_callback(const nav_msgs::Odometry::ConstPtr &rival1_msg){
    rival1_odom.header.stamp = rival1_msg->header.stamp;
    rival1_odom.x = rival1_msg->pose.pose.position.x;
    rival1_odom.y = rival1_msg->pose.pose.position.y;
    rival1_odom.Vx = rival1_msg->twist.twist.linear.x;
    rival1_odom.Vy = rival1_msg->twist.twist.linear.y;
}

void RivalMulti::Rival2_callback(const nav_msgs::Odometry::ConstPtr &rival2_msg){
    rival2_odom.header.stamp = rival2_msg->header.stamp;
    rival2_odom.x = rival2_msg->pose.pose.position.x;
    rival2_odom.y = rival2_msg->pose.pose.position.y;
    rival2_odom.Vx = rival2_msg->twist.twist.linear.x;
    rival2_odom.Vy = rival2_msg->twist.twist.linear.y;
}

void RivalMulti::Lidar_callback(const nav_msgs::Odometry::ConstPtr &lidar_msg){
    Lidar_vec.clear();
    OdomInfo lidar_data;
    // write the lidar_msg in lidar_data
    Lidar_vec.push_back(lidar_data);
}

bool RivalMulti::Rival_match(std::string name, OdomInfo rival_data, std::vector<OdomInfo>& Lidar_vec){
    bool match_status = true;
    double match_error;

    std::vector<OdomInfo>::iterator it;
    for(it = Lidar_vec.begin(); it!=Lidar_vec.end(); it++){
        match_error = distance(it->x, it->y, rival_data.x, rival_data.y);
        if(match_error <= match_tole){
            it = Lidar_vec.erase(it);
            // Lidar_vec.erase(remove(Lidar_vec.begin(), Lidar_vec.end(), *it), Lidar_vec.end());
            printf("%s match successful : %f\n", name.c_str(), match_error);
            return true;
        }        
    }
    printf("%s match failed.\n", name.c_str());
    return false;
}

double RivalMulti::distance(double a_x, double a_y, double b_x, double b_y){
    return sqrt( pow((a_x-b_x),2) + pow((a_y-b_y),2) );
}

void RivalMulti::publish_rival_odom(std::string name, OdomInfo odom_data){
    nav_msgs::Odometry rival_odom;
    double time_delay;

    rival_odom.header.stamp = ros::Time::now();
    rival_odom.pose.pose.position.x = odom_data.x;
    rival_odom.pose.pose.position.y = odom_data.y;
    rival_odom.twist.twist.linear.x = odom_data.Vx;
    rival_odom.twist.twist.linear.y = odom_data.Vy;
    time_delay = rival_odom.header.stamp.toSec() - odom_data.header.stamp.toSec();
    if(strcmp(name.c_str(), "rival1") == 0){
        rival1_pub.publish(rival_odom);
        printf("%s publish time delay : %f\n", name.c_str(), time_delay);
    }
    else if(strcmp(name.c_str(), "rival2") == 0){
        rival2_pub.publish(rival_odom);
        printf("%s publish time delay : %f\n", name.c_str(), time_delay);
    }
}

bool RivalMulti::distribute_rival_odom(bool rival1_ok, bool rival2_ok, std::vector<OdomInfo>& Lidar_vec){
    if(rival1_ok && rival2_ok){
        return 0;
    }  
    bool boundary_ok = false;
    if(!rival1_ok){
        std::vector<OdomInfo>::iterator it;
        for(it = Lidar_vec.begin(); it!=Lidar_vec.end(); it++){
            boundary_ok = boundary(it->x, it->y);
            if(boundary_ok){
                publish_rival_odom("rival1", *it);
                printf("Rival1 use lidar info to publish\n");
                it = Lidar_vec.erase(it);
                rival1_ok = true;
            }
        }
    }
    if(!rival2_ok){
        std::vector<OdomInfo>::iterator it;
        for(it = Lidar_vec.begin(); it!=Lidar_vec.end(); it++){
            boundary_ok = boundary(it->x, it->y);
            if(boundary_ok){
                publish_rival_odom("rival2", *it);
                printf("Rival2 use lidar info to publish\n");
                it = Lidar_vec.erase(it);
                rival2_ok = true;
            }
        }
    }
    if(rival1_ok && rival2_ok){
        return true;
    }
    else{
        return false;
    }
}

bool RivalMulti::boundary(double x, double y){
    if(x <= boundary_upper_x && x >= boundary_down_x && x!=0){
        if(y <= boundary_upper_y && y >= boundary_down_y && y!=0){
            return true;
        }
    }
    return false;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_rival_detector");
    ros::NodeHandle nh; 
    ros::NodeHandle nh_("~");

    RivalMulti rivalmulti(nh, nh_);
    ros::Rate rate(freq);

    bool rival1_ok, rival2_ok;
    bool distribute_ok;
    while (ros::ok()) {
        ros::spinOnce();
        if(fusion_switch){
            rival1_ok = rivalmulti.Rival_match("rival1", rival1_odom, Lidar_vec);
            rival2_ok = rivalmulti.Rival_match("rival2", rival2_odom, Lidar_vec);
            if(rival1_ok){
                rivalmulti.publish_rival_odom("rival1", rival1_odom);
                printf("successful publish rival1 odom\n");
            }
            if(rival2_ok){
                rivalmulti.publish_rival_odom("rival2", rival2_odom);
                printf("successful publish rival2 odom\n");
            }
            distribute_ok = rivalmulti.distribute_rival_odom(rival1_ok, rival2_ok, Lidar_vec);
            if(!distribute_ok) printf("distribute failure\n");
        }
        else{
            printf("Just use the tracker data\n");
            rivalmulti.publish_rival_odom("rival1", rival1_odom);
            printf("successful publish rival1 odom\n");
            rivalmulti.publish_rival_odom("rival2", rival1_odom);
            printf("successful publish rival2 odom\n");
        }
        rate.sleep();
    }
}
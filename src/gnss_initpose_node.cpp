#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <string>
#include <cmath>

extern "C"{
    #include<proj_api.h>
}

using std::string;
using std::cos;
using std::sin;

class gnss_initpose_node
{

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::ServiceServer srv_;

    tf2_ros::TransformBroadcaster br_;
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PoseWithCovarianceStamped pose;


    string base_frame_, map_frame_;
    string gnss_topic_, point_topic_;

    void gnss_cb(const sensor_msgs::NavSatFixConstPtr& gnss_msg);
    void send_pose(const double latitude, const double longitude);
    void srv_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    projPJ pj_latlong_, pj_coord_;
    double map_lat_, map_long_, map_x_, map_y_;
    double robot_lat_, robot_long_, robot_x_, robot_y_;
    double map_frame_direction_;
    string latlong_param_, coord_param_;

public:
    gnss_initpose_node(/* args */);
    ~gnss_initpose_node();
};

gnss_initpose_node::gnss_initpose_node(/* args */)
{
    ros::NodeHandle pnh_("~");

    pnh_.param<string>("map_frame",map_frame_,"map");
    pnh_.param<string>("base_frame",base_frame_,"base_link");
    pnh_.param<string>("gnss_topic",gnss_topic_,"fix");   
    pnh_.param<string>("point_topic",point_topic_,"initial_pose");
    pnh_.param<string>("latlong_param",latlong_param_,"+proj=latlong +ellps=WGS84");
    pnh_.param<string>("coord_param",coord_param_,"+init=epsg:2451");
    pnh_.param<double>("map_lat", map_lat_,0);
    pnh_.param<double>("map_long", map_long_,0);
    pnh_.param<double>("map_frame_direction",map_frame_direction_,0);

    sub_ = nh_.subscribe<sensor_msgs::NavSatFix>(gnss_topic_,1,&gnss_initpose_node::gnss_cb,this);
    pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(point_topic_,1);

    if(!(pj_latlong_ = pj_init_plus(latlong_param_.c_str()))){
        ROS_ERROR("Exception Parameter latlong_param = %s",latlong_param_.c_str());
        return;
    }

    if(!(pj_coord_ = pj_init_plus(coord_param_.c_str()))){
        ROS_ERROR("Exception Parameter coord_param = %s",coord_param_.c_str());
        return;
    } 

    map_x_ = map_long_ * DEG_TO_RAD;
    map_y_ = map_lat_ * DEG_TO_RAD;

    int p = pj_transform(pj_latlong_, pj_coord_, 1, 1, &map_x_, &map_y_,NULL);

    if(p != 0){
        ROS_ERROR("ERROR DETECTED pj_transform()");
        ROS_ERROR("ERROR_CODE = %d",p);
        ROS_ERROR("ERROR DETAIL = %s",pj_strerrno(p));
    return;
    }

}

gnss_initpose_node::~gnss_initpose_node()
{
    pj_free(pj_latlong_);
    pj_free(pj_coord_);
}

void 
gnss_initpose_node::gnss_cb(const sensor_msgs::NavSatFixConstPtr& gnss_msg){
    robot_lat_ = gnss_msg->latitude;
    robot_long_ = gnss_msg->longitude;
}

void 
gnss_initpose_node::send_pose(const double latitude, const double longitude){

    robot_x_ = longitude * DEG_TO_RAD;
    robot_y_ = latitude * DEG_TO_RAD;

    int p = pj_transform(pj_latlong_, pj_coord_, 1, 1, &robot_x_, &robot_y_,NULL);

    if(p != 0){
        ROS_ERROR("ERROR DETECTED pj_transform()");
        ROS_ERROR("ERROR_CODE = %d",p);
        ROS_ERROR("ERROR DETAIL = %s",pj_strerrno(p));
    return;
    }

    //Rotation matrix2d
    robot_x_ = cos(map_frame_direction_) * robot_x_ - sin(map_frame_direction_) * robot_y_;
    robot_y_ = sin(map_frame_direction_) * robot_x_ + sin(map_frame_direction_) * robot_y_;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = map_frame_;
    transformStamped.child_frame_id = base_frame_;
    transformStamped.transform.translation.x = robot_x_-map_x_;
    transformStamped.transform.translation.y = robot_y_-map_y_;
    transformStamped.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setRPY(0,0,0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br_.sendTransform(transformStamped);

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = base_frame_;
    pose.pose.pose.position.x = robot_x_-map_x_;
    pose.pose.pose.position.y = robot_y_-map_y_;
    pose.pose.pose.position.z = 0;
    pose.pose.pose.orientation.x = 0;
    pose.pose.pose.orientation.y = 0;
    pose.pose.pose.orientation.z = 0;
    pose.pose.pose.orientation.w = 1;

    pub_.publish(pose);

}

void
gnss_initpose_node::srv_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    send_pose(robot_lat_,robot_long_);

}

int main(int argc, char** argv){
    ros::init(argc, argv, "gnss_initpose_node");
    gnss_initpose_node gnss;

    ros::spin();

    return 0;
}
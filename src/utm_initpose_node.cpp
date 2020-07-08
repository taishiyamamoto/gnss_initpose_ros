#include <mutex>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>

    using std::cos;
    using std::sin;
    using std::string;

    class UTMInitposeNode
    {
        private:
            std::mutex mtx_;

            ros::NodeHandle nh_;
            ros::Subscriber sub_;
            ros::Publisher pub1_;
            ros::Publisher pub2_;
            ros::ServiceServer srv_;

            geometry_msgs::PoseWithCovarianceStamped world_pose_;
            geometry_msgs::PoseWithCovarianceStamped map_pose_;
            geometry_msgs::PoseWithCovarianceStamped robot_pose_;

            string base_frame_, map_frame_;
            string utm_topic_;
            string gps_pose_topic_;
            string initial_pose_topic_;
            
            std::vector<double> datum_;

            void utm_cb(const nav_msgs::OdometryConstPtr& utm_msg);
            void trans_coord();
            bool srv_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

        public:
            UTMInitposeNode();
            ~UTMInitposeNode();
    };

    UTMInitposeNode::UTMInitposeNode()
    {   
        ros::NodeHandle pnh_("~");
        pnh_.param<string>("map_frame",map_frame_,"map");
        pnh_.param<string>("base_frame",base_frame_,"base_link");
        pnh_.param<string>("utm_topic",utm_topic_,"gps/odom");
        pnh_.param<string>("initial_pose_topic",initial_pose_topic_,"initialpose");
        pnh_.param<string>("gps_pose_topic",gps_pose_topic_,"gps/position");
        pnh_.getParam("datum",datum_);

        ROS_DEBUG("%s",map_frame_.c_str());
        ROS_DEBUG("%s",base_frame_.c_str());
        ROS_DEBUG("%s",utm_topic_.c_str());
        ROS_DEBUG("%s",initial_pose_topic_.c_str());
        ROS_DEBUG("%s",gps_pose_topic_.c_str());
        ROS_DEBUG("%lf",datum_[0]);
        ROS_DEBUG("%lf",datum_[1]);
        ROS_DEBUG("%lf",datum_[2]);
        map_pose_.pose.pose.position.x = datum_[0];
        map_pose_.pose.pose.position.y = datum_[1];

        sub_ = nh_.subscribe<nav_msgs::Odometry>(utm_topic_,10,&UTMInitposeNode::utm_cb,this);
        pub1_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(initial_pose_topic_,10);
        pub2_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(gps_pose_topic_,10);
        srv_ = nh_.advertiseService("utm_initpose",&UTMInitposeNode::srv_cb,this);
    }

    UTMInitposeNode::~UTMInitposeNode()
    {

    }

    void 
    UTMInitposeNode::utm_cb(const nav_msgs::OdometryConstPtr& utm_msg){
        world_pose_.pose = utm_msg->pose;

        trans_coord();
        robot_pose_.header.stamp = ros::Time::now();
        robot_pose_.header.frame_id = map_frame_;
        pub2_.publish(robot_pose_);
    }

    void 
    UTMInitposeNode::trans_coord(){
        std::lock_guard<std::mutex> lock(mtx_);
        double local_x = world_pose_.pose.pose.position.x - map_pose_.pose.pose.position.x;
        double local_y = world_pose_.pose.pose.position.y - map_pose_.pose.pose.position.y; 
        //double local_z = world_pose_.pose.pose.position.z - map_pose_.pose.pose.position.z;

        //Rotation matrix2d
        robot_pose_.pose.pose.position.x = cos(datum_[2]) * local_x - sin(datum_[2]) * local_y;
        robot_pose_.pose.pose.position.y = sin(datum_[2]) * local_x + cos(datum_[2]) * local_y;
        robot_pose_.pose.pose.position.z = 0;

        robot_pose_.pose.pose.orientation.x = 0;
        robot_pose_.pose.pose.orientation.y = 0;
        robot_pose_.pose.pose.orientation.z = 0;
        robot_pose_.pose.pose.orientation.w = 1;
        robot_pose_.pose.covariance = world_pose_.pose.covariance;
    }

    bool
    UTMInitposeNode::srv_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){

        trans_coord();

        robot_pose_.header.stamp = ros::Time::now();
        robot_pose_.header.frame_id = map_frame_;

        pub1_.publish(robot_pose_);
        res.success = true;

    return res.success;
    }


int main(int argc, char** argv){
    ros::init(argc, argv, "utm_initpose_node");
    
    UTMInitposeNode utm;

    ros::spin();

    return 0;
}
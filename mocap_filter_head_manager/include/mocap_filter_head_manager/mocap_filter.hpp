#ifndef MOCAP_FILTER_HPP
#define MOCAP_FILTER_HPP

#include <ros/ros.h>
#include <optitrack_ros/or_pose_estimator_state.h>
#include <geometry_msgs/PointStamped.h>
#include <dynamic_reconfigure/server.h>

#define PUBLISH_PERIOD_PARAM_NAME "publish_period"
#define NO_DATA_TIMEOUT_PARAM_NAME "no_data_timeout"


struct Params{
    double publishPeriod;
    double noDataTimeout;
};

class MocapFilter{
    public:
    MocapFilter(ros::NodeHandlePtr& nh);
    ~MocapFilter();

    void onNewPoint(const optitrack_ros::or_pose_estimator_stateConstPtr& msg);
    void publishPoint(const ros::TimerEvent& e);

    protected:
    void updateParams();
    ros::NodeHandlePtr nh_;
    ros::Subscriber optitrackSub_;
    ros::Publisher headManagerPub_;
    ros::Timer pubTimer_;
    geometry_msgs::PointStamped lastPoint_;
    ros::Time lastPointTime_;
    Params params_; 
};

#endif /* MOCAP_FILTER_HPP */

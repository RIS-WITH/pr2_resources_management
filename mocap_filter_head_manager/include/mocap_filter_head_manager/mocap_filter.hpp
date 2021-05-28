#ifndef MOCAP_FILTER_HPP
#define MOCAP_FILTER_HPP

#include <ros/ros.h>
#include <optitrack_ros/or_pose_estimator_state.h>
#include <geometry_msgs/PointStamped.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_ros/transform_listener.h>

#define PUBLISH_PERIOD_PARAM_NAME "publish_period"
#define NO_DATA_TIMEOUT_PARAM_NAME "no_data_timeout"
#define STANDARD_DIST_PARAM_NAME "standard_priority_distance"
#define HIGH_DIST_PARAM_NAME "high_priority_distance"
#define DISTANCE_FRAMES_PARAM_NAME "distance_frames"


struct Params{
    double publishPeriod;
    double noDataTimeout;
    double standardDistSq;
    double highDistSq;
    std::vector<std::string> distanceFrames;
};

class MocapFilter{
    public:
    MocapFilter(ros::NodeHandlePtr& nh);
    ~MocapFilter();

    void onNewPoint(const optitrack_ros::or_pose_estimator_stateConstPtr& msg);
    void publishPoint(const ros::TimerEvent& e);
    double getMinSqDistancesLastPointToFrames() const;

    protected:
    void updateParams();
    ros::NodeHandlePtr nh_;
    ros::Subscriber optitrackSub_;
    ros::Publisher headManagerPub_;
    ros::Timer pubTimer_;
    geometry_msgs::PointStamped lastPoint_;
    ros::Time lastPointTime_;
    Params params_;
    tf2_ros::Buffer tfBuffer_;
    std::unique_ptr<tf2_ros::TransformListener> tfListener_;
};

#endif /* MOCAP_FILTER_HPP */

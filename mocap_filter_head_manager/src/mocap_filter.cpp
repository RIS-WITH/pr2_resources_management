#include "mocap_filter.hpp"
#include <pr2_head_manager_msgs/Point.h>


MocapFilter::MocapFilter(ros::NodeHandlePtr& nh): nh_(nh){
    updateParams();
    headManagerPub_ = nh_->advertise<pr2_head_manager_msgs::Point>("/head_manager_output", 1);
    optitrackSub_ = nh_->subscribe("/optitrack_input_topic", 1, &MocapFilter::onNewPoint, this);
    pubTimer_ = nh_->createTimer(ros::Duration(params_.publishPeriod), &MocapFilter::publishPoint, this);
}

MocapFilter::~MocapFilter(){
    pr2_head_manager_msgs::Point pt;
    pt.priority.value = pt.priority.VOID;
    headManagerPub_.publish(pt);
}

void MocapFilter::onNewPoint(const optitrack_ros::or_pose_estimator_stateConstPtr& msg){
    if (msg->pos.size() == 0){
        return;
    }else if (msg->pos.size() > 1){
        ROS_WARN_STREAM("Mutiple body positions returned by optitrack. Should be only 1, while we received " << msg->pos.size());
    }
    lastPointTime_ = ros::Time::now();
    lastPoint_.header.stamp = ros::Time(msg->ts.sec, msg->ts.nsec);
    lastPoint_.header.frame_id = "optitrack";
    lastPoint_.point.x = msg->pos[0].x;
    lastPoint_.point.y = msg->pos[0].y;
    lastPoint_.point.z = msg->pos[0].z;
}

void MocapFilter::publishPoint(const ros::TimerEvent& e){
    pr2_head_manager_msgs::Point pt;
    if (ros::Time::now() - lastPointTime_ > ros::Duration(params_.noDataTimeout)){
        pt.priority.value = pt.priority.VOID;
        headManagerPub_.publish(pt);
        return;
    }
    pt.priority.value = pt.priority.STANDARD;  // TODO: something else?
    pt.data = lastPoint_;
    headManagerPub_.publish(pt);
    updateParams();    
}

void MocapFilter::updateParams(){
    Params p;
    nh_->param(PUBLISH_PERIOD_PARAM_NAME, p.publishPeriod, 0.1);
    nh_->param(NO_DATA_TIMEOUT_PARAM_NAME, p.noDataTimeout, 2.0);

    if (p.publishPeriod != params_.publishPeriod){
        pubTimer_ = nh_->createTimer(p.publishPeriod, &MocapFilter::publishPoint, this);
        params_ = p;
    }else if (p.noDataTimeout != params_.noDataTimeout){
        params_ = p;
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "mocap_filter");
    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

    MocapFilter mf(nh);

    ros::spin();

    return 0;
}
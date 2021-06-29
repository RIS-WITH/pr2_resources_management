#include "mocap_filter.hpp"
#include <pr2_head_manager_msgs/Point.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


MocapFilter::MocapFilter(ros::NodeHandlePtr& nh): nh_(nh){
    updateParams();
    tfListener_ = std::unique_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(tfBuffer_));
    headManagerPub_ = nh_->advertise<pr2_head_manager_msgs::Point>("/head_manager_output", 1);
    optitrackSub_ = nh_->subscribe("/optitrack_input_topic", 10, &MocapFilter::onNewPoint, this);
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

    double minDistSq = getMinSqDistancesLastPointToFrames();
    if (minDistSq <= params_.highDistSq){
        pt.priority.value = pt.priority.STANDARD;
    }else if (minDistSq <= params_.standardDistSq){
        pt.priority.value = pt.priority.LOW;
    }else{
        pt.priority.value = pt.priority.VOID;
    }
    pt.data = lastPoint_;
    headManagerPub_.publish(pt);
    updateParams();    
}

void MocapFilter::updateParams(){

    double oldPublishPeriod = params_.publishPeriod;
    nh_->param(PUBLISH_PERIOD_PARAM_NAME, params_.publishPeriod, 0.1);
    nh_->param(NO_DATA_TIMEOUT_PARAM_NAME, params_.noDataTimeout, 2.0);
    nh_->param(STANDARD_DIST_PARAM_NAME, params_.standardDistSq, 3.0);
    nh_->param(HIGH_DIST_PARAM_NAME, params_.highDistSq, 1.0);
    nh_->param(DISTANCE_FRAMES_PARAM_NAME, params_.distanceFrames, {"base_footprint"});

    params_.standardDistSq *= params_.standardDistSq;
    params_.highDistSq *= params_.highDistSq;

    if (params_.publishPeriod != oldPublishPeriod){
        pubTimer_ = nh_->createTimer(params_.publishPeriod, &MocapFilter::publishPoint, this);
    }
}

double MocapFilter::getMinSqDistancesLastPointToFrames() const{
    geometry_msgs::TransformStamped frame2optitrack;
    tf2::Transform frame2optitrackTf;
    tf2::Vector3 point, result;
    tf2::fromMsg(lastPoint_.point, point);
    double minSqDist = 9999999.0;
    for (const std::string& frame: params_.distanceFrames){
        try{
        frame2optitrack = tfBuffer_.lookupTransform(frame, lastPoint_.header.frame_id,
                                    ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN_STREAM("Can't transform the origin frame of the point of interest: '" << lastPoint_.header.frame_id 
                        << "' into the frame specified in parameter: '" << frame << "'. Exception: " << ex.what());
            continue;
        }
        tf2::fromMsg(frame2optitrack.transform, frame2optitrackTf);
        result = frame2optitrackTf * point;
        minSqDist = std::min(minSqDist, result.length2());
    }
    return minSqDist;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "mocap_filter");
    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

    MocapFilter mf(nh);

    ros::spin();

    return 0;
}
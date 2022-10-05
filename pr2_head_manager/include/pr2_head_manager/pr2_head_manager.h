#include "pr2_head_manager_msgs/StateMachineRegister.h"
#include "pr2_head_manager_msgs/StateMachineExtract.h"
#include "pr2_head_manager_msgs/Point.h"
#include "pr2_head_manager_msgs/PitchYaw.h"
#include "pr2_head_manager_msgs/PrioritizedPitch.h"
#include "pr2_head_manager_msgs/PrioritizedYaw.h"
#include "pr2_head_manager/ArtificialLife.h"

#include <resource_management/ReactiveInputs.h>
#include <resource_management/ResourceManager.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <thread>

#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>

#define NODE_NAME "PR2_HEAD_MANAGER"

class Pr2HeadManager : public resource_management::ResourceManager<pr2_head_manager_msgs::StateMachineRegister
      ,pr2_head_manager_msgs::StateMachineExtract
      ,pr2_head_manager_msgs::Point
      ,pr2_head_manager_msgs::PitchYaw
      ,pr2_head_manager_msgs::PrioritizedPitch
      ,pr2_head_manager_msgs::PrioritizedYaw
>
{
public:
    Pr2HeadManager(const ros::NodeHandlePtr &nh, std::vector<std::string>& plugins, bool synchronized = false):
        ResourceManager (std::move(nh),{"human_head_monitoring", "human_hand_monitoring", "environment_monitoring", "acting", "speaking", "proximity"}, plugins, synchronized)
    {
        // this in lambda is necessary for gcc <= 5.1
        resource_management::MessageWrapper<geometry_msgs::PointStamped>::registerPublishFunction([this](auto data, auto is_new){ this->publishPointMsg(data, is_new); });
        resource_management::MessageWrapper<pr2_head_manager_msgs::RawPitchYaw>::registerPublishFunction([this](auto data, auto is_new){ this->publishPitchYawMsg(data, is_new); });
        resource_management::MessageWrapper<pr2_head_manager_msgs::Pitch>::registerPublishFunction([this](auto data, auto is_new){ this->publishPrioritizedPitchMsg(data, is_new); });
        resource_management::MessageWrapper<pr2_head_manager_msgs::Yaw>::registerPublishFunction([this](auto data, auto is_new){ this->publishPrioritizedYawMsg(data, is_new); });

        // Remove if your do not need artificial life
        _artificialLife = (std::make_shared<pr2_head_manager::ArtificialLife>(_artificialLifeBuffer));

	    point_head_client = new actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>(
        "/head_traj_controller/joint_trajectory_action", true);
    	joint_state_sub = _nh->subscribe("/joint_states", 1, &Pr2HeadManager::onJointState, this);

      ROS_INFO_STREAM(NODE_NAME << ": Waiting for head controller");
      point_head_client->waitForServer();
      ROS_INFO_STREAM(NODE_NAME << ": Head controller found");
      tfListener = new tf2_ros::TransformListener(tfBuffer);
    	watchdog = _nh->createTimer(ros::Duration(0.1), &Pr2HeadManager::onWatchDog, this);
    	ROS_INFO_STREAM(NODE_NAME << ": Online.");
    }

  ~Pr2HeadManager(){
    delete point_head_client;
    delete tfListener;
  }

private:
    std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> stateFromMsg(const pr2_head_manager_msgs::StateMachineRegister::Request &msg) override;
    std::vector<std::tuple<std::string,std::string,resource_management_msgs::EndCondition>>
    transitionFromMsg(const pr2_head_manager_msgs::StateMachine &msg) override;
    pr2_head_manager_msgs::StateMachineRegister::Response generateResponseMsg(uint32_t id) override;

    void publishPointMsg(geometry_msgs::PointStamped msg, bool is_new);
    void publishPitchYawMsg(pr2_head_manager_msgs::RawPitchYaw msg, bool is_new);
    void publishPrioritizedPitchMsg(pr2_head_manager_msgs::Pitch msg, bool is_new);
    void publishPrioritizedYawMsg(pr2_head_manager_msgs::Yaw msg, bool is_new);

    void checkGoalActive();
    void onWatchDog(const ros::TimerEvent&);
    void onJointState(const sensor_msgs::JointStateConstPtr& msg);

    actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction >* point_head_client;
    ros::Subscriber joint_state_sub;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener* tfListener;
    bool _is_command_running;
    ros::Timer watchdog;
    double currentTilt, currentPan;

    enum ePastDataType{
      NONE,
      RAW_PITCH_YAW,
      POINT
    };
    ePastDataType pastDataType;
};

#include "pr2_head_manager/pr2_head_manager.h"


std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> Pr2HeadManager::stateFromMsg(const pr2_head_manager_msgs::StateMachineRegister::Request &msg)
{
    std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> states;

    for(auto x : msg.state_machine.states_Point){
        auto wrap = states[x.header.id] = std::make_shared<resource_management::MessageWrapper<geometry_msgs::PointStamped>>(x.data);
        wrap->setPriority(static_cast<resource_management::importance_priority_t>(msg.header.priority.value));
    }

    for(auto x : msg.state_machine.states_PitchYaw){
        auto wrap = states[x.header.id] = std::make_shared<resource_management::MessageWrapper<pr2_head_manager_msgs::RawPitchYaw>>(x.data);
        wrap->setPriority(static_cast<resource_management::importance_priority_t>(msg.header.priority.value));
    }

    for(auto x : msg.state_machine.states_PrioritizedPitch){
        auto wrap = states[x.header.id] = std::make_shared<resource_management::MessageWrapper<pr2_head_manager_msgs::Pitch>>(x.data);
        wrap->setPriority(static_cast<resource_management::importance_priority_t>(msg.header.priority.value));
    }

    for(auto x : msg.state_machine.states_PrioritizedYaw){
        auto wrap = states[x.header.id] = std::make_shared<resource_management::MessageWrapper<pr2_head_manager_msgs::Yaw>>(x.data);
        wrap->setPriority(static_cast<resource_management::importance_priority_t>(msg.header.priority.value));
    }

    return states;
}

std::vector<std::tuple<std::string,std::string,resource_management_msgs::EndCondition>>
Pr2HeadManager::transitionFromMsg(const pr2_head_manager_msgs::StateMachine &msg)
{
    std::vector<std::tuple<std::string,std::string,resource_management_msgs::EndCondition>> transitions;

    for(auto x : msg.states_Point){
        for(auto t : x.header.transitions){
            transitions.push_back(
                        std::make_tuple<std::string,std::string,resource_management_msgs::EndCondition>(
                            std::string(x.header.id),
                            std::string(t.next_state),
                            resource_management_msgs::EndCondition(t.end_condition)));
        }
    }

    for(auto x : msg.states_PitchYaw){
        for(auto t : x.header.transitions){
            transitions.push_back(
                        std::make_tuple<std::string,std::string,resource_management_msgs::EndCondition>(
                            std::string(x.header.id),
                            std::string(t.next_state),
                            resource_management_msgs::EndCondition(t.end_condition)));
        }
    }

    for(auto x : msg.states_PrioritizedPitch){
        for(auto t : x.header.transitions){
            transitions.push_back(
                        std::make_tuple<std::string,std::string,resource_management_msgs::EndCondition>(
                            std::string(x.header.id),
                            std::string(t.next_state),
                            resource_management_msgs::EndCondition(t.end_condition)));
        }
    }

    for(auto x : msg.states_PrioritizedYaw){
        for(auto t : x.header.transitions){
            transitions.push_back(
                        std::make_tuple<std::string,std::string,resource_management_msgs::EndCondition>(
                            std::string(x.header.id),
                            std::string(t.next_state),
                            resource_management_msgs::EndCondition(t.end_condition)));
        }
    }
    return transitions;
}

pr2_head_manager_msgs::StateMachineRegister::Response Pr2HeadManager::generateResponseMsg(uint32_t id)
{
  pr2_head_manager_msgs::StateMachineRegister::Response res;
  res.id = id;
  return res;
}

void Pr2HeadManager::publishPointMsg(geometry_msgs::PointStamped msg, bool is_new)
{
  if (!is_new){
        return;
    }

    // TODO: Make IK to check if the reference frame of the point has moved

    tf::Point target_in_root;
    geometry_msgs::TransformStamped torso2frame;
    try
    {
        std::string error_msg;
        msg.header.stamp = ros::Time(0);
        torso2frame = tfBuffer.lookupTransform("torso_lift_link", msg.header.frame_id, ros::Time(0));
    }
    catch(const tf::TransformException &ex)
    {
        ROS_ERROR("Transform failure: %s", ex.what());
        ROS_ERROR("Timestamp %f", msg.header.stamp.toSec());
        return;
    }
    geometry_msgs::PointStamped target_in_root_msg;
    tf2::doTransform(msg, target_in_root_msg, torso2frame);
    tf::pointMsgToTF(target_in_root_msg.point, target_in_root);
    target_in_root.setZ(target_in_root.getZ() - 0.3);
    target_in_root -= {-0.01707, 0, 0.38145};
    double pitch = -asin(target_in_root.getZ() / target_in_root.length());
    double yaw = atan2(target_in_root.getY(), target_in_root.getX());

    pr2_controllers_msgs::JointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("head_pan_joint");
    goal.trajectory.joint_names.push_back("head_tilt_joint");
    goal.trajectory.points.resize(1);

    // Positions
    goal.trajectory.points[0].positions.resize(2);
    goal.trajectory.points[0].positions[0] = yaw;
    goal.trajectory.points[0].positions[1] = pitch;
    // Velocities
    goal.trajectory.points[0].velocities.resize(2);
    goal.trajectory.points[0].velocities[0] = 0.0;
    goal.trajectory.points[0].velocities[1] = 0.0;

    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[0].time_from_start = ros::Duration(1.0);

    pastDataType = ePastDataType::POINT;

    ROS_INFO_STREAM("New point : " << target_in_root.x() << "," << target_in_root.y() << "," << target_in_root.z() << "\tPitch: " << pitch <<"\tYaw: "<<yaw);

    point_head_client->sendGoal(goal);
}

void Pr2HeadManager::publishPitchYawMsg(pr2_head_manager_msgs::RawPitchYaw msg, bool is_new)
{
  if (!is_new) {
        return;
    }

    pr2_controllers_msgs::JointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("head_pan_joint");
    goal.trajectory.joint_names.push_back("head_tilt_joint");
    goal.trajectory.points.resize(1);

    // Positions
    goal.trajectory.points[0].positions.resize(2);
    goal.trajectory.points[0].positions[0] = msg.yaw;
    goal.trajectory.points[0].positions[1] = msg.pitch;
    // Velocities
    goal.trajectory.points[0].velocities.resize(2);
    goal.trajectory.points[0].velocities[0] = 0.0;
    goal.trajectory.points[0].velocities[1] = 0.0;
    // Accelerations
    goal.trajectory.points[0].accelerations.resize(2);
    goal.trajectory.points[0].accelerations[0] = 0.0;
    goal.trajectory.points[0].accelerations[1] = 0.0;

    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[0].time_from_start = ros::Duration(1.0);

    pastDataType = ePastDataType::RAW_PITCH_YAW;

    point_head_client->sendGoal(goal);
}

void Pr2HeadManager::publishPrioritizedPitchMsg(pr2_head_manager_msgs::Pitch msg, bool is_new)
{
  if (!is_new) {
    return;
  }

  pr2_controllers_msgs::JointTrajectoryGoal goal;
  goal.trajectory.joint_names.push_back("head_pan_joint");
  goal.trajectory.joint_names.push_back("head_tilt_joint");
  goal.trajectory.points.resize(1);

  // Positions
  goal.trajectory.points[0].positions.resize(2);
  goal.trajectory.points[0].positions[0] = currentPan;
  goal.trajectory.points[0].positions[1] = msg.pitch;
  // Velocities
  goal.trajectory.points[0].velocities.resize(2);
  goal.trajectory.points[0].velocities[0] = 0.0;
  goal.trajectory.points[0].velocities[1] = 0.0;
  // Accelerations
  goal.trajectory.points[0].accelerations.resize(2);
  goal.trajectory.points[0].accelerations[0] = 0.0;
  goal.trajectory.points[0].accelerations[1] = 0.0;

  // To be reached 1 second after starting along the trajectory
  goal.trajectory.points[0].time_from_start = ros::Duration(1.0);

  pastDataType = ePastDataType::RAW_PITCH_YAW;

  point_head_client->sendGoal(goal);
}

void Pr2HeadManager::publishPrioritizedYawMsg(pr2_head_manager_msgs::Yaw msg, bool is_new)
{
  if (!is_new) {
    return;
  }

  pr2_controllers_msgs::JointTrajectoryGoal goal;
  goal.trajectory.joint_names.push_back("head_pan_joint");
  goal.trajectory.joint_names.push_back("head_tilt_joint");
  goal.trajectory.points.resize(1);

  // Positions
  goal.trajectory.points[0].positions.resize(2);
  goal.trajectory.points[0].positions[0] = msg.yaw;
  goal.trajectory.points[0].positions[1] = currentTilt;
  // Velocities
  goal.trajectory.points[0].velocities.resize(2);
  goal.trajectory.points[0].velocities[0] = 0.0;
  goal.trajectory.points[0].velocities[1] = 0.0;
  // Accelerations
  goal.trajectory.points[0].accelerations.resize(2);
  goal.trajectory.points[0].accelerations[0] = 0.0;
  goal.trajectory.points[0].accelerations[1] = 0.0;

  // To be reached 1 second after starting along the trajectory
  goal.trajectory.points[0].time_from_start = ros::Duration(1.0);

  pastDataType = ePastDataType::RAW_PITCH_YAW;

  point_head_client->sendGoal(goal);
}

void Pr2HeadManager::checkGoalActive() {
  auto head_state = point_head_client->getState().state_;
    if (_is_command_running){
        if (head_state != actionlib::SimpleClientGoalState::ACTIVE){
            this->done();
            _is_command_running = false;
            std::cout << "done" << std::endl;
        }
    }else{
        if (head_state == actionlib::SimpleClientGoalState::ACTIVE){
          _is_command_running = true;
        }
    }
}
void Pr2HeadManager::onWatchDog(const ros::TimerEvent&) {
  checkGoalActive();
}
void Pr2HeadManager::onJointState(const sensor_msgs::JointStateConstPtr &msg) {
  bool tiltFound = false, panFound = false;
  for (size_t i=0; i < msg->name.size(); i++){
    if (msg->name[i] == "head_pan_joint"){
      currentPan = msg->position[i];
      panFound = true;
    }else if (msg->name[i] == "head_tilt_joint"){
      currentTilt = msg->position[i];
      tiltFound = true;
    }
    if (tiltFound && panFound){
      break;
    }
  }
}


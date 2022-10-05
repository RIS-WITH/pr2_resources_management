#include "pr2_head_manager/ArtificialLife.h"
#include "resource_management/message_storage/MessageWrapper.h"
#include "pr2_head_manager_msgs/RawPitchYaw.h"

namespace pr2_head_manager {

ArtificialLife::ArtificialLife(std::shared_ptr<resource_management::ReactiveBuffer> buffer) :
        resource_management::ArtificialLife(100 /* you can change the artficial life frame rate here*/, buffer),
        poisson_distribution(3000), normal_distribution(0.0, 0.2)
{
  // set an initial value in the artificial life buffer
  // if you do not do that that resource will not start in artificial life mode

  // Example:

  // 1 - Wrap your data with one of your types:
  // auto wrapped_Point_data = std::make_shared<resource_management::MessageWrapper<geometry_msgs::PointStamped>>(data);
  // auto wrapped_PitchYaw_data = std::make_shared<resource_management::MessageWrapper<pr2_head_manager_msgs::RawPitchYaw>>(data);
  // auto wrapped_PrioritizedPitch_data = std::make_shared<resource_management::MessageWrapper<pr2_head_manager_msgs::Pitch>>(data);
  // auto wrapped_PrioritizedYaw_data = std::make_shared<resource_management::MessageWrapper<pr2_head_manager_msgs::Yaw>>(data);
  //
  // 2 - Set the useless priority to your wrapped data:
  // wrapped_Point_data->setPriority(resource_management::useless);
  // wrapped_PitchYaw_data->setPriority(resource_management::useless);
  // wrapped_PrioritizedPitch_data->setPriority(resource_management::useless);
  // wrapped_PrioritizedYaw_data->setPriority(resource_management::useless);
  //
  // 3 - Insert your wrapped data into the rtificial life buffer:
  // _buffer->setData(wrapped_Point_data);
  // _buffer->setData(wrapped_PitchYaw_data);
  // _buffer->setData(wrapped_PrioritizedPitch_data);
  // _buffer->setData(wrapped_PrioritizedYaw_data);
    pr2_head_manager_msgs::RawPitchYaw data;
    data.pitch = 0.0;
    data.yaw = 0.0;
    auto wrapped_PitchYaw_data = std::make_shared<resource_management::MessageWrapper<pr2_head_manager_msgs::RawPitchYaw>>(data);
    wrapped_PitchYaw_data->setPriority(resource_management::low);
    _buffer->setData(wrapped_PitchYaw_data);

    pitch = data.pitch;
    yaw = data.yaw;
}

void ArtificialLife::init()
{
  // Put our own initialisation function here
  // It will be called each time a new artificial life cycle begins

  // Set an initial value in the artificial life buffer
  // if you do not do that, at each new cycle, the resource
  // will start with the previous artificial life value

  // Example:

  // 1 - Wrap your data with one of your types:
  // auto wrapped_Point_data = std::make_shared<resource_management::MessageWrapper<geometry_msgs::PointStamped>>(data);
  // auto wrapped_PitchYaw_data = std::make_shared<resource_management::MessageWrapper<pr2_head_manager_msgs::RawPitchYaw>>(data);
  // auto wrapped_PrioritizedPitch_data = std::make_shared<resource_management::MessageWrapper<pr2_head_manager_msgs::Pitch>>(data);
  // auto wrapped_PrioritizedYaw_data = std::make_shared<resource_management::MessageWrapper<pr2_head_manager_msgs::Yaw>>(data);
  //
  // 2 - Set the useless priority to your wrapped data:
  // wrapped_Point_data->setPriority(resource_management::useless);
  // wrapped_PitchYaw_data->setPriority(resource_management::useless);
  // wrapped_PrioritizedPitch_data->setPriority(resource_management::useless);
  // wrapped_PrioritizedYaw_data->setPriority(resource_management::useless);
  //
  // 3 - Insert your wrapped data into the rtificial life buffer:
  // _buffer->setData(wrapped_Point_data);
  // _buffer->setData(wrapped_PitchYaw_data);
  // _buffer->setData(wrapped_PrioritizedPitch_data);
  // _buffer->setData(wrapped_PrioritizedYaw_data);
    pr2_head_manager_msgs::RawPitchYaw data;
    data.pitch = 0.0;
    data.yaw = 0.0;
    auto wrapped_PitchYaw_data = std::make_shared<resource_management::MessageWrapper<pr2_head_manager_msgs::RawPitchYaw>>(data);
    wrapped_PitchYaw_data->setPriority(resource_management::low);
    _buffer->setData(wrapped_PitchYaw_data);
    last_moved_time = std::chrono::system_clock::now();
    next_duration = poisson_distribution(generator);
}

void ArtificialLife::inLoop()
{
  // This function will be called at the specified frame rate
  // will the artificial life cycle is running

  // DO NOT CREATE YOUR OWN LOOP HERE

  // creat a new data and feed it to the artficial life buffer
  // Example:

  // 1 - Wrap your data with one of your types:
  // auto wrapped_Point_data = std::make_shared<resource_management::MessageWrapper<geometry_msgs::PointStamped>>(data);
  // auto wrapped_PitchYaw_data = std::make_shared<resource_management::MessageWrapper<pr2_head_manager_msgs::RawPitchYaw>>(data);
  // auto wrapped_PrioritizedPitch_data = std::make_shared<resource_management::MessageWrapper<pr2_head_manager_msgs::Pitch>>(data);
  // auto wrapped_PrioritizedYaw_data = std::make_shared<resource_management::MessageWrapper<pr2_head_manager_msgs::Yaw>>(data);
  //
  // 2 - Set the useless priority to your wrapped data:
  // wrapped_Point_data->setPriority(resource_management::useless);
  // wrapped_PitchYaw_data->setPriority(resource_management::useless);
  // wrapped_PrioritizedPitch_data->setPriority(resource_management::useless);
  // wrapped_PrioritizedYaw_data->setPriority(resource_management::useless);
  //
  // 3 - Insert your wrapped data into the rtificial life buffer:
  // _buffer->setData(wrapped_Point_data);
  // _buffer->setData(wrapped_PitchYaw_data);
  // _buffer->setData(wrapped_PrioritizedPitch_data);
  // _buffer->setData(wrapped_PrioritizedYaw_data);
    auto now = std::chrono::system_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_moved_time).count() >= next_duration)
    {
        pr2_head_manager_msgs::RawPitchYaw data;
        do {
          data.pitch = getNearValue(pitch, 0.15);
        } while(data.pitch < -0.1);
        
        data.yaw = getNearValue(yaw, 0.15);
        auto wrapped_PitchYaw_data = std::make_shared<resource_management::MessageWrapper<pr2_head_manager_msgs::RawPitchYaw>>(
                data);
        wrapped_PitchYaw_data->setPriority(resource_management::low);
        _buffer->setData(wrapped_PitchYaw_data);
        last_moved_time = now;
        next_duration = poisson_distribution(generator);

        pitch = data.pitch;
        yaw = data.yaw;
    }
}

double ArtificialLife::getNearValue(double previous_value, double max_delta)
{
  double value = 0;
  do {
    value = normal_distribution(generator);
  } while(std::abs(value - previous_value) > max_delta);
  return value;
}

} // namespace pr2_head_manager

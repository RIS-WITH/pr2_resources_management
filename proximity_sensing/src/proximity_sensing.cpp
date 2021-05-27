//
// Created by gbuisan on 29/03/19.
//
#include <ros/ros.h>
#include <pr2_head_manager_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>


ros::Publisher point_head_pub;
std::vector<float> old_scan;
ros::Time last_move_time;

constexpr const float distance_threshold = 0.2;
constexpr const float max_distance = 2.0; // Do not look at things farther than
constexpr const float look_altitude = 1.2; // From laser frame
constexpr const float time_not_moving_uninteresting = 2.0; // sec

void onNewLidarScan(const sensor_msgs::LaserScan &msg){
    if (!old_scan.empty()){
        size_t index_max = 0;
        float max_motion = 0;
        for (size_t i=0; i <= msg.ranges.size(); i++){
            float motion = std::abs(msg.ranges[i] - old_scan[i]);
            if (i > 0 && i < msg.ranges.size() - 1 && abs(msg.ranges[i] - msg.ranges[i-1]) < 0.02
                && abs(msg.ranges[i] - msg.ranges[i+1]) < 0.02 && msg.ranges[i] > msg.range_min
                && msg.ranges[i] < max_distance && motion > max_motion && msg.ranges[i] < max_distance) {
                max_motion = motion;
                index_max = i;
            }
        }
        if (max_motion >= distance_threshold){
            std::cout << max_motion << std::endl;
            pr2_head_manager_msgs::Point pt;
            float angle = msg.angle_min + index_max * msg.angle_increment;
            pt.data.header.frame_id = msg.header.frame_id;
            pt.data.point.x = msg.ranges[index_max] * std::cos(angle);
            pt.data.point.y = msg.ranges[index_max] * std::sin(angle);
            pt.data.point.z = look_altitude;
            pt.priority.value = resource_management_msgs::MessagePriority::LOW;
            point_head_pub.publish(pt);
            last_move_time = ros::Time::now();
        }else if (ros::Time::now() - last_move_time >= ros::Duration(time_not_moving_uninteresting)){
            pr2_head_manager_msgs::Point pt;
            pt.priority.value = resource_management_msgs::MessagePriority::VOID;
            point_head_pub.publish(pt);
            last_move_time = ros::Time::now(); // To avoid over publishing
        }
    }
    old_scan = msg.ranges;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "proximity_sensing");
    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

    point_head_pub = nh->advertise<pr2_head_manager_msgs::Point>("/pr2_head_manager/proximity_pr2_head_manager_msgs_Point", 10, true);
    ros::Subscriber lidar_sub = nh->subscribe("/base_scan", 10, onNewLidarScan);

    ros::spin();
    return 0;
}

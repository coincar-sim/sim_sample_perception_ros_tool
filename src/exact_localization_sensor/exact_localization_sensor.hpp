#pragma once

#include <exception>
#include <string>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>

#include <automated_driving_msgs/BasicSafetyMessage.h>
#include <automated_driving_msgs/MotionState.h>
#include <automated_driving_msgs/ObjectState.h>
#include <automated_driving_msgs/ObjectStateArray.h>
#include <simulation_utils/util_perception.hpp>

#include "sim_sample_perception_ros_tool/ExactLocalizationSensorParameters.h"

namespace sim_sample_perception_ros_tool {

class ExactLocalizationSensor {
public:
    ExactLocalizationSensor(ros::NodeHandle, ros::NodeHandle);

private:
    ros::Publisher percEgoMotionPub_;
    ros::Publisher egoBSMPub_;
    ros::Subscriber objectsStateSub_;
    ros::Timer timerBSM_;

    automated_driving_msgs::MotionState latestMotionState_;

    ExactLocalizationSensorParameters params_;

    dynamic_reconfigure::Server<ExactLocalizationSensorConfig>
        reconfigSrv_; // Dynamic reconfiguration service

    void subCallback(const automated_driving_msgs::ObjectStateArray::ConstPtr& msg);
    void timerCallbackBSM(const ros::TimerEvent&);

    void reconfigureRequest(ExactLocalizationSensorConfig&, uint32_t);
};

} // namespace sim_sample_perception_ros_tool

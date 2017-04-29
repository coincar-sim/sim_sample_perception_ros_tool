#pragma once

#include <exception>
#include <string>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>

#include <automated_driving_msgs/ObjectState.h>
#include <automated_driving_msgs/ObjectStateArray.h>
#include <simulation_utils/util_perception.hpp>

#include "sim_sample_perception_ros_tool/AbstractObjectSensorParameters.h"

namespace sim_sample_perception_ros_tool {

class AbstractObjectSensor {
public:
    AbstractObjectSensor(ros::NodeHandle, ros::NodeHandle);

private:
    ros::Publisher percObjectsPub_;
    ros::Subscriber objectsStateSub_;

    AbstractObjectSensorParameters params_;

    automated_driving_msgs::ObjectStateArray latestPerceptedObjects_;

    dynamic_reconfigure::Server<AbstractObjectSensorConfig>
        reconfigSrv_; // Dynamic reconfiguration service

    void subCallback(const automated_driving_msgs::ObjectStateArray::ConstPtr& msg);

    void reconfigureRequest(AbstractObjectSensorConfig&, uint32_t);
};

} // namespace sim_sample_perception_ros_tool

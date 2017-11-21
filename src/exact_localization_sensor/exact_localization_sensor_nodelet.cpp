#include "exact_localization_sensor.hpp"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace sim_sample_perception_ros_tool {

class ExactLocalizationSensorNodelet : public nodelet::Nodelet {

    virtual void onInit();
    boost::shared_ptr<ExactLocalizationSensor> m_;
};

void ExactLocalizationSensorNodelet::onInit() {
    m_.reset(new ExactLocalizationSensor(getNodeHandle(), getPrivateNodeHandle()));
}

} // namespace sim_sample_perception_ros_tool

PLUGINLIB_DECLARE_CLASS(sim_sample_perception_ros_tool,
                        ExactLocalizationSensorNodelet,
                        sim_sample_perception_ros_tool::ExactLocalizationSensorNodelet,
                        nodelet::Nodelet);

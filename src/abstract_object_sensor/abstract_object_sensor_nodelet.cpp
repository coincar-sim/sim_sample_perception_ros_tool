#include "abstract_object_sensor.hpp"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace sim_sample_perception_ros_tool {

class AbstractObjectSensorNodelet : public nodelet::Nodelet {

    virtual void onInit();
    boost::shared_ptr<AbstractObjectSensor> m_;
};

void AbstractObjectSensorNodelet::onInit() {
    m_.reset(new AbstractObjectSensor(getNodeHandle(), getPrivateNodeHandle()));
}

} // namespace sim_sample_perception_ros_tool

PLUGINLIB_DECLARE_CLASS(sim_sample_perception_ros_tool,
                        AbstractObjectSensorNodelet,
                        sim_sample_perception_ros_tool::AbstractObjectSensorNodelet,
                        nodelet::Nodelet);

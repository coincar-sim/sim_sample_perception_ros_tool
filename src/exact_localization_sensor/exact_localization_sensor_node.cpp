#include "exact_localization_sensor.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "exact_localization_sensor_node");

    sim_sample_perception_ros_tool::ExactLocalizationSensor exact_localization_sensor(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return 0;
}

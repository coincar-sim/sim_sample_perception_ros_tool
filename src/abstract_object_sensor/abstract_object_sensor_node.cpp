#include "abstract_object_sensor.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "abstract_object_sensor_node");

    sim_sample_perception_ros_tool::AbstractObjectSensor abstract_object_sensor(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return 0;
}

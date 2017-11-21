#include "abstract_object_sensor.hpp"

namespace sim_sample_perception_ros_tool {

AbstractObjectSensor::AbstractObjectSensor(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
        : reconfigSrv_{private_node_handle}, params_{private_node_handle} {

    /**
     * Initialization
     */
    params_.fromParamServer();

    /**
     * Set up dynamic reconfiguration
     */
    reconfigSrv_.setCallback(boost::bind(&AbstractObjectSensor::reconfigureRequest, this, _1, _2));

    /**
     * Publishers & subscriber
     */
    // publisher for perceived objects (all perceived objects / objects from ground truth except
    // itself)
    percObjectsPub_ = node_handle.advertise<automated_driving_msgs::ObjectStateArray>(
        params_.perceivedObjects_sensor_out_topic, params_.msg_queue_size);

    // Instantiate subscriber last, to assure all objects are initialized when first message is
    // received.
    objectsStateSub_ = node_handle.subscribe(params_.objects_ground_truth_topic_with_ns,
                                             params_.msg_queue_size,
                                             &AbstractObjectSensor::subCallback,
                                             this,
                                             ros::TransportHints().tcpNoDelay());
}

/**
 *This Callback receives ground truth ObjectStateArray and publishes
 *      perceived Objects as ObjectStateArray
 */
void AbstractObjectSensor::subCallback(const automated_driving_msgs::ObjectStateArray::ConstPtr& msg) {

    automated_driving_msgs::ObjectStateArray perceivedObjects =
        util_perception::RemoveObjectFromObjectStateArray(*msg, params_.vehicle_id);

    for (size_t i = 0; i < perceivedObjects.objects.size(); i++) {

        const int32_t objectId = perceivedObjects.objects[i].object_id;
        bool foundAndUnique;

        automated_driving_msgs::ObjectState& objectState = perceivedObjects.objects[i];
        automated_driving_msgs::ObjectState lastObjectState =
            util_perception::ObjectStateFromObjectStateArray(latestPerceivedObjects_, objectId, foundAndUnique);
        // automated_driving_msgs::ObjectStatePtr lastObjStatePtr =
        // util_perception::ObjectStatePtrFromObjectStateArray(latestPerceivedObjects_, objectId,
        // foundAndUnique);

        if (!foundAndUnique) {
            continue;
        } else {
            ros::Duration deltaTime = objectState.header.stamp - lastObjectState.motion_state.header.stamp;

            if (!util_perception::poseValid(objectState.motion_state)) {
                ROS_DEBUG("Received MotionState.pose is marked as unreliable. Forwarding it anyway.");
            }

            if (!util_perception::twistValid(objectState.motion_state)) {
                if (util_perception::poseValid(lastObjectState.motion_state)) {
                    util_perception::diffPoseToTwist(lastObjectState.motion_state.pose.pose,
                                                     objectState.motion_state.pose.pose,
                                                     deltaTime,
                                                     objectState.motion_state.twist);
                } else {
                    ROS_DEBUG("Could not calculate twist as latest pose not valid.");
                }
            }

            if (!util_perception::accelValid(objectState.motion_state)) {
                if (util_perception::twistValid(lastObjectState.motion_state) &&
                    util_perception::twistValid(objectState.motion_state)) {
                    util_perception::diffTwistToAccel(lastObjectState.motion_state.twist.twist,
                                                      objectState.motion_state.twist.twist,
                                                      deltaTime,
                                                      objectState.motion_state.accel);
                } else {
                    ROS_DEBUG("Could not calculate accel as latest twists not valid.");
                }
            }
        }
    }

    // publish perceived objects
    percObjectsPub_.publish(perceivedObjects);

    // set new newPerceivedObjects state as latest perceived MotionState state
    latestPerceivedObjects_ = perceivedObjects;
}


/**
 * This callback is called whenever a change was made in the dynamic_reconfigure window
 */
void AbstractObjectSensor::reconfigureRequest(AbstractObjectSensorConfig& config, uint32_t level) {
    params_.fromConfig(config);
}

} // namespace sim_sample_perception_ros_tool

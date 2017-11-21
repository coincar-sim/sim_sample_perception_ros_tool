#include "exact_localization_sensor.hpp"

namespace sim_sample_perception_ros_tool {

ExactLocalizationSensor::ExactLocalizationSensor(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
        : reconfigSrv_{private_node_handle}, params_{private_node_handle} {

    /**
     * Initialization
     */
    params_.fromParamServer();

    /**
     * Set up dynamic reconfiguration
     */
    reconfigSrv_.setCallback(boost::bind(&ExactLocalizationSensor::reconfigureRequest, this, _1, _2));

    // as with initialization no information about the motion state is available set diagonals of
    // covariance matrizes to -1
    // This ensures no calculations for velocity and acceleration are made before reliable data is
    // received.
    latestMotionState_.pose.covariance = util_perception::invalidCov;
    latestMotionState_.twist.covariance = util_perception::invalidCov;
    latestMotionState_.accel.covariance = util_perception::invalidCov;

    /**
     * Publishers & subscriber
     */
    // publisher for perceived Motion (egoMotion)
    percEgoMotionPub_ = node_handle.advertise<automated_driving_msgs::MotionState>(params_.egoMotion_sensor_out_topic,
                                                                                   params_.msg_queue_size);

    // publisher for BasicSafetyMessage (egoBSM)
    egoBSMPub_ = node_handle.advertise<automated_driving_msgs::BasicSafetyMessage>(params_.BSM_out_topic,
                                                                                   params_.msg_queue_size);

    // Timer for BasicSafetyMessage (egoBSM)
    timerBSM_ = node_handle.createTimer(
        ros::Duration(1.0 / params_.BSM_fequency), &ExactLocalizationSensor::timerCallbackBSM, this);

    // Instantiate subscriber last, to assure all objects are initialized when first message is
    // received.
    objectsStateSub_ = node_handle.subscribe(params_.objects_ground_truth_topic_with_ns,
                                             params_.msg_queue_size,
                                             &ExactLocalizationSensor::subCallback,
                                             this,
                                             ros::TransportHints().tcpNoDelay());
}

/**
 *This Callback receives ObjectStateArrays and updates and publishes
 *			the MotionState of the own vehicleID
 *			If necessary the Velocity and Acceleration is calculated from the received and latest saved
 *data.
 */
void ExactLocalizationSensor::subCallback(const automated_driving_msgs::ObjectStateArray::ConstPtr& msg) {

    bool foundAndUnique;
    automated_driving_msgs::ObjectState egoObjectState =
        util_perception::ObjectStateFromObjectStateArray(*msg, params_.vehicle_id, foundAndUnique);

    if (!foundAndUnique) {
        return;
    } else {
        ros::Duration deltaTime = egoObjectState.header.stamp - latestMotionState_.header.stamp;
        if (deltaTime.toSec() < 0.0001) {
            ROS_WARN("DeltaTime too small for calculation of new state, deltaTime=%f", deltaTime.toSec());
            return;
        }

        util_perception::incorporatePrecedingDataToMotionstate(latestMotionState_, egoObjectState.motion_state);
    }

    // publish new motion state
    percEgoMotionPub_.publish(egoObjectState.motion_state);

    // set new EgoObjectState state as latest EgoObjectState state
    latestMotionState_ = egoObjectState.motion_state;
}

/**
 *This Callback publishes every params_.BSM_fequency hertz a Basic Safety Message
 */
void ExactLocalizationSensor::timerCallbackBSM(const ros::TimerEvent&) {
    // create BasicSafetyMessage (BSM) message
    automated_driving_msgs::BasicSafetyMessage egoBSM;
    // TODO: Create BSM according to specification and with all needed data. Until now only the
    // motionState is packed into the BSM.
    if (&latestMotionState_) {
        egoBSM.header.stamp = ros::Time::now();
        egoBSM.header.frame_id = latestMotionState_.header.frame_id;
        egoBSM.object_state.motion_state = latestMotionState_;
        // publish BSM
        egoBSMPub_.publish(egoBSM);
    }
}

/**
 * This callback is called whenever a change was made in the dynamic_reconfigure window
 */
void ExactLocalizationSensor::reconfigureRequest(ExactLocalizationSensorConfig& config, uint32_t level) {
    params_.fromConfig(config);
}

} // namespace sim_sample_perception_ros_tool

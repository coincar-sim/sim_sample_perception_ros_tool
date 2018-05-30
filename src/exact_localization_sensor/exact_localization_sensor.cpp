/*
 * Copyright (c) 2017
 * FZI Forschungszentrum Informatik, Karlsruhe, Germany (www.fzi.de)
 * KIT, Institute of Measurement and Control, Karlsruhe, Germany (www.mrt.kit.edu)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <util_automated_driving_msgs/util_automated_driving_msgs.hpp>
#include <util_geometry_msgs/util_geometry_msgs.hpp>

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
    latestMotionState_.pose.covariance = util_geometry_msgs::checks::covarianceUnkownValues;
    latestMotionState_.twist.covariance = util_geometry_msgs::checks::covarianceUnkownValues;
    latestMotionState_.accel.covariance = util_geometry_msgs::checks::covarianceUnkownValues;

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
        util_automated_driving_msgs::conversions::objectStateFromObjectStateArray(
            msg, params_.vehicle_id, foundAndUnique);

    if (!foundAndUnique) {
        return;
    } else {
        ros::Duration deltaTime = egoObjectState.header.stamp - latestMotionState_.header.stamp;
        if (deltaTime.toSec() < 0.0001) {
            ROS_WARN("DeltaTime too small for calculation of new state, deltaTime=%f", deltaTime.toSec());
            return;
        }

        util_automated_driving_msgs::computations::incorporatePrecedingDataToMotionstate(latestMotionState_,
                                                                                         egoObjectState.motion_state);
    }

    // set new EgoObjectState state as latest EgoObjectState state
    latestMotionState_ = egoObjectState.motion_state;

    // add uncertainty to ego-motionstate here, if desired

    // publish new motion state
    percEgoMotionPub_.publish(egoObjectState.motion_state);
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

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
        util_automated_driving_msgs::conversions::removeObjectFromObjectStateArray(*msg, params_.vehicle_id);

    for (size_t i = 0; i < perceivedObjects.objects.size(); i++) {

        const int32_t objectId = perceivedObjects.objects[i].object_id;
        bool foundAndUnique;

        automated_driving_msgs::ObjectState& objectState = perceivedObjects.objects[i];
        automated_driving_msgs::ObjectState lastObjectState =
            util_automated_driving_msgs::conversions::objectStateFromObjectStateArray(latestPerceivedObjects_, objectId, foundAndUnique);

        if (!foundAndUnique) {
            continue;
        } else {

            util_automated_driving_msgs::computations::incorporatePrecedingDataToMotionstate(lastObjectState.motion_state,
                                                                   objectState.motion_state);
        }
    }

    // set new newPerceivedObjects state as latest perceived MotionState state
    latestPerceivedObjects_ = perceivedObjects;

    // add uncertainty to other objects' motionstates here, if desired

    // publish perceived objects
    percObjectsPub_.publish(perceivedObjects);


}

/**
 * This callback is called whenever a change was made in the dynamic_reconfigure window
 */
void AbstractObjectSensor::reconfigureRequest(AbstractObjectSensorConfig& config, uint32_t level) {
    params_.fromConfig(config);
}

} // namespace sim_sample_perception_ros_tool

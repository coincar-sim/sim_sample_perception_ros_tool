# sim_sample_perception_ros_tool
Sample perception module for a vehicle in the simulation framework.

Consists of two sensors:

#### exact_localization_sensor
* publishes the exact vehicle's MotionState (exact as received from the localization management)
* publishes regularly the vehicle's BasicSafetyMessage
* pose is received from the localization management, velocity and acceleration are calculated using difference quotient if possible

#### abstract_object_sensor
* publishes all other vehicle's ObjectStates (exact as received from the localization management)
* the objects poses are received from the localization management, velocity and acceleration are calculated using difference quotient if possible

## Installation
* this package is part of the simulation framework
* see simulation_management_ros_tool for installation and more details

## Usage
* started within the a vehicle launchfile of the simulation_initialization_ros_tool

#### Parameters
* parameters that need to be passed to the launchfile `sample_perception.launch`:
  * **vehicle_id**: Id of the vehicle, needs to be unique within the framework
  * **vehicle_ns**: Namespace of the vehicle, needs to be unique within the framework
  * **objects_ground_truth_topic_with_ns**: Topic under which the ground truth states of the objects are received
  * **perc_pred_obj_topic**: Topic for the perceived objects
  * **perc_egomotion_topic**: Topic for the perceived ego motion state
  * **internal_communication_subns**: Subnamespace for vehicle-internal communication
  * **BSM_topic**: Topic for the basic safety messages

## Contribution

* fork this repo
* use your own algorithms for modeling sensor characteristics and fusing sensor information
* ensure that
  * `/$(arg vehicle_ns)/$(arg perc_egomotion_topic)` is published
  * `/$(arg vehicle_ns)/$(arg perc_pred_obj_topic)` is published
  * all internal ROS communication stays within the perception namespace

## License
Contact the maintainer.

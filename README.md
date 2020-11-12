# Gazebo Model Attachement Plugin
[![pipeline status](https://git.web.boeing.com/robotics/ros/gazebo_model_attachment_plugin/badges/master/pipeline.svg)](https://git.web.boeing.com/robotics/ros/gazebo_model_attachment_plugin/commits/master)
[![coverage](https://git.web.boeing.com/robotics/ros/gazebo_model_attachment_plugin/badges/master/coverage.svg)](https://git.web.boeing.com/robotics/ros/gazebo_model_attachment_plugin/commits/master)

## Motivation
 Once a model has been spawned within Gazebo, it can often be necessary to add or remove _Links_ at runtime without destroying the model. This can be necessary for simulating actions such as end effector swap, item transport (loading and uploading), or pick and place operations. 
This package enables such simulations by allowing _Models_ to be attached to each other.

## Goals
- The plugin shall provide a way for models within Gazebo to be ridgidly attached to each other such that one will follow the others movements.

## Requirements
- This shall advertise an attachment and a detachment service that can be called with a ROS service call.
- These service calls allow _Models_ within Gazebo to be joined rigidly at the _Link_ level at runtime. This is essential for simulating processes such as End Effector change / removal or consumable swap.

## Definitions

| Definition | Description                                                                                                              |
| ---------- | ------------------------------------------------------------------------------------------------------------------------ |
| Model       | An instanced SDF in Gazebo. _Models_ consist of _Links_ connected by _Joints_.                                                                                                               |
| Link        | A sub component of a _Model_. This plugin creates static 'attachment' _Joints_ between _Links_. _Links_ can contain geometry elements or attached plugins.                                                                                                               |
| Joint       | A connection between _Links_ in Gazebo. 

## Design

### Assumptions

- The models must have been spawned and contain the specified link names.
- Physics should be disabled using _gazebo_no_physics_plugin_. 

### Limitations
- The plugin will not teleport the child model to make the pose of  _model_name_1/link_name_1_ match _model_name_2/link_name_2_. The attachment will instead occur with the pose difference at the time of the service call being maintained. 
    - If the user wishes to have zero pose difference a call to the _gazebo/set_link_state_ service can be made. 

## Requirements Evaluation 

| Requirement | Met? | Comments |
| ------------| ------- | ---------- |
| Attach a model to another model at the link level. | Yes | None |
| Destroy a previously created attachment. | Yes | None |


## Related Components
| Name                | Link                                                                       |
| ------------------- | -------------------------------------------------------------------------- |
| ee_manager | https://git.web.boeing.com/robotics/ros/ee_manager                |
| gazebo_ros_pkgs | https://git.web.boeing.com/robotics/ros-thirdparty/gazebo_ros_pkgs |
| ee_manager              | https://git.web.boeing.com/robotics/ros/ee_manager |
| gazebo_no_physics_plugin | https://git.web.boeing.com/robotics/ros/gazebo_no_physics_plugin |
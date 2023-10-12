# Gazebo Model Attachement Plugin

[![pipeline status](https://git.web.boeing.com/robotics/ros/gazebo_model_attachment_plugin/badges/master/pipeline.svg)](https://git.web.boeing.com/robotics/ros/gazebo_model_attachment_plugin/commits/master)
[![coverage](https://git.web.boeing.com/robotics/ros/gazebo_model_attachment_plugin/badges/master/coverage.svg)](https://git.web.boeing.com/robotics/ros/gazebo_model_attachment_plugin/commits/master)

## Motivation

 Once a model has been spawned within Gazebo, it can often be necessary to add or remove _Links_ at runtime without destroying the
 model. This can be necessary for simulating actions such as end effector swap, item transport (loading and uploading), or pick and place operations.
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
| gazebo_no_physics_plugin | https://git.web.boeing.com/robotics/ros/gazebo_no_physics_plugin |

# Authors
The Boeing Company

     	Beau Colley-Allerton
     	Jason Cochrane


# License

Copyright 2023 The Boeing Company

Licensed under the Apache License, Version 2.0 (the "License") with the following modification;
you may not use this file except in compliance with the Apache License and the following modification to it:

(Appended as Section 10)

By accepting this software, recipient agrees that the representations, warranties, obligations, and liabilities of The Boeing Company set forth in this software, if any, are exclusive and in substitution for all other all other representations, warranties, obligations, and liabilities of The Boeing Company.
Recipient hereby waives, releases, and renounces all other rights, remedies, and claims (including tortious claims for loss of or damage to property) of recipient against The Boeing Company with respect to this software.
The Boeing Company hereby disclaims all implied warranties, including but not limited to, all implied warranties of merchantability, fitness, course of dealing, and usage of trade.
The Boeing Company shall have no liability, whether arising in contract (including warranty), tort (whether or not arising from the negligence of The Boeing Company), or otherwise, for any loss of use, revenue, or profit, or for any other unspecified direct, indirect, incidental, or consequential damages for anything delivered or otherwise provided by The Boeing Company under this software.

You may obtain a copy of the original, unmodified License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
echo 

# Contributing

Any contribution that you make to this repository will
be under the Modified Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0):

```
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
```

To contribute, issue a PR and @brta-jc (jason.cochrane@boeing.com).
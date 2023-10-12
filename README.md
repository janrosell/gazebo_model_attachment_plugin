---
BUILD_CMD: catkin_build
BUILD_TOOL: catkin
SERVICE_CALL: rosservice
DISTRO: humble
---

# Gazebo Model Attachment Plugin

## Overview

 Once a model has been spawned within Gazebo, it can often be necessary to add or remove _Links_ at runtime without destroying the model. This can be necessary for simulating actions such as end effector swap, item transport (loading and uploading), or pick and place operations. This package enables such simulations by allowing Models to be attached to each other.

## Installation

### Dependencies

TODO

### Installation from Packages

*PENDING APPROVAL ON ROS_DISTRO*

To install all packages from the this repository as Debian packages use

    sudo apt-get install ros-{{DISTRO}}-boeing-gazebo-model-attachment-plugin
    
Or better, use `rosdep`:

	sudo rosdep install --from-paths src

#### Building

To build from source, clone the latest version from this repository into your ros workspace and compile the package using {{BUILD_TOOL}}

	cd ros_ws/src
	git clone https://github.com/boeing/gazebo_model_attachment_plugin.git
	cd ../
	rosdep install --from-paths . --ignore-src
	{{BUILD_CMD}}

## Usage

Add your library path to the `GAZEBO_PLUGIN_PATH`

    export GAZEBO_PLUGIN_PATH=$HOME/ros_ws/build:$GAZEBO_PLUGIN_PATH

Add the plugin to your world file
    
    <sdf version='1.6'>
    <world name='default'>

     <plugin name="model_attachment" filename="libgazebo_model_attachment_plugin_lib.so"></plugin>
        
        <include>
          <uri>model://ground_plane</uri>
        </include>

        <include>
          <uri>model://sun</uri>
        </include>

        <plugin name="factory" filename="libfactory.so"/>
      </world>
    </sdf>

Run Gazebo

    gazebo ~/gazebo_plugin_tutorial/test.world

#### Services

* **`attach`** (boeing_gazebo_model_attachment_plugin/Attach.srv)

	Creates a joint between two links

		{{SERVICE_CMD}} call /gazebo_model_attachment_plugin/attach

  ##### Parameters

  * **`model_1`** (string)

	The name of the parent model.

  * **`link_1`** (string)
  The name of the link on the parent model.
  
  * **`model_2`** (string)

	The name of the child model.

  * **`link_2`** (string)
  The name of the link on the child model.
<br/>
* **`detach`** (boeing_gazebo_model_attachment_plugin/Detach.srv)
  removes a joint between two links

		{{SERVICE_CMD}} call /gazebo_model_attachment_plugin/detach
  ##### Parameters

  * **`link_1`** (string)
  The name of the link on the parent model.
  

  * **`link_2`** (string)
  The name of the link on the child model.
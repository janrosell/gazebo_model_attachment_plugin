# Gazebo Model Attachement Plugin
[![pipeline status](https://git.web.boeing.com/robotics/ros/gazebo_model_attachment_plugin/badges/master/pipeline.svg)](https://git.web.boeing.com/robotics/ros/gazebo_model_attachment_plugin/commits/master)
[![coverage](https://git.web.boeing.com/robotics/ros/gazebo_model_attachment_plugin/badges/master/coverage.svg)](https://git.web.boeing.com/robotics/ros/gazebo_model_attachment_plugin/commits/master)

## Overview

This plugin exposes an attachment and also detachment services that can be called with ROS service call.

## Motivation

Implements the ability for realtime attach/detach of models in Gazebo. This is essential for processes such as EE change or consumable swap.

## API

#### Subscribed topics

N/A

#### Published topics

N/A

#### Parameters

N/A

#### Subscribed topics

N/A

#### Published topics

N/A

## Installation

Add the package to your .rosinstall file and then add the following line of code to your .world file.

```xml
<gazebo>
    <plugin name="gazebo_model_attachment_plugin" filename="libgazebo_model_attachment_plugin.so"></plugin>
</gazebo>
```

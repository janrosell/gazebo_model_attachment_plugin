// Copyright 2018 Boeing
#ifndef GAZEBO_EE_MONITOR_PLUGIN_GAZEBO_EE_MONITOR_PLUGIN_H
#define GAZEBO_EE_MONITOR_PLUGIN_GAZEBO_EE_MONITOR_PLUGIN_H

#include <mutex>
#include <thread>
#include <vector>
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <gazebo_ee_monitor_plugin/Attach.h>
#include <gazebo_ee_monitor_plugin/Detach.h>

namespace gazebo
{

struct FixedJoint
{
    std::string model1;
    physics::ModelPtr m1;
    std::string link1;
    physics::LinkPtr l1;

    std::string model2;
    physics::ModelPtr m2;
    std::string link2;
    physics::LinkPtr l2;

    physics::JointPtr joint;
};

class EEManager : public WorldPlugin
{
  public:
    EEManager();

    virtual ~EEManager();

  protected:
    void Load(physics::WorldPtr world, sdf::ElementPtr sdf);

  private:
    physics::WorldPtr world_;

    bool attachCallback(gazebo_ee_monitor_plugin::Attach::Request& req, gazebo_ee_monitor_plugin::Attach::Response& res);
    bool detachCallback(gazebo_ee_monitor_plugin::Detach::Request& req, gazebo_ee_monitor_plugin::Detach::Response& res);

    bool attach(std::string model1, std::string link1, std::string model2, std::string link2);
    bool detach(std::string model1, std::string link1, std::string model2, std::string link2);
    bool getJoint(std::string model1, std::string link1, std::string model2, std::string link2, FixedJoint &joint);

    ros::NodeHandle nh_;
    ros::ServiceServer attach_srv_;
    ros::ServiceServer detach_srv_;

    ros::CallbackQueue queue_;
    std::thread callback_queue_thread_;
    void queueThread();

    std::vector<FixedJoint> joints_;
};

}  // namespace gazebo

#endif  // GAZEBO_EE_MONITOR_PLUGIN_GAZEBO_EE_MONITOR_PLUGIN_H

// Copyright 2018 Boeing
#ifndef EE_MANAGER_PLUGIN_H
#define EE_MANAGER_PLUGIN_H


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
#include <gazebo/transport/transport.hh>

#include <ee_manager/EndEffector.h>
#include "gazebo_ee_monitor_plugin/Attach.h"
#include "gazebo_ee_monitor_plugin/AttachRequest.h"
#include "gazebo_ee_monitor_plugin/AttachResponse.h"

namespace gazebo
{


class EEManager : public ModelPlugin
{
  public:
    EEManager();

    virtual ~EEManager();

  protected:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    virtual void UpdateChild();

  private:
    void activeEECallback(const ee_manager::EndEffector msg);

    /// \brief Internal representation of a fixed joint
    struct fixedJoint{
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

    bool attach(std::string model1, std::string link1, std::string model2, std::string link2);
    bool detach(std::string model1, std::string link1, std::string model2, std::string link2);
    bool getJoint(std::string model1, std::string link1, std::string model2, std::string link2, fixedJoint &joint);

    physics::ModelPtr _model_local;

    ros::NodeHandle *rosnode_;
    ros::Subscriber sub_;

    std::mutex lock_;
    ee_manager::EndEffector active_ee_msg;


    std::string topic_name_;
    std::string robot_namespace_;
    std::string robot_model_name_;
    std::string ee_attachment_link_name_;
    bool got_ee_msg_;

    // Custom Callback Queue
    ros::CallbackQueue queue_;
    std::thread callback_queue_thread_;
    void queueThread();

    // Pointer to the update event connection
    event::ConnectionPtr update_connection_;

    std::vector<fixedJoint> joints;
};

}  // namespace gazebo

#endif  // EE_MANAGER_PLUGIN_H

// Copyright 2018 Boeing
#include <gazebo_ee_monitor_plugin/gazebo_ee_monitor_plugin.h>
#include <gazebo/physics/World.hh>
#include <vector>

namespace gazebo
{

EEManager::EEManager()
{
}

EEManager::~EEManager()
{
    // Custom Callback Queue
    queue_.clear();
    queue_.disable();
    sub_.shutdown();
    rosnode_->shutdown();
    callback_queue_thread_.join();
    joints.clear();
    delete rosnode_;

}

void EEManager::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    _model_local = _model;

    // Set up parameter defualts
    robot_model_name_ = "mobile_drilling_agv";
    ee_attachment_link_name_ = "master_ati_link";
    topic_name_ = "/ee_manager/active_ee";
    robot_namespace_ = "";
    got_ee_msg_ = false;

    if (_sdf->HasElement("robotNamespace"))
    {
        robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
        robot_namespace_.erase(std::remove(robot_namespace_.begin(), robot_namespace_.end(), '"'), robot_namespace_.end());
    }

    ROS_INFO_STREAM("Initialising EE Manager Plugin for robot: " << robot_namespace_);

    //Check for parameters and set if required
    if (!_sdf->HasElement("activeEETopic"))
    {
        ROS_WARN_NAMED("gazebo_ee_monitor_plugin", "gazebo_ee_monitor_plugin (ns = %s) missing <activeEETopic>, defaults to \"%s\"",
                       robot_namespace_.c_str(), topic_name_.c_str());
    }
    else
    {
        topic_name_ = _sdf->GetElement("activeEETopic")->Get<std::string>().c_str();
    }

    if (!_sdf->HasElement("eeAttachmentLink"))
    {
        ROS_WARN_NAMED("gazebo_ee_monitor_plugin", "gazebo_ee_monitor_plugin (ns = %s) missing <eeAttachmentLink>, defaults to \"%s\"",
                       robot_namespace_.c_str(), ee_attachment_link_name_.c_str());
    }
    else
    {
        ee_attachment_link_name_ = _sdf->GetElement("eeAttachmentLink")->Get<std::string>().c_str();
    }

    if (!_sdf->HasElement("robotModelName"))
    {
        ROS_WARN_NAMED("gazebo_ee_monitor_plugin", "gazebo_ee_monitor_plugin (ns = %s) missing <robotModelName>, defaults to \"%s\"",
                       robot_namespace_.c_str(), robot_model_name_.c_str());
    }
    else
    {
        robot_model_name_ = _sdf->GetElement("robotModelName")->Get<std::string>().c_str();
    }

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                             << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }
    rosnode_ = new ros::NodeHandle(robot_namespace_);

    ROS_INFO_NAMED("gazebo_ee_monitor_plugin", "Subscribing to topic: %s", topic_name_.c_str());
    sub_ = rosnode_->subscribe(topic_name_, 1, &EEManager::activeEECallback, this,
                               ros::TransportHints().tcpNoDelay());

    // Custom Callback Queue
    callback_queue_thread_ = std::thread(&EEManager::queueThread, this);
    // New Mechanism for Updating every World Cycle
    // Listen to the update event. This event is broadcast every simulation iteration
    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&EEManager::UpdateChild, this));

}

void EEManager::activeEECallback(const ee_manager::EndEffector msg)
{
    ROS_INFO_STREAM("Got active EE topic update.");
    std::lock_guard<std::mutex> lock(lock_);
    active_ee_msg = msg;
    got_ee_msg_ = true;
}

void EEManager::UpdateChild()
{
    std::lock_guard<std::mutex> lock(lock_);
    if (got_ee_msg_)
    {
        got_ee_msg_ = false;

        std::string model_name_1 = robot_model_name_; // TODO: config this out
        std::string model_name_2 = active_ee_msg.name;
        std::string link_name_1 = ee_attachment_link_name_; // TODO: config this out
        std::string link_name_2 = active_ee_msg.ee_link_name;

        ROS_INFO_STREAM("Received request to attach model: '" << model_name_1
                                                              << "' using link: '" << link_name_1 << "' with model: '"
                                                              << model_name_2 << "' using link: '" << link_name_2
                                                              << "'");

        //Check the robot model and link exist
        if (_model_local->GetWorld()->ModelByName(model_name_1) == NULL)
        {
            ROS_WARN_STREAM("Could not perform end effector attach. Model: '" << model_name_1 << "' not found.");
            return;
        }
        if (_model_local->GetWorld()->ModelByName(model_name_1)->GetLink(link_name_1) == NULL)
        {
            ROS_WARN_STREAM(
                "Could not perform end effector attach. Model: '" << model_name_1 << "' does not have link '"
                                                                  << link_name_1 << "'.");
            return;
        }

        if (model_name_2 == "")
        {
            ROS_INFO_STREAM("Desired end effector is empty. Assuming detachment required.");
        }
        else
        {
            //Check the end effector model and link exists
            if (_model_local->GetWorld()->ModelByName(model_name_2) == NULL)
            {
                ROS_WARN_STREAM("Could not perform end effector attach. Model: '" << model_name_2 << "' not found.");
                return;
            }

            if (_model_local->GetWorld()->ModelByName(model_name_2)->GetLink(link_name_2) == NULL)
            {
                ROS_WARN_STREAM(
                    "Could not perform end effector attach. Model: '" << model_name_2 << "' does not have link '"
                                                                      << link_name_2 << "'.");
                return;
            }
        }

        //First detach all all attached joints
        using Iter_msg = std::vector<fixedJoint>::const_iterator;
        for (Iter_msg joint = joints.begin();
             joint != joints.end(); ++joint)
        {
            ROS_INFO_STREAM("Detaching model: '" << joint->model1
                                                 << "' using link: '" << joint->link1 << "' with model: '"
                                                 << joint->model2 << "' using link: '" << joint->link2
                                                 << "'");
            if (this->detach(joint->model1, joint->link1, joint->model2, joint->link2) == false)
            {
                ROS_ERROR_STREAM("Could not perform the detach.");
                return;
            }
            else
            {
                ROS_INFO_STREAM("Detach was succesful");
            }
        }

        // Attach a new end effector only if the desired is not blank
        if (model_name_2 != "")
        {
            if (!this->attach(model_name_1, link_name_1,
                              model_name_2, link_name_2))
            {
                ROS_ERROR_STREAM("Could not make the attach.");
                return;
            }
            else
            {
                ROS_INFO_STREAM("Attach was succesful");
            }
        }

    }

}

void EEManager::queueThread()
{
    const double timeout = 0.01;
    while (rosnode_->ok())
    {
        queue_.callAvailable(ros::WallDuration(timeout));
    }
}

bool EEManager::attach(std::string model1, std::string link1, std::string model2, std::string link2)
{

    // look for any previous instance of the joint first.
    // if we try to create a joint in between two links
    // more than once (even deleting any reference to the first one)
    // gazebo hangs/crashes
    fixedJoint j;
    if (this->getJoint(model1, link1, model2, link2, j) == false)
    {
        ROS_INFO_STREAM("Creating new joint.");

        j.model1 = model1;
        j.link1 = link1;
        j.model2 = model2;
        j.link2 = link2;
        physics::BasePtr b1 = _model_local->GetWorld()->ModelByName(model1);
        if (b1 == NULL)
        {
            ROS_ERROR_STREAM(model1 << " model was not found");
            return false;
        }
        physics::BasePtr b2 = _model_local->GetWorld()->ModelByName(model2);
        if (b2 == NULL)
        {
            ROS_ERROR_STREAM(model2 << " model was not found");
            return false;
        }

        physics::ModelPtr m1(dynamic_cast<physics::Model *>(b1.get()));
        j.m1 = m1;
        physics::ModelPtr m2(dynamic_cast<physics::Model *>(b2.get()));
        j.m2 = m2;

        ROS_DEBUG_STREAM("Getting link: '" << link1 << "' from model: '" << model1 << "'");
        physics::LinkPtr l1 = m1->GetLink(link1);
        if (l1 == NULL)
        {
            ROS_ERROR_STREAM(link1 << " link was not found");
            return false;
        }
        if (l1->GetInertial() == NULL)
        {
            ROS_ERROR_STREAM("link1 inertia is NULL!");
        }
        else
        {
            ROS_DEBUG_STREAM("link1 inertia is not NULL, for example, mass is: " << l1->GetInertial()->Mass());
        }
        j.l1 = l1;

        ROS_DEBUG_STREAM("Getting link: '" << link2 << "' from model: '" << model2 << "'");
        physics::LinkPtr l2 = m2->GetLink(link2);
        if (l2 == NULL)
        {
            ROS_ERROR_STREAM(link2 << " link was not found");
            return false;
        }
        if (l2->GetInertial() == NULL)
        {
            ROS_ERROR_STREAM("link2 inertia is NULL!");
        }
        else
        {
            ROS_DEBUG_STREAM("link2 inertia is not NULL, for example, mass is: " << l2->GetInertial()->Mass());
        }
        j.l2 = l2;

        // Then push the update
        j.joint = _model_local->GetWorld()->Physics()->CreateJoint("fixed", m1);

        this->joints.push_back(j); // local copy
    }

    j.joint->Attach(j.l1, j.l2);
    j.joint->Load(j.l1, j.l2, ignition::math::Pose3d());
    j.joint->SetModel(j.m2);
    j.joint->Init();
    j.m2->SetWorldPose(j.l1->WorldPose()); // Set EE position to be relative to ee_link
    return true;
}

bool EEManager::detach(std::string model1, std::string link1,
                       std::string model2, std::string link2)
{
    // search for the instance of joint and do detach
    fixedJoint j;
    if (this->getJoint(model1, link1, model2, link2, j))
    {
        j.joint->Detach();
        return true;
    }

    return false;
}

bool EEManager::getJoint(std::string model1, std::string link1,
                         std::string model2, std::string link2,
                         fixedJoint &joint)
{
    fixedJoint j;
    for (std::vector<fixedJoint>::iterator it = this->joints.begin(); it != this->joints.end(); ++it)
    {
        j = *it;
        if ((j.model1.compare(model1) == 0) && (j.model2.compare(model2) == 0)
            && (j.link1.compare(link1) == 0) && (j.link2.compare(link2) == 0))
        {
            joint = j;
            return true;
        }
    }
    return false;

}

GZ_REGISTER_MODEL_PLUGIN(EEManager)
}  // namespace gazebo



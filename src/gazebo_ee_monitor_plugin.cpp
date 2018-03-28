// Copyright 2018 Boeing
#include <gazebo/physics/World.hh>
#include <gazebo_ee_monitor_plugin/gazebo_ee_monitor_plugin.h>
#include <vector>
#include <string>

namespace gazebo
{

EEManager::EEManager()
{
}

EEManager::~EEManager()
{
    queue_.clear();
    queue_.disable();
    callback_queue_thread_.join();
}

void EEManager::Load(physics::WorldPtr world, sdf::ElementPtr sdf)
{
    ROS_INFO("Initialising CustomWorldPlugin Plugin");
    world_ = world;

    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin");
        return;
    }
    nh_ = ros::NodeHandle("~");

    attach_srv_ = nh_.advertiseService("attach", &EEManager::attachCallback, this);
    detach_srv_ = nh_.advertiseService("detach", &EEManager::detachCallback, this);

    callback_queue_thread_ = std::thread(&EEManager::queueThread, this);
}

bool EEManager::attachCallback(gazebo_ee_monitor_plugin::Attach::Request& req,
                               gazebo_ee_monitor_plugin::Attach::Response& res)
{
    ROS_INFO_STREAM("Received request to attach model: '" << req.model_name_1 << "' using link: '" << req.link_name_1
                                                          << "' with model: '" << req.model_name_2 << "' using link: '"
                                                          << req.link_name_2 << "'");

    #if GAZEBO_MAJOR_VERSION >= 8
    physics::ModelPtr m1 = world_->ModelByName(req.model_name_1);
    #else
    physics::ModelPtr m1 = world_->GetModel(req.model_name_1);
    #endif
    if (m1 == nullptr)
    {
        const std::string error_msg = "Could not find model " + req.model_name_1;
        ROS_ERROR_STREAM(error_msg);
        res.message = error_msg;
        res.success = false;
        return true;
    }

    if (m1->GetLink(req.link_name_1) == nullptr)
    {
        const std::string error_msg = "Could not find link " + req.link_name_1 + " on model " + req.model_name_1;
        ROS_ERROR_STREAM(error_msg);
        res.message = error_msg;
        res.success = false;
        return true;
    }

    #if GAZEBO_MAJOR_VERSION >= 8
    physics::ModelPtr m2 = world_->ModelByName(req.model_name_2);
    #else
    physics::ModelPtr m2 = world_->GetModel(req.model_name_2);
    #endif
    if (m2 == nullptr)
    {
        const std::string error_msg = "Could not find model " + req.model_name_2;
        ROS_ERROR_STREAM(error_msg);
        res.message = error_msg;
        res.success = false;
        return true;
    }

    if (m2->GetLink(req.link_name_2) == NULL)
    {
        const std::string error_msg = "Could not find link " + req.link_name_2 + " on model " + req.model_name_2;
        ROS_ERROR_STREAM(error_msg);
        res.message = error_msg;
        res.success = false;
        return true;
    }

    if (!attach(req.model_name_1, req.link_name_1, req.model_name_2, req.link_name_2))
    {
        const std::string error_msg = "Failed to attach";
        ROS_ERROR_STREAM(error_msg);
        res.message = error_msg;
        res.success = false;
        return true;
    }

    res.success = true;
    return true;
}

bool EEManager::detachCallback(gazebo_ee_monitor_plugin::Detach::Request& req,
                               gazebo_ee_monitor_plugin::Detach::Response& res)
{
    ROS_INFO_STREAM("Received request to detach model: '" << req.model_name_1 << "' using link: '" << req.link_name_1
                                                          << "' with model: '" << req.model_name_2 << "' using link: '"
                                                          << req.link_name_2 << "'");

    #if GAZEBO_MAJOR_VERSION >= 8
    physics::ModelPtr m1 = world_->ModelByName(req.model_name_1);
    #else
    physics::ModelPtr m1 = world_->GetModel(req.model_name_1);
    #endif
    if (m1 == nullptr)
    {
        const std::string error_msg = "Could not find model " + req.model_name_1;
        ROS_ERROR_STREAM(error_msg);
        res.message = error_msg;
        res.success = false;
        return true;
    }

    if (m1->GetLink(req.link_name_1) == nullptr)
    {
        const std::string error_msg = "Could not find link " + req.link_name_1 + " on model " + req.model_name_1;
        ROS_ERROR_STREAM(error_msg);
        res.message = error_msg;
        res.success = false;
        return true;
    }

    #if GAZEBO_MAJOR_VERSION >= 8
    physics::ModelPtr m2 = world_->ModelByName(req.model_name_2);
    #else
    physics::ModelPtr m2 = world_->GetModel(req.model_name_2);
    #endif
    if (m2 == nullptr)
    {
        const std::string error_msg = "Could not find model " + req.model_name_2;
        ROS_ERROR_STREAM(error_msg);
        res.message = error_msg;
        res.success = false;
        return true;
    }

    if (m2->GetLink(req.link_name_2) == NULL)
    {
        const std::string error_msg = "Could not find link " + req.link_name_2 + " on model " + req.model_name_2;
        ROS_ERROR_STREAM(error_msg);
        res.message = error_msg;
        res.success = false;
        return true;
    }

    if (!detach(req.model_name_1, req.link_name_1, req.model_name_2, req.link_name_2))
    {
        const std::string error_msg = "Failed to detach";
        ROS_ERROR_STREAM(error_msg);
        res.message = error_msg;
        res.success = false;
        return true;
    }

    res.success = true;
    return true;
}

void EEManager::queueThread()
{
    const double timeout = 0.01;
    while (nh_.ok())
    {
        queue_.callAvailable(ros::WallDuration(timeout));
    }
}

bool EEManager::attach(std::string model1, std::string link1, std::string model2, std::string link2)
{
    FixedJoint j;
    if (!getJoint(model1, link1, model2, link2, j))
    {
        ROS_INFO_STREAM("Creating new joint.");

        j.model1 = model1;
        j.link1 = link1;
        j.model2 = model2;
        j.link2 = link2;

        #if GAZEBO_MAJOR_VERSION >= 8
        j.m1 = world_->ModelByName(model1);
        #else
        j.m1 = world_->GetModel(model1);
        #endif

        if (j.m1 == nullptr)
        {
            ROS_ERROR_STREAM(model1 << " model was not found");
            return false;
        }

        #if GAZEBO_MAJOR_VERSION >= 8
        j.m2 = world_->ModelByName(model2);
        #else
        j.m2 = world_->GetModel(model2);
        #endif

        if (j.m2 == nullptr)
        {
            ROS_ERROR_STREAM(model2 << " model was not found");
            return false;
        }

        ROS_DEBUG_STREAM("Getting link: '" << link1 << "' from model: '" << model1 << "'");
        physics::LinkPtr l1 = j.m1->GetLink(link1);
        if (l1 == nullptr)
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
            double mass;
            #if GAZEBO_MAJOR_VERSION >= 8
            mass = l1->GetInertial()->Mass();
            #else
            mass = l2->GetInertial()->GetMass();
            #endif
            ROS_DEBUG_STREAM("link1 inertia is not NULL, for example, mass is: " << mass);
        }
        j.l1 = l1;

        ROS_DEBUG_STREAM("Getting link: '" << link2 << "' from model: '" << model2 << "'");
        physics::LinkPtr l2 = j.m2->GetLink(link2);
        if (l2 == nullptr)
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
            double mass;
            #if GAZEBO_MAJOR_VERSION >= 8
            mass = l2->GetInertial()->Mass();
            #else
            mass = l2->GetInertial()->GetMass();
            #endif
            ROS_DEBUG_STREAM("link2 inertia is not NULL, for example, mass is: " << mass);
        }
        j.l2 = l2;

        #if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d l1wp = l1->WorldPose();
        ignition::math::Pose3d l2rp = l2->RelativePose();
        #else
        math::Pose l1wp = l1->GetWorldPose();
        math::Pose l2rp = l2->GetRelativePose();
        #endif

        {
            const bool is_paused = world_->IsPaused();
            world_->SetPaused(true);

            #if GAZEBO_MAJOR_VERSION >= 8
            j.m2->SetWorldPose(l1wp * l2rp.Inverse());
            #else
            j.m2->SetWorldPose(l1wp * l2rp.GetInverse());
            #endif

            j.joint = j.m1->CreateJoint("attachment", "fixed", j.l1, j.l2);

            if (j.joint == nullptr)
            {
                ROS_ERROR_STREAM("CreateJoint returned nullptr");
                return false;
            }

            world_->SetPaused(is_paused);
        }

        joints_.push_back(j);  // local copy
    }

    return true;
}

bool EEManager::detach(std::string model1, std::string link1, std::string model2, std::string link2)
{
    // search for the instance of joint and do detach
    FixedJoint j;
    if (getJoint(model1, link1, model2, link2, j))
    {
        return j.m1->RemoveJoint(j.joint->GetName());
    }

    return false;
}

bool EEManager::getJoint(std::string model1, std::string link1, std::string model2, std::string link2,
                         FixedJoint& joint)
{
    FixedJoint j;
    for (std::vector<FixedJoint>::iterator it = joints_.begin(); it != joints_.end(); ++it)
    {
        j = *it;
        if ((j.model1.compare(model1) == 0) && (j.model2.compare(model2) == 0) && (j.link1.compare(link1) == 0) &&
            (j.link2.compare(link2) == 0))
        {
            joint = j;
            return true;
        }
    }
    return false;
}

GZ_REGISTER_WORLD_PLUGIN(EEManager)
}  // namespace gazebo

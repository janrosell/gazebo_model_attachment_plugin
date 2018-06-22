// Copyright 2018 Boeing
#include <gazebo/physics/World.hh>
#include <gazebo_model_attachment_plugin/gazebo_model_attachment_plugin.h>
#include <string>
#include <vector>

namespace gazebo
{

ModelAttachmentPlugin::ModelAttachmentPlugin()
{
}

ModelAttachmentPlugin::~ModelAttachmentPlugin()
{
    queue_.clear();
    queue_.disable();
    callback_queue_thread_.join();
}

// cppcheck-suppress unusedFunction
void ModelAttachmentPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf)
{
    ROS_INFO("Initialising CustomWorldPlugin Plugin");
    world_ = world;
    auto sdf_ptr = sdf;  // As all Parameters must be used -Wall, but we can't change internal Gazebo function

    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin");
        return;
    }
    nh_ = ros::NodeHandle("~");

    attach_srv_ = nh_.advertiseService("attach", &ModelAttachmentPlugin::attachCallback, this);
    detach_srv_ = nh_.advertiseService("detach", &ModelAttachmentPlugin::detachCallback, this);

    callback_queue_thread_ = std::thread(&ModelAttachmentPlugin::queueThread, this);
}

bool ModelAttachmentPlugin::attachCallback(gazebo_model_attachment_plugin::Attach::Request& req,
                                           gazebo_model_attachment_plugin::Attach::Response& res)
{
    ROS_INFO_STREAM("Received request to attach model: '" << req.model_name_1 << "' to '" << req.model_name_2);

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

    physics::LinkPtr l1 = m1->GetLink(req.link_name_1);
    if (l1 == nullptr)
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

    physics::LinkPtr l2 = m2->GetLink(req.link_name_2);
    if (l2 == nullptr)
    {
        const std::string error_msg = "Could not find link " + req.link_name_2 + " on model " + req.model_name_2;
        ROS_ERROR_STREAM(error_msg);
        res.message = error_msg;
        res.success = false;
        return true;
    }

    try
    {
        attach(req.joint_name, m1, m2, l1, l2);
    }
    catch (const std::exception& e)
    {
        const std::string error_msg = "Failed to detach: " + std::string(e.what());
        ROS_ERROR_STREAM(error_msg);
        res.message = error_msg;
        res.success = false;
        return true;
    }

    res.success = true;
    return true;
}

bool ModelAttachmentPlugin::detachCallback(gazebo_model_attachment_plugin::Detach::Request& req,
                                           gazebo_model_attachment_plugin::Detach::Response& res)
{
    ROS_INFO_STREAM("Received request to detach model: '" << req.model_name_1 << "' from '" << req.model_name_2);

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

    try
    {
        detach(req.joint_name, m1, m2);
    }
    catch (const std::exception& e)
    {
        const std::string error_msg = "Failed to detach: " + std::string(e.what());
        ROS_ERROR_STREAM(error_msg);
        res.message = error_msg;
        res.success = false;
        return true;
    }

    res.success = true;
    return true;
}

void ModelAttachmentPlugin::queueThread()
{
    const double timeout = 0.01;
    while (nh_.ok())
    {
        queue_.callAvailable(ros::WallDuration(timeout));
    }
}

void ModelAttachmentPlugin::attach(const std::string& joint_name, physics::ModelPtr m1, physics::ModelPtr m2,
                                   physics::LinkPtr l1, physics::LinkPtr l2)
{
    if (m1 == nullptr)
        throw std::runtime_error("Model 1 is null");

    if (m2 == nullptr)
        throw std::runtime_error("Model 2 is null");

    if (l1 == nullptr)
        throw std::runtime_error("Link 1 is null");

    if (l2 == nullptr)
        throw std::runtime_error("Link 2 is null");

#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d l1wp = l1->WorldPose();
    ignition::math::Pose3d l2rp = l2->RelativePose();
#else
    math::Pose l1wp = l1->GetWorldPose();
    math::Pose l2rp = l2->GetRelativePose();
#endif

    const bool is_paused = world_->IsPaused();
    world_->SetPaused(true);

#if GAZEBO_MAJOR_VERSION >= 8
    m2->SetWorldPose(l1wp * l2rp.Inverse());
#else
    m2->SetWorldPose(l1wp * l2rp.GetInverse());
#endif

    physics::JointPtr joint = m1->CreateJoint(joint_name, "fixed", l1, l2);

    if (joint == nullptr)
        throw std::runtime_error("CreateJoint returned nullptr");

    m1->AddChild(m2);

    world_->SetPaused(is_paused);
}

void ModelAttachmentPlugin::detach(const std::string& joint_name, physics::ModelPtr m1, physics::ModelPtr m2)
{
    if (m1 == nullptr)
        throw std::runtime_error("Model 1 is null");

    if (m2 == nullptr)
        throw std::runtime_error("Model 2 is null");

    physics::JointPtr joint = m1->GetJoint(joint_name);
    if (joint == nullptr)
        throw std::runtime_error("No joint on model " + m1->GetName() + " by name " + joint_name);

    bool success = m1->RemoveJoint(joint_name);

    if (!success)
        throw std::runtime_error("Unable to remove joint from model");

#if GAZEBO_MAJOR_VERSION >= 8
    m2->SetParent(m1->GetWorld()->ModelByName("default"));
#else
    m2->SetParent(m1->GetWorld()->GetModel("default"));
#endif

    // We need to flush the children vector of the parent
    // TODO: Calling m1->RemoveChild(boost::dynamic_pointer_cast<physics::Entity>(m2)); will also destroy the child
    // TODO: through a call to Fini()
    // TODO: We need a way to remove the child from model->children without calling Fini()
    // For now assume our attachment point has no other relevant children
    m1->RemoveChildren();

    return;
}

GZ_REGISTER_WORLD_PLUGIN(ModelAttachmentPlugin)
}  // namespace gazebo

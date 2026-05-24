#include "crazyflie_interfaces_rviz_displays/pose_named_array_display.hpp"
#include <rviz_common/logging.hpp>
#include "rviz_common/validate_floats.hpp"
#include "rviz_common/msg_conversions.hpp"


#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

namespace crazyflie_interfaces_rviz_displays
{
struct ShapeType
{
  enum
  {
    Arrow2d,
    Arrow3d,
    Axes,
  };
};

PoseNamedArrayDisplay::PoseNamedArrayDisplay()
{
    initializeProperties();
    shape_property_->addOption("Arrow (Flat)", ShapeType::Arrow2d);
    shape_property_->addOption("Arrow (3D)", ShapeType::Arrow3d);
    shape_property_->addOption("Axes", ShapeType::Axes); 
}

PoseNamedArrayDisplay::PoseNamedArrayDisplay(
    rviz_common::DisplayContext * display_context, 
    Ogre::SceneNode * scene_node)
    : PoseNamedArrayDisplay() 
{
    context_ = display_context;
    scene_node_ = scene_node;
    scene_manager_ = context_->getSceneManager();
    
    clock_ = context_->getClock();
    arrows2d_ = std::make_unique<rviz_default_plugins::displays::FlatArrowsArray>(scene_manager_);
    arrows2d_->createAndAttachManualObject(scene_node);
    arrow_node_ = scene_node_->createChildSceneNode();
    axes_node_ = scene_node_->createChildSceneNode();
    names_node_ = scene_node_->createChildSceneNode();
    point_node_ = scene_node_->createChildSceneNode();

    updateShapeChoice();
}

void PoseNamedArrayDisplay::initializeProperties() 
{
    pose_timeout_property_ = new rviz_common::properties::FloatProperty(
        "Pose Timeout",
        10.0,
        "Time in seconds after which a pose is considered outdated and not shown anymore.",
        this);
    pose_timeout_property_->setMin(1.0);

    rotation_validity_property_ = new rviz_common::properties::BoolProperty(
        "Show Rotation Validity",
        true,
        "If selected, poses with invalid rotation will be shown not as an arrow/axes but as a point.",
        this);

    show_names_property_ = new rviz_common::properties::BoolProperty(
        "Show Names", 
        true,
        "Whether to show the names of the poses as text in RViz.",
        this, 
        SLOT(updateShowNames()));

    shape_property_ = new rviz_common::properties::EnumProperty(
        "Shape", 
        "Arrow (Flat)",
        "The shape to display each pose as.",
        this,
        SLOT(updateShapeChoice())
    );
}

void PoseNamedArrayDisplay::onInitialize()
{
  MFDClass::onInitialize();

  clock_ = context_->getClock();
  arrows2d_ = std::make_unique<rviz_default_plugins::displays::FlatArrowsArray>(scene_manager_);
  arrows2d_->createAndAttachManualObject(scene_node_);
  arrow_node_ = scene_node_->createChildSceneNode();
  axes_node_ = scene_node_->createChildSceneNode();
  names_node_ = scene_node_->createChildSceneNode();
  point_node_ = scene_node_->createChildSceneNode();
  updateShapeChoice();
}
void PoseNamedArrayDisplay::onEnable()
{
    MFDClass::onEnable();
    names_node_->setVisible(show_names_property_->getBool());
    arrow_node_->setVisible(true);
    axes_node_->setVisible(true);
    point_node_->setVisible(true);
}

void PoseNamedArrayDisplay::onDisable()
{
    MFDClass::onDisable();
    names_node_->setVisible(false);
    arrow_node_->setVisible(false);
    axes_node_->setVisible(false);
    point_node_->setVisible(false);
}

bool PoseNamedArrayDisplay::validateFloats(const crazyflie_interfaces::msg::PoseNamedArray & msg) 
{
    for (const auto & pose : msg.poses) {
        if (!rviz_common::validateFloats(pose.pose)) {
            return false;
        }
    }
    return true;
}

bool PoseNamedArrayDisplay::setTransform(std_msgs::msg::Header const & header) 
{
    rclcpp::Time time_stamp(header.stamp, RCL_ROS_TIME);
    if (!updateFrame(header.frame_id, time_stamp)) {
        setMissingTransformToFixedFrame(header.frame_id);
        return false;
    }
    setTransformOk();

    return true;
}

inline double computeAlpha(double age_seconds, double pose_timeout)
{
    return std::clamp(
        (1.0 - (age_seconds / pose_timeout)) / 0.7,
        0.0,
        1.0
    );
}

void PoseNamedArrayDisplay::update(std::chrono::nanoseconds wall_dt, std::chrono::nanoseconds ros_dt) {
    (void)wall_dt;
    (void)ros_dt;
    rclcpp::Time now = clock_->now();
    double pose_timeout = pose_timeout_property_->getFloat();
    rclcpp::Time timeout_threshold = now - rclcpp::Duration::from_seconds(pose_timeout);

    for (auto it = poses_map_.begin(); it != poses_map_.end(); ) {
        if (it->second.last_update_time < timeout_threshold) {
            it = poses_map_.erase(it);
        } else {
            ++it;
        }
    }

    for (auto & name_text_pair : name_texts_) {
        const std::string & name = name_text_pair.first;

        auto pose_it = poses_map_.find(name);

        if (pose_it == poses_map_.end()) {
            name_text_pair.second.second->setVisible(false);
        } else {
            Ogre::Vector3 position = rviz_common::pointMsgToOgre(pose_it->second.pose_named.pose.position);
            name_text_pair.second.second->setPosition(position);    
            name_text_pair.second.second->setVisible(show_names_property_->getBool());
            name_text_pair.second.first->setColor(Ogre::ColourValue(1.0, 1.0, 1.0, computeAlpha((now - pose_it->second.last_update_time).seconds(), pose_timeout)));
        }
    }

    
    poses_.clear();
    poses_alphas_.clear();
    invalid_rotation_poses_.clear();
    invalid_rotation_poses_alphas_.clear();
    
    poses_.reserve(poses_map_.size());
    poses_alphas_.reserve(poses_map_.size());
    invalid_rotation_poses_.reserve(poses_map_.size());
    invalid_rotation_poses_alphas_.reserve(poses_map_.size());

    bool separate_invalid = rotation_validity_property_->getBool();

    for (const auto& [name, pose_msg] : poses_map_) {

        auto & pose_named = pose_msg.pose_named;
        auto & last_update_time = pose_msg.last_update_time;
        auto& target =
            (pose_named.rotation_valid || !separate_invalid)
            ? poses_
            : invalid_rotation_poses_;
        auto& pose_alpha = 
            (pose_named.rotation_valid || !separate_invalid)
            ? poses_alphas_ 
            : invalid_rotation_poses_alphas_;

        pose_alpha.push_back(computeAlpha((now - last_update_time).seconds(), pose_timeout));
        auto& pose = target.emplace_back();

        pose.position =
            rviz_common::pointMsgToOgre(pose_named.pose.position);

        pose.orientation =
            rviz_common::quaternionMsgToOgre(pose_named.pose.orientation);
    } 
    updateDisplay();

    // context_->queueRender();
}

void PoseNamedArrayDisplay::updateDisplay()
{

  int shape = shape_property_->getOptionInt();
  switch (shape) {
    case ShapeType::Arrow2d:
      updateArrows2d();
      arrows3d_.clear();
      axes_.clear();
      break;
    case ShapeType::Arrow3d:
      updateArrows3d();
      arrows2d_->clear();
      axes_.clear();
      break;
    case ShapeType::Axes:
      updateAxes();
      arrows2d_->clear();
      arrows3d_.clear();
      break;
  }
  updatePoints();
}

void PoseNamedArrayDisplay::updateArrows2d()
{
  arrows2d_->updateManualObject(
    Ogre::ColourValue(1.0, 0.0, 0.0, 1.0),
    1.0,
    0.3,
    poses_);
}

void PoseNamedArrayDisplay::updateArrows3d()
{
    while (arrows3d_.size() < poses_.size()) {
        arrows3d_.push_back(makeArrow3d());
    }
    while (arrows3d_.size() > poses_.size()) {
        arrows3d_.pop_back();
    }

    Ogre::Quaternion adjust_orientation(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y);
    for (std::size_t i = 0; i < poses_.size(); ++i) {
        arrows3d_[i]->setPosition(poses_[i].position);
        arrows3d_[i]->setOrientation(poses_[i].orientation * adjust_orientation);
        arrows3d_[i]->setColor(1.0, 0.0, 0.0, poses_alphas_[i]);
    }
}

void PoseNamedArrayDisplay::updateAxes()
{
  while (axes_.size() < poses_.size()) {
    axes_.push_back(makeAxes());
  }
  while (axes_.size() > poses_.size()) {
    axes_.pop_back();
  }
  for (std::size_t i = 0; i < poses_.size(); ++i) {
    axes_[i]->setPosition(poses_[i].position);
    axes_[i]->setOrientation(poses_[i].orientation);
    axes_[i]->setColor(1.0, 0.0, 0.0, poses_alphas_[i]);
  }
}

void PoseNamedArrayDisplay::updatePoints()
{
    while (points_.size() < invalid_rotation_poses_.size()) {
        points_.push_back(makeSphere());
    }
    while (points_.size() > invalid_rotation_poses_.size()) {
        points_.pop_back();
    }

    for (std::size_t i = 0; i < invalid_rotation_poses_.size(); ++i) {
        points_[i]->setPosition(invalid_rotation_poses_[i].position);
        points_[i]->setColor(1.0, 0.0, 0.0, invalid_rotation_poses_alphas_[i]);
    }
}


std::unique_ptr<rviz_rendering::Arrow> PoseNamedArrayDisplay::makeArrow3d()
{
  Ogre::ColourValue color = Ogre::ColourValue::Red;

  auto arrow = std::make_unique<rviz_rendering::Arrow>(
    scene_manager_,
    arrow_node_,
    0.23,
    0.01,
    0.07,
    0.03
  );

  arrow->setColor(color);
  return arrow;
}

std::unique_ptr<rviz_rendering::Axes> PoseNamedArrayDisplay::makeAxes()
{
  return std::make_unique<rviz_rendering::Axes>(
    scene_manager_,
    axes_node_,
    0.3,
    0.01
  );
}

std::unique_ptr<rviz_rendering::Shape> PoseNamedArrayDisplay::makeSphere()
{
    auto sphere =  std::make_unique<rviz_rendering::Shape>(
        rviz_rendering::Shape::Sphere,
        scene_manager_,
        point_node_
    );
    sphere->setScale(Ogre::Vector3(0.08, 0.08, 0.02));
    sphere->setColor(Ogre::ColourValue::Red);
    return sphere;
}


void PoseNamedArrayDisplay::updateShapeChoice()
{
  if (initialized()) {
    updateDisplay();
  }
}

void PoseNamedArrayDisplay::updateShowNames()
{
    names_node_->setVisible(show_names_property_->getBool());
}

void PoseNamedArrayDisplay::processMessage(const crazyflie_interfaces::msg::PoseNamedArray::ConstSharedPtr msg)
{
    if (!validateFloats(*msg)) {
        setStatus(
        rviz_common::properties::StatusProperty::Error,
        "Topic",
        "Message contained invalid floating point values (nans or infs)");
        return;
    }

    if (!setTransform(msg->header))
        return;

    // Create a text object for each pose we have never seen before.
    for (auto & pose : msg->poses)
    {
        std::string name = pose.name;
        if (name_texts_.find(name) == name_texts_.end()) {
            auto text = new rviz_rendering::MovableText(name, "Liberation Sans", 0.1f);
            text->setTextAlignment(rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_BELOW);
            auto name_node = names_node_->createChildSceneNode();
            name_node->attachObject(text);
            name_texts_[name] = std::make_pair(text, name_node);
        }
    }

    rclcpp::Time time_stamp = clock_->now();
    for (const auto & pose : msg->poses) {
        poses_map_[pose.name] = {pose, time_stamp};
    }
}
} // namespace crazyflie_interfaces_rviz_displays


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    crazyflie_interfaces_rviz_displays::PoseNamedArrayDisplay,
    rviz_common::Display
)
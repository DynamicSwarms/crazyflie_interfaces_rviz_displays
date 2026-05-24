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

void PoseNamedArrayDisplay::updateDisplay()
{
  int shape = shape_property_->getOptionInt();
  bool show_names = show_names_property_->getBool();
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

    for (auto & name_text_pair : name_texts_) {
        const std::string & name = name_text_pair.first;

        auto it = std::find_if(
            msg->poses.begin(),
            msg->poses.end(),
            [&name](const crazyflie_interfaces::msg::PoseNamed & p)
            {
                return p.name == name;
            });

        if (it == msg->poses.end()) {
                name_text_pair.second.second->setVisible(false);
        } else {
            Ogre::Vector3 position = rviz_common::pointMsgToOgre(it->pose.position);
            name_text_pair.second.second->setPosition(position);    
            name_text_pair.second.second->setVisible(show_names_property_->getBool());
        }
    }

    
    int nbr_valid_rotations = 0;
    int nbr_invalid_rotations = 0;
    bool seperate_invalid_rotations = rotation_validity_property_->getBool();

    for (std::size_t i = 0; i < msg->poses.size(); ++i) {
        if (msg->poses[i].rotation_valid || !seperate_invalid_rotations) {
            nbr_valid_rotations++;
        } else {
            nbr_invalid_rotations++;
        }
    }
    poses_.resize(nbr_valid_rotations);
    invalid_rotation_poses_.resize(nbr_invalid_rotations);


    int valid_index = 0;
    int invalid_index = 0;
    for (std::size_t i = 0; i < msg->poses.size(); ++i) {
        if (msg->poses[i].rotation_valid || !seperate_invalid_rotations) {
            poses_[valid_index].position = rviz_common::pointMsgToOgre(msg->poses[i].pose.position);
            poses_[valid_index].orientation = rviz_common::quaternionMsgToOgre(msg->poses[i].pose.orientation);
            valid_index++;
        } else {
            invalid_rotation_poses_[invalid_index].position = rviz_common::pointMsgToOgre(msg->poses[i].pose.position);
            invalid_rotation_poses_[invalid_index].orientation = rviz_common::quaternionMsgToOgre(msg->poses[i].pose.orientation);
            invalid_index++;
        }
    }

    updateDisplay();
    context_->queueRender();
}
} // namespace crazyflie_interfaces_rviz_displays


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    crazyflie_interfaces_rviz_displays::PoseNamedArrayDisplay,
    rviz_common::Display
)
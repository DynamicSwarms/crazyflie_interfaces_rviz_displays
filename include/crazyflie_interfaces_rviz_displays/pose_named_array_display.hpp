#pragma once

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/enum_property.hpp>

#include "crazyflie_interfaces/msg/pose_named_array.hpp"

#include <memory>
#include <vector>
#include <unordered_map>
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/axes.hpp"
#include <rviz_default_plugins/displays/pose_array/flat_arrows_array.hpp>
#include "rviz_rendering/objects/movable_text.hpp"

using rviz_default_plugins::displays::OgrePose;

namespace crazyflie_interfaces_rviz_displays
{   

    class PoseNamedArrayDisplay 
        : public rviz_common::MessageFilterDisplay<crazyflie_interfaces::msg::PoseNamedArray>
    {
        Q_OBJECT
    public:
        PoseNamedArrayDisplay();
        PoseNamedArrayDisplay(
            rviz_common::DisplayContext * display_context, 
            Ogre::SceneNode * scene_node
        );
    private: 
        void initializeProperties();
        bool validateFloats(const crazyflie_interfaces::msg::PoseNamedArray & msg);
        bool setTransform(std_msgs::msg::Header const & header);
        void updateDisplay();
        void updateArrows2d();
        void updateArrows3d();
        void updateAxes();
        void updatePoints();


        std::unique_ptr<rviz_rendering::Axes> makeAxes();
        std::unique_ptr<rviz_rendering::Arrow> makeArrow3d();
        std::unique_ptr<rviz_rendering::Shape> makeSphere();
    protected:
        void onInitialize() override;
        void onEnable() override;
        void onDisable() override;
        void processMessage(const crazyflie_interfaces::msg::PoseNamedArray::ConstSharedPtr msg) override;

    private Q_SLOTS:
        void updateShapeChoice();
        void updateShowNames();
    private:
        rviz_common::properties::BoolProperty * rotation_validity_property_;
        rviz_common::properties::BoolProperty * show_names_property_;
        rviz_common::properties::EnumProperty * shape_property_;
        
        std::vector<OgrePose> poses_;
        std::vector<OgrePose> invalid_rotation_poses_;

        std::unique_ptr<rviz_default_plugins::displays::FlatArrowsArray> arrows2d_;
        std::vector<std::unique_ptr<rviz_rendering::Arrow>> arrows3d_;
        std::vector<std::unique_ptr<rviz_rendering::Axes>> axes_;
        std::vector<std::unique_ptr<rviz_rendering::Shape>> points_;

        std::unordered_map<std::string, std::pair<rviz_rendering::MovableText *, Ogre::SceneNode *>> name_texts_;

        Ogre::SceneNode * arrow_node_;
        Ogre::SceneNode * axes_node_;
        Ogre::SceneNode * names_node_;
        Ogre::SceneNode * point_node_;

        std::chrono::nanoseconds update_timer_;
    };


} // namespace crazyflie_interfaces_rviz_displays

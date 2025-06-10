#ifndef OBJECT_INFO_MULTI_ARRAY_DISPLAYER_HPP
#define OBJECT_INFO_MULTI_ARRAY_DISPLAYER_HPP

// C, C++
#include <memory>

// ROS
#include <rviz_common/properties/color_property.hpp>
#include <rviz_rendering/objects/movable_text.hpp>
#include <rviz_rendering/objects/shape.hpp>

// ROS msg
#include <aiformula_interfaces/msg/object_info_multi_array.hpp>

// Original
#include <rviz_aiformula_plugins/base_displayer.hpp>

namespace rviz_aiformula_plugins {

class ObjectInfoMultiArrayDisplayer : public BaseDisplayer<aiformula_interfaces::msg::ObjectInfoMultiArray> {
    Q_OBJECT

private:
    void onInitialize() override;
    void processMessage(const aiformula_interfaces::msg::ObjectInfoMultiArray::ConstSharedPtr msg) override;
    void displayObjectPositions(const std::vector<aiformula_interfaces::msg::ObjectInfo> &objects);
    void displayObjectWidth(const std::vector<aiformula_interfaces::msg::ObjectInfo> &objects);
    void displayObjectId(const std::vector<aiformula_interfaces::msg::ObjectInfo> &objects);
    static Ogre::ColourValue getColorWithConfidence(Ogre::ColourValue color, float confidence);

    std::unique_ptr<rviz_common::properties::ColorProperty> position_color_property_;
    std::unique_ptr<rviz_common::properties::ColorProperty> width_color_property_;
    std::unique_ptr<rviz_common::properties::ColorProperty> id_color_property_;
    std::vector<std::unique_ptr<rviz_rendering::Shape>> position_shapes_;
    std::vector<std::unique_ptr<rviz_rendering::Shape>> width_shapes_;
    std::vector<std::unique_ptr<rviz_rendering::MovableText>> id_texts_;
};

}  // namespace rviz_aiformula_plugins

#endif  // OBJECT_INFO_MULTI_ARRAY_DISPLAYER_HPP

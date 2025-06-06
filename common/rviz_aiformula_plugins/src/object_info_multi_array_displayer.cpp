#include <rviz_aiformula_plugins/object_info_multi_array_displayer.hpp>

namespace rviz_aiformula_plugins {

void ObjectInfoMultiArrayDisplayer::onInitialize() {
    MFDClass::onInitialize();

    position_color_property_ = std::make_unique<rviz_common::properties::ColorProperty>(
        "Position Color", QColor(0, 255, 0), "Color to draw the object position.", this);
    connect(position_color_property_.get(), &rviz_common::properties::ColorProperty::changed, this,
            [this]() { this->updateColor(position_shapes_, position_color_property_.get()); });
    width_color_property_ = std::make_unique<rviz_common::properties::ColorProperty>(
        "Width Color", QColor(0, 150, 0), "Color to draw the object width.", this);
    connect(width_color_property_.get(), &rviz_common::properties::ColorProperty::changed, this,
            [this]() { this->updateColor(width_shapes_, width_color_property_.get()); });
    id_color_property_ = std::make_unique<rviz_common::properties::ColorProperty>("ID Color", QColor(255, 255, 255),
                                                                                  "Color to draw the object id.", this);
    connect(id_color_property_.get(), &rviz_common::properties::ColorProperty::changed, this,
            [this]() { this->updateColor(id_texts_, id_color_property_.get()); });
}

void ObjectInfoMultiArrayDisplayer::processMessage(
    const aiformula_interfaces::msg::ObjectInfoMultiArray::ConstSharedPtr msg) {
    this->updateSceneNodeTransform(msg->header);
    displayObjectPositions(msg->objects);
    displayObjectWidth(msg->objects);
    displayObjectId(msg->objects);
}

void ObjectInfoMultiArrayDisplayer::displayObjectPositions(
    const std::vector<aiformula_interfaces::msg::ObjectInfo> &objects) {
    static const Ogre::Vector3 marker_scale(0.6, 0.6, 0.1);
    const Ogre::ColourValue color = rviz_common::properties::qtToOgre(position_color_property_->getColor());

    position_shapes_.clear();
    for (const auto &object : objects) {
        auto &shape = *position_shapes_.emplace_back(
            std::make_unique<rviz_rendering::Shape>(rviz_rendering::Shape::Type::Cube, scene_manager_, scene_node_));
        shape.setPosition(Ogre::Vector3(object.x, object.y, 0.0));
        shape.setScale(marker_scale);
        shape.setColor(getColorWithConfidence(color, object.confidence));
    }
}

void ObjectInfoMultiArrayDisplayer::displayObjectWidth(
    const std::vector<aiformula_interfaces::msg::ObjectInfo> &objects) {
    const Ogre::ColourValue color = rviz_common::properties::qtToOgre(width_color_property_->getColor());

    width_shapes_.clear();
    for (const auto &object : objects) {
        auto &shape = *width_shapes_.emplace_back(
            std::make_unique<rviz_rendering::Shape>(rviz_rendering::Shape::Type::Cube, scene_manager_, scene_node_));
        shape.setPosition(Ogre::Vector3(object.x, object.y, 0.0));
        shape.setScale(Ogre::Vector3(0.2, object.width, 0.0));
        shape.setColor(getColorWithConfidence(color, object.confidence));
    }
}

void ObjectInfoMultiArrayDisplayer::displayObjectId(const std::vector<aiformula_interfaces::msg::ObjectInfo> &objects) {
    const Ogre::ColourValue color = rviz_common::properties::qtToOgre(id_color_property_->getColor());

    id_texts_.clear();
    for (const auto &object : objects) {
        auto *text =
            id_texts_.emplace_back(std::make_unique<rviz_rendering::MovableText>(std::to_string(object.id))).get();
        text->setTextAlignment(rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_CENTER);
        text->setCharacterHeight(1.0);
        text->setColor(color);

        Ogre::SceneNode *text_node = scene_node_->createChildSceneNode();
        text_node->attachObject(text);
        text_node->setPosition(Ogre::Vector3(object.x - 1.0, object.y, 0.0));
    }
}

Ogre::ColourValue ObjectInfoMultiArrayDisplayer::getColorWithConfidence(Ogre::ColourValue color, float confidence) {
    confidence = std::clamp(confidence, 0.0f, 1.0f);
    color.r *= confidence;
    color.g *= confidence;
    color.b *= confidence;
    return color;
}

}  // namespace rviz_aiformula_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_aiformula_plugins::ObjectInfoMultiArrayDisplayer, rviz_common::Display)

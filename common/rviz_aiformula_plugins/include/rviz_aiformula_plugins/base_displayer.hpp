#ifndef BASE_DISPLAYER_HPP
#define BASE_DISPLAYER_HPP

// ROS
#include <rviz_common/logging.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/parse_color.hpp>

namespace rviz_aiformula_plugins {

template <typename MessageType>
class BaseDisplayer : public rviz_common::MessageFilterDisplay<MessageType> {
protected:
    inline void updateSceneNodeTransform(const std_msgs::msg::Header &header) {
        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        if (!this->context_->getFrameManager()->getTransform(header, position, orientation)) {
            RVIZ_COMMON_LOG_DEBUG_STREAM("Error transforming from frame '" << header.frame_id << "' to frame '"
                                                                           << qPrintable(this->fixed_frame_) << "'");
            return;
        }
        this->scene_node_->setPosition(position);
        this->scene_node_->setOrientation(orientation);
    }

    template <typename DrawableObjectPtr>
    inline void updateColor(std::vector<DrawableObjectPtr> &objects,
                            rviz_common::properties::ColorProperty *color_property) {
        Ogre::ColourValue color = rviz_common::properties::qtToOgre(color_property->getColor());
        for (auto &obj : objects) {
            obj->setColor(color);
        }
    }
};

}  // namespace rviz_aiformula_plugins

#endif  // BASE_DISPLAYER_HPP

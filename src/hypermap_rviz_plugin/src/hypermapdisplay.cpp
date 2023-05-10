#include "hypermapdisplay.h"

#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <OgreImage.h>
#include <OgreManualObject.h>

#include "nav_msgs/OccupancyGrid.h"
#include "hypermap_msgs/SemanticMap.h"

#include "rviz/default_plugin/map_display.h"
#include "semanticmapdisplay.h"

PLUGINLIB_EXPORT_CLASS(hypermap::HypermapDisplay, rviz::Display)

namespace hypermap
{

HypermapDisplay::HypermapDisplay() : rviz::DisplayGroup()
{
    topic_property_ = new rviz::RosTopicProperty("Topic", "", ros::message_traits::datatype<hypermap_msgs::HypermapMetaData>(),
                                                 "hypermap_msgs::HypermapMetaData topic to subscribe to.", this, SLOT(updateTopic()));

    connect(this, SIGNAL(mapReceived()), this, SLOT(updateLayerProps()));
}

void HypermapDisplay::setTopic(const QString &topic, const QString &datatype)
{
    topic_property_->setString(topic);
}

void HypermapDisplay::updateTopic()
{
    map_sub_.shutdown();
    map_sub_ = update_nh_.subscribe(topic_property_->getTopicStd(), 1, &HypermapDisplay::receiveMapMeta, this);
}

QString HypermapDisplay::getDisplayName(const std::string &layerClass)
{
    if (layerClass == "OccupancyGridLayer")
        return "rviz/Map";
    else if (layerClass == "SemanticLayer")
        return "hypermap/SemanticMap";
    else
        return QString::fromStdString(layerClass) + QStringLiteral(" is not a valid layer class");
}

QString HypermapDisplay::getTopicName(const std::string &layerClass, const std::string &layerName)
{
    if (layerClass == "OccupancyGridLayer")
        return QString::fromStdString(current_map_meta_.node_name) + QStringLiteral("/") + QString::fromStdString(layerName) + QStringLiteral("_map");
    else if (layerClass == "SemanticLayer")
        return QString::fromStdString(current_map_meta_.node_name) + QStringLiteral("/") + QString::fromStdString(layerName) + QStringLiteral("_semmap");
    else
        return QString::fromStdString(layerClass) + QStringLiteral(" is not a valid layer class");
}

QString HypermapDisplay::getTopicType(const std::string &layerClass)
{
    if (layerClass == "OccupancyGridLayer")
        return ros::message_traits::datatype<nav_msgs::OccupancyGrid>();
    else if (layerClass == "SemanticLayer")
        return ros::message_traits::datatype<hypermap_msgs::SemanticMap>();
    else
        return QString::fromStdString(layerClass) + QStringLiteral(" is not a valid layer class");
}

void HypermapDisplay::updateLayerProps()
{
    removeAllDisplays();
    for (const hypermap_msgs::LayerMetaData &layerMeta : current_map_meta_.layers)
    {
        Display *disp = createDisplay(getDisplayName(layerMeta.class_name));
        addDisplay(disp);
        disp->setObjectName(QString::fromStdString(layerMeta.name));
        disp->initialize(context_);
        disp->setTopic(getTopicName(layerMeta.class_name, layerMeta.name), getTopicType(layerMeta.class_name));
        disp->setEnabled(true);
    }
}

void HypermapDisplay::receiveMapMeta(const hypermap_msgs::HypermapMetaData::ConstPtr& msg)
{
    current_map_meta_ = *msg;
    ROS_INFO("Map meta received");
    Q_EMIT mapReceived();
}

}

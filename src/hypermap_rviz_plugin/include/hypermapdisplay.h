#ifndef HYPERMAPDISPLAY_H
#define HYPERMAPDISPLAY_H

#include "rviz/display.h"
#include "rviz/display_group.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/bool_property.h"
#include "hypermap_msgs/HypermapMetaData.h"

namespace hypermap
{

class HypermapDisplay : public rviz::DisplayGroup
{
Q_OBJECT
public:
  HypermapDisplay();

  // Overrides from Display
  virtual void setTopic(const QString &topic, const QString &datatype);

Q_SIGNALS:
  void mapReceived();

protected Q_SLOTS:
  void updateTopic();
  void updateLayerProps();

protected:
  void receiveMapMeta(const hypermap_msgs::HypermapMetaData::ConstPtr& msg);

  hypermap_msgs::HypermapMetaData current_map_meta_;
  ros::Subscriber map_sub_;

  rviz::RosTopicProperty *topic_property_;

private:
  QString getDisplayName(const std::string &layerClass);
  QString getTopicName(const std::string &layerClass, const std::string &layerName);
  QString getTopicType(const std::string &layerClass);
};

}
#endif // HYPERMAPDISPLAY_H

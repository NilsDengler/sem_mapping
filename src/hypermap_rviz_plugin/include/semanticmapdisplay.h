#ifndef SEMANTICMAPDISPLAY_H
#define SEMANTICMAPDISPLAY_H

#include <map>
#include <string>

#include "rviz/display.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/bool_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/status_property.h"
#include "hypermap_msgs/SemanticMap.h"

namespace hypermap {

class SemanticMapDisplay : public rviz::Display
{
Q_OBJECT
public:
  SemanticMapDisplay();

  virtual void setTopic(const QString &topic, const QString &datatype);
  virtual void fixedFrameChanged();
  virtual void onEnable();
  virtual void onDisable();

Q_SIGNALS:
  void mapReceived();

protected Q_SLOTS:
  void updateTopic();
  void updateVisual();

protected:
  void receiveMap(const hypermap_msgs::SemanticMap::ConstPtr& msg);
  void updateTransform();
  void subscribe();
  void unsubscribe();
  void clearVisual();

  rviz::RosTopicProperty *topic_property_;
  rviz::BoolProperty *show_polygons_property_;
  rviz::BoolProperty *show_labels_property_;
  rviz::FloatProperty *char_height_property_;

  rviz::Property *show_classes_property_;
  std::map<std::string, int> class_list_;

  hypermap_msgs::SemanticMap current_map_;
  ros::Subscriber map_sub_;
  bool loaded_;
};

}

#endif // SEMANTICMAPDISPLAY_H

#include "semanticmapdisplay.h"

#include <random>
#include <Ogre.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
//#include <rviz/view_manager.h>
//#include <rviz/tool_manager.h>
#include "geometry_msgs/Point32.h"
#include "earcut.hpp"
#include "glasbey.h"

#include "movable_text.h"

namespace mapbox {
namespace util {

template <>
struct nth<0, geometry_msgs::Point32> {
    inline static geometry_msgs::Point32::_x_type get(const geometry_msgs::Point32 &t) {
        return t.x;
    }
};

template <>
struct nth<1, geometry_msgs::Point32> {
    inline static geometry_msgs::Point32::_y_type get(const geometry_msgs::Point32 &t) {
        return t.y;
    }
};

} // namespace util
} // namespace mapbox

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(hypermap::SemanticMapDisplay, rviz::Display)

namespace hypermap
{

SemanticMapDisplay::SemanticMapDisplay() : rviz::Display(), loaded_(false)
{
    topic_property_ = new rviz::RosTopicProperty("Topic", "", ros::message_traits::datatype<hypermap_msgs::SemanticMap>(),
                                                 "hypermap_msgs::SemanticMap topic to subscribe to.", this, SLOT(updateTopic()));

    show_polygons_property_ = new rviz::BoolProperty("Show shapes", true, "Display shapes of semantic objects", this);
    show_labels_property_ = new rviz::BoolProperty("Show labels", true, "Display names of semantic objects", this);
    char_height_property_ = new rviz::FloatProperty("Char height", 0.3, "Char height for labels", this);
    show_classes_property_ = new rviz::Property("Select classes", QVariant(), "Change which classes are shown", this);

    connect(this, SIGNAL(mapReceived()), this, SLOT(updateVisual()));
    connect(show_polygons_property_, SIGNAL(changed()), this, SLOT(updateVisual()));
    connect(show_labels_property_, SIGNAL(changed()), this, SLOT(updateVisual()));
    connect(char_height_property_, SIGNAL(changed()), this, SLOT(updateVisual()));
}

void SemanticMapDisplay::setTopic(const QString &topic, const QString &datatype)
{
    topic_property_->setString(topic);
}

void SemanticMapDisplay::updateTopic()
{
    unsubscribe();
    clearVisual();
    class_list_.clear();
    show_classes_property_->removeChildren();
    subscribe();
}

void SemanticMapDisplay::subscribe()
{
    if (!isEnabled())
        return;

    map_sub_.shutdown();

    if(!topic_property_->getTopic().isEmpty())
    {
        try
        {
            map_sub_ = update_nh_.subscribe(topic_property_->getTopicStd(), 1, &SemanticMapDisplay::receiveMap, this);
            setStatus(rviz::StatusProperty::Ok, "Topic", "OK");
        }
        catch (ros::Exception &e)
        {
            setStatus(rviz::StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
        }
    }

    /*Ogre::ManualObject *mo = scene_manager_->createManualObject("test");

    mo->begin("BaseWhite", Ogre::RenderOperation::OT_TRIANGLE_FAN);
    mo->position(0,0,0);
    mo->position(1,0,0);
    mo->position(1,1,0);
    mo->position(0,1,0);
    mo->end();

    scene_node_->attachObject(mo);*/

    /*Ogre::Polygon *po = new Ogre::Polygon();

    po->insertVertex(Ogre::Vector3(0, 0, 0));
    po->insertVertex(Ogre::Vector3(0, 2, 0));
    po->insertVertex(Ogre::Vector3(2, 2, 0));
    po->insertVertex(Ogre::Vector3(2, 0, 0));

    scene_node_->addChild(po);*/
}

void SemanticMapDisplay::unsubscribe()
{
    map_sub_.shutdown();
    loaded_ = false;
    setStatus(rviz::StatusProperty::Warn, "Message", "No map received");
}

void SemanticMapDisplay::onEnable()
{
    subscribe();
}

void SemanticMapDisplay::onDisable()
{
    unsubscribe();
    clearVisual();
}

void SemanticMapDisplay::updateTransform()
{
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if(!context_->getFrameManager()->getTransform(current_map_.header.frame_id, ros::Time(0), position, orientation))
    {
        ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", current_map_.header.frame_id.c_str(), qPrintable(fixed_frame_));
    }

    scene_node_->setPosition(position);
    scene_node_->setOrientation(orientation);
}

void SemanticMapDisplay::clearVisual()
{
    scene_node_->detachAllObjects();
    scene_node_->removeAndDestroyAllChildren();
}

void SemanticMapDisplay::updateVisual()
{
    clearVisual();

    if (!loaded_)
    {
        return;
    }

    /*Ogre::ManualObject *mob = scene_manager_->createManualObject("test");

    mob->begin("BaseWhite", Ogre::RenderOperation::OT_TRIANGLE_FAN);
    mob->position(0,0,0);
    mob->position(1,0,0);
    mob->position(1,1,0);
    mob->position(0,1,0);
    mob->end();

    scene_node_->attachObject(mob);*/

    /*hypermap::MovableText *testTxt = new hypermap::MovableText("test text", "Liberation Sans", 0.3);
    testTxt->setTextAlignment(hypermap::MovableText::H_CENTER, hypermap::MovableText::V_CENTER);
    testTxt->setGlobalTranslation(Ogre::Vector3(3, 4, 5));
    scene_node_->attachObject(testTxt);*/

    updateTransform();

    //std::default_random_engine generator;
    //std::uniform_real_distribution<float> distribution(0.0,1.0);

    //uint8_t cind = 2;

    for (const auto &obj : current_map_.objects)
    {
        auto c_it = class_list_.find(obj.name);
        if (c_it == class_list_.end())
        {
            rviz::BoolProperty *prop = new rviz::BoolProperty(QString::fromStdString(obj.name), true, "Show class", show_classes_property_);
            std::tie(c_it, std::ignore) = class_list_.insert(std::make_pair(obj.name, class_list_.size()));
            connect(prop, SIGNAL(changed()), this, SLOT(updateVisual()));
        }
        else
        {
            rviz::BoolProperty *prop = (BoolProperty*) show_classes_property_->childAt(c_it->second);
            if (!prop->getBool())
                continue;
        }

        if (show_polygons_property_->getBool())
        {
            std::vector<std::vector<geometry_msgs::Point32>> pg;
            pg.push_back(obj.shape.points);
            std::vector<uint32_t> indices = mapbox::earcut(pg);
            //ROS_INFO_STREAM("Inds : " << indices.size());
            Ogre::ManualObject *mo = scene_manager_->createManualObject();
            //Ogre::ColourValue col(glasbey[cind][0] / 255.0, glasbey[cind][1] / 255.0, glasbey[cind][2] / 255.0);
            //cind = (cind + 1) % 256;
            uint8_t cind = (2 + c_it->second) % 256;
            Ogre::ColourValue col(glasbey[cind][0] / 255.0, glasbey[cind][1] / 255.0, glasbey[cind][2] / 255.0);

            mo->estimateVertexCount(obj.shape.points.size());
            mo->estimateIndexCount(indices.size());
            mo->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
            bool shape_valid = true;
            for (const auto &point : obj.shape.points)
            {
                if (!std::isfinite(point.x) || !std::isfinite(point.y))
                {
                    ROS_WARN_STREAM("Invalid shape detected");
                    shape_valid = false;
                    break;
                }
                mo->position(point.x, point.y, 0);
                mo->colour(col);
                //ROS_INFO_STREAM("Point added: " <<  point.x << ", " << point.y);
            }
            if (!shape_valid)
            {
                delete mo;
                continue;
            }

            for (uint32_t ind : indices)
            {
                mo->index(ind);
            }
            mo->end();
            //double mo_prio = 1.0 / mo->getBoundingRadius();
            //ROS_INFO_STREAM("Prio: " << mo_prio);
            //ushort mo_prio_sh = (ushort)(mo_prio * 1000);
            //mo->setRenderQueueGroupAndPriority(0, mo_prio_sh);
            scene_node_->attachObject(mo);
        }

        if (show_labels_property_->getBool())
        {
            if (!std::isfinite(obj.position.x) || !std::isfinite(obj.position.y))
            {
                ROS_WARN_STREAM("Invalid position detected");
                continue;
            }
            //Ogre::ColourValue col(glasbey[cind][0] / 255.0, glasbey[cind][1] / 255.0, glasbey[cind][2] / 255.0);
            //cind++;
            hypermap::MovableText *mo_txt = new hypermap::MovableText(obj.name, "Liberation Sans", char_height_property_->getFloat()/*, col*/);
            mo_txt->setTextAlignment(hypermap::MovableText::H_CENTER, hypermap::MovableText::V_CENTER);
            mo_txt->setLocalTranslation(Ogre::Vector3(obj.position.x, obj.position.y, 0));
            mo_txt->showOnTop();
            //mo_txt->setRenderQueueGroup(1);
            scene_node_->attachObject(mo_txt);
        }
    }
}

void SemanticMapDisplay::fixedFrameChanged()
{
    updateTransform();
}

void SemanticMapDisplay::receiveMap(const hypermap_msgs::SemanticMap::ConstPtr& msg)
{
    current_map_ = *msg;
    loaded_ = true;
    setStatus(rviz::StatusProperty::Ok, "Message", "Map received");
    Q_EMIT mapReceived();
}

}

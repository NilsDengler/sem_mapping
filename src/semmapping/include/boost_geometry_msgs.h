#ifndef BOOST_GEOMETRY_MSGS_H
#define BOOST_GEOMETRY_MSGS_H

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/arithmetic/arithmetic.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Polygon.h"

namespace semmapping
{

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

//typedef bg::model::point<double, 2, bg::cs::cartesian> point;
typedef bg::model::d2::point_xy<double> point;
typedef bg::model::box<point> box;
typedef bg::model::polygon<point> polygon;
typedef bg::model::multi_polygon<polygon> multi_polygon;
typedef bg::model::multi_point<point> multi_point;
typedef bg::model::ring<point> ring;

typedef boost::shared_ptr<polygon> polygon_ptr;
typedef boost::shared_ptr<polygon const> polygon_const_ptr;

inline geometry_msgs::Point boostToPointMsg(const point &p)
{
    geometry_msgs::Point pm;
    pm.x = p.x(); //p.get<0>();
    pm.y = p.y(); //p.get<1>();
    return pm;
}

inline geometry_msgs::Point32 boostToPoint32Msg(const point &p)
{
    geometry_msgs::Point32 pm;
    pm.x = p.x(); //get<0>();
    pm.y = p.y(); //get<1>();
    return pm;
}

inline geometry_msgs::Polygon boostToPolygonMsg(const polygon &pg)
{
    geometry_msgs::Polygon pgm;

    for (size_t i = 0; i < pg.outer().size() - 1; i++)
        pgm.points.push_back(boostToPoint32Msg(pg.outer()[i]));

    //for (const auto &p : pg.outer())
    //    pgm.points.push_back(boostToPoint32Msg(p));

    return pgm;
}

inline point pointMsgToBoost(const geometry_msgs::Point &pm)
{
    return point(pm.x, pm.y);
}

inline point point32MsgToBoost(const geometry_msgs::Point32 &pm)
{
    return point(pm.x, pm.y);
}

inline polygon polygonMsgToBoost(const geometry_msgs::Polygon &pgm)
{
    polygon pg;
    for (const auto &p : pgm.points)
      bg::append(pg.outer(), point32MsgToBoost(p));

    bg::correct(pg);
    return pg;
}

inline point pclToBoost(const pcl::PointXYZ &p)
{
    return point(p.x, p.y);
}

//inline point pclToBoost(const pcl::PointXY &p)
//{
//    return point(p.x, p.y);
//}

inline pcl::PointXYZ boostToPcl(const point &p)
{
    return pcl::PointXYZ(p.x(), p.y(), 0);
}

//inline pcl::PointXY boostToPclXY(const point &p)
//{
//    return pcl::PointXY{(float)p.x(), (float)p.y()};
//}

inline polygon pclToBoost(const pcl::PointCloud<pcl::PointXYZ> &pc)
{
    polygon pg;
    for (const pcl::PointXYZ &p : pc)
        bg::append(pg.outer(), pclToBoost(p));

    bg::correct(pg);
    return pg;
}

//inline polygon pclToBoost(const pcl::PointCloud<pcl::PointXY> &pc)
//{
//    polygon pg;
//    for (const pcl::PointXY &p : pc)
//        bg::append(pg.outer(), pclToBoost(p));
//
//    bg::correct(pg);
//    return pg;
//}


}

#endif // BOOST_GEOMETRY_MSGS_H

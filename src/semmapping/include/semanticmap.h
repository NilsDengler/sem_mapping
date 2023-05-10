#ifndef SEMANTICMAP_H
#define SEMANTICMAP_H

#include "boost_geometry_msgs.h"
#include <mapping_msgs/SemanticMap.h>
#include <mapping_msgs/FindObjects.h>
#include <mapping_msgs/ObjectPositions.h>
#include <pcl/surface/convex_hull.h>
#include <tf2_ros/buffer.h>
#include <queue>
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/model_outlier_removal.h>
#include <semmapping/ParamsConfig.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <chrono>
#include <pcl/surface/concave_hull.h>
namespace semmapping
{

const size_t MAX_SHAPES = 5; // maximum number of differnt shapes kept in object
const int MIN_CERTAINTY = 10;
const double FIT_THRESH = 0.3; // threshold to consider new shape as evidence
const double MIN_FIT = 0.8; // minmal fit for shape to be considered same

struct UncertainShape
{
    polygon shape;
    point centroid;
    double certainty;
};

struct SemanticObject
{
  std::string name;
  std::vector<UncertainShape> shapes;
  double exist_certainty = 0;
  double mean_height;
  double rotation_angle = 0;
  polygon shape_union;
  point shape_union_cen;
  point centroid_sum;
  point centroid_sum_sq;
  point centroid_mean;
  box bounding_box;
  std::queue<int> counting_queue;
  box oriented_box;
  polygon obb;
  point oriented_box_cen;
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud;
  int times_merged = 0;
  int pushed_to_queue = 0;
  bool isCombined = false;
  //std::vector<std::string> tags;
  //std::vector<double> confidence;
};

class SemanticMap
{
  typedef std::pair<box, size_t> rtree_entry;
  std::vector<size_t> added_objects;
  size_t next_index = 0;
  bgi::rtree< rtree_entry, bgi::rstar<16> > objectRtree;
  std::map<size_t, SemanticObject> objectList;
  std::set<size_t> tableList;


  inline static double ref_fit(const polygon &newpg, const polygon &refpg)
  {
      multi_polygon sect;
      bg::intersection(newpg, refpg, sect);
      return bg::area(sect) / bg::area(refpg);
  }

/*  inline static double union_fit(const polygon &a, const polygon &b)
  {
      multi_polygon un;
      bg::union_(a, b, un);
      multi_polygon sect;
      bg::intersection(a, b, sect);
      std::cout << "intersect: " << bg::area(sect) << " unioin: " <<bg::area(un) << std::endl;
      //intersection over union
      return bg::area(sect) / bg::area(un);
  }*/

    inline static double union_fit(const polygon &a, const polygon &b)
    {
        double smallest_area;
        multi_polygon un;
        bg::union_(a, b, un);
        if(bg::area(a)>bg::area(b))  smallest_area=bg::area(b);
        else   smallest_area=bg::area(a);
        multi_polygon sect;
        bg::intersection(a, b, sect);
        std::cout << "intersect: " << bg::area(sect) << " unioin: " <<bg::area(un) << std::endl;
        //intersection over union
        return bg::area(sect) / smallest_area;
    }

  inline static point point_square(const point &p)
  {
      return point(p.x()*p.x(), p.y()*p.y());
  }

  inline static point get_mean(const point &sum, size_t n)
  {
      return point(sum.x()/n, sum.y()/n);
  }

  inline static point get_std(const point &sum, const point &sum_sq, size_t n)
  {
      return point((sum_sq.x() - sum.x()*sum.x()/n) / n, (sum_sq.y() - sum.y()*sum.y()/n) / n);
  }

  inline static void addToMean(point &mean, const point &toAdd, size_t n)
  {
      point tmp = toAdd;
      bg::subtract_point(tmp, mean);
      bg::divide_value(tmp, n);
      bg::add_point(mean, tmp);
  }

  inline static void removeFromMean(point &mean, const point &toRemove, size_t n)
  {
      point tmp = mean;
      bg::subtract_point(tmp, toRemove);
      bg::divide_value(tmp, n);
      bg::add_point(mean, tmp);
  }

  inline static box getSearchBox(const polygon &pg)
  {
      point centroid;
      bg::centroid(pg, centroid);
      //double searchRadius = bg::perimeter(pg) / 8;
      //double searchRadius = std::sqrt(bg::area(pg));
      //if (searchRadius < 1)
      //    searchRadius = 1;
      double searchRadius = 0.5;

      return box(point(centroid.x() - searchRadius, centroid.y() - searchRadius), point(centroid.x() + searchRadius, centroid.y() + searchRadius));
  }

  inline static void addToPointCloud(multi_point &cloud, const polygon &pg)
  {
      for (const point &p : pg.outer())
      {
          cloud.push_back(p);
      }
  }

  inline static void addToPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const polygon &pg)
  {
      for (const point &p : pg.outer())
      {
          cloud->push_back(boostToPcl(p));
      }
  }

  polygon computeConvexHullPcl(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
  {

      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
      coefficients->values.resize (4);
      coefficients->values[0] = coefficients->values[1] = 0;
      coefficients->values[2] = 1.0;
      coefficients->values[3] = 0;

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::ProjectInliers<pcl::PointXYZ> proj;
      proj.setModelType (pcl::SACMODEL_PLANE);
      proj.setModelCoefficients (coefficients);
      proj.setInputCloud (cloud);
      proj.filter (*cloud_projected);

//      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
//      pcl::VoxelGrid<pcl::PointXYZ> sor;
//      sor.setInputCloud (cloud_projected);
//      sor.setLeafSize (0.02f, 0.02f, 0.02f);
//      sor.filter (*cloud_filtered);
      if (cloud_projected->points.size() == 0){
          polygon pg;
          return pg;
      }
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
      //pcl::ConcaveHull<pcl::PointXYZ> chull;
      //chull.setAlpha(0.5);
      pcl::ConvexHull<pcl::PointXYZ> chull;
      chull.setInputCloud (cloud_projected);
      chull.setDimension(2);
      chull.reconstruct (*cloud_hull);
      return pclToBoost(*cloud_hull);
  }
private:
    tf2_ros::Buffer &tfBuffer;
    pcl::visualization::PCLVisualizer* &viewer;
    boost::mutex &viewer_mtx;
    int &semmap_vport0;
    int &semmap_vport1;


public:
  SemanticMap(tf2_ros::Buffer &tfBuffer, pcl::visualization::PCLVisualizer* &viewer, boost::mutex &viewer_mtx, int &semmap_vport0, int &semmap_vport1, semmapping::ParamsConfig &param_config );

  void addEvidence(const std::string &name, const polygon &pg, double mean_height,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  void removeEvidence(const polygon &visibilityArea, const point &robot);

  //int deleteLeastConsistentShape(size_t id);
  void deleteLeastConsistentShape(size_t id);
  //void cleanupShapes(SemanticObject &obj);
  void updateUnion(size_t id);
  void filterIntersectionThresh(std::set<size_t> &object_list, const polygon &pg);
  int findFittingExistingShape(std::vector<UncertainShape> &shapes, const polygon &pg);
  void addNewObject(const std::string &name, const polygon &initial_shape, double &mean_height, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  void addObject(const SemanticObject &obj);
  void removeObject(size_t id);
  void clearAll();
  point getRobotPosition();
  float robotIsToClose(point object);
  pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliers(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr removeRadiusOutliers(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr calculateIcp(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr calculateAlignment(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2);
  size_t combineObjects(std::set<size_t> objects);
  std::set<size_t> getObjectsInRange(const polygon &pg);
  std::set<size_t> getObjectsWithinRange(const polygon &pg);
  std::set<size_t> getObjectsByNameInRange(const std::string &name, const polygon &pg);
  std::set<size_t> getObjectsByNameInRange(const std::string &name, const box &bx);
  box create_oriented_box(polygon poly, double &best_angle);
  polygon polygonFromBox(const box &bbox, const double &angle);
  std::vector<pcl::PointCloud<pcl::PointXYZ>>getObjectPointsEuc(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
  std::vector<pcl::PointCloud<pcl::PointXYZ>>getObjectPointsReg(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
  mapping_msgs::SemanticMap::Ptr createMapMessage(const point &robot,double loaded);
  mapping_msgs::ObjectPositions::Ptr findObjectPosition(const mapping_msgs::FindObjects &request);
  bool writeMapData(std::ostream &output);
  bool writeLikelihoodData(std::ostream &output);
  bool readMapData(std::istream &input);

  int viewer_index = 0;
    semmapping::ParamsConfig &param_config;
    std::chrono::duration<double,std::milli> InRange_time;
    double InRange_time_processed;
    double avg_InRange_time;

  double avg_pc_size=0;

  std::vector<double> likelihood_value_chair;
  std::vector<double> time_value_chair;
  std::vector<double> likelihood_value_table;
  std::vector<double> time_value_table;

};

}

#endif // SEMANTICMAP_H

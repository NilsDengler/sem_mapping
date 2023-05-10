#include "semanticmap.h"
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <pcl/features/normal_3d.h>

namespace semmapping
{

    SemanticMap::SemanticMap(tf2_ros::Buffer &tfBuffer, pcl::visualization::PCLVisualizer* &viewer, boost::mutex &viewer_mtx, int &semmap_vport0, int &semmap_vport1, semmapping::ParamsConfig &param_config): tfBuffer(tfBuffer), viewer(viewer), viewer_mtx(viewer_mtx), semmap_vport0(semmap_vport0), semmap_vport1(semmap_vport1), param_config(param_config)
{
}

void SemanticMap::filterIntersectionThresh(std::set<size_t> &object_list, const polygon &pg)
{
    ROS_INFO("Filtering objects");
    for (auto it = object_list.begin(); it != object_list.end(); )
    {
        const SemanticObject &obj = objectList.at(*it);
        if (union_fit(pg, obj.shape_union) < FIT_THRESH)// || bg::covered_by(obj.shape_union,pg))
        {
            it = object_list.erase(it);
            ROS_INFO("Object removed");
            continue;
        }
        it++;
    }
}

box SemanticMap::create_oriented_box(polygon poly, double &best_angle){
    box best_box;
    double minArea = DBL_MAX;
    double minAngle = DBL_MAX;
    point current_edge = poly.outer()[0];
    for (int i=0; i < poly.outer().size()-1;i++){
        point next_edge = poly.outer()[i+1];
        box temp_box;
        polygon temp_polygon;
        double angle = atan2(next_edge.y()-current_edge.y(),next_edge.x() - current_edge.x());
        bg::strategy::transform::rotate_transformer<boost::geometry::degree, double, 2, 2> rotate(-1.0 * angle * (180/ M_PI));
        bg::transform(poly, temp_polygon, rotate);
        temp_box = bg::return_envelope<box>(temp_polygon);
        if (bg::area(temp_box) < minArea) {
            minArea = bg::area(temp_box);
            best_box = temp_box;
            minAngle = angle;
        }
        current_edge = next_edge;
    }
    best_angle = minAngle;
    return best_box;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudFromInd(pcl::PointIndices::Ptr input_indices,  pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud){
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        typename pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setInputCloud(input_cloud);
        extract.setIndices(input_indices);
        extract.filter(*result_cloud);
        return result_cloud;
    }

std::vector<pcl::PointCloud<pcl::PointXYZ>> SemanticMap::getObjectPointsReg(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
    {
        pcl::PointCloud<pcl::PointXYZ> return_cloud;
        std::vector<pcl::PointIndices> cluster_indices;
        std::vector<pcl::PointCloud<pcl::PointXYZ>> point_cloud_cluster;
        pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod (tree);
        normal_estimator.setInputCloud (cloud);
        normal_estimator.setKSearch (50);
        normal_estimator.compute (*normals);


        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize (50);
        reg.setMaxClusterSize (1000000);
        reg.setSearchMethod (tree);
        reg.setNumberOfNeighbours (30);
        reg.setInputCloud (cloud);

        reg.setInputNormals (normals);
        reg.setSmoothnessThreshold (20.0 / 180.0 * M_PI);
        reg.setCurvatureThreshold (1.0);

        std::vector <pcl::PointIndices> clusters;
        reg.extract (cluster_indices);

        if (cluster_indices.size() == 0)
        {
            ROS_WARN("No points in cluster indicies");
            return_cloud.points = cloud->points;
            point_cloud_cluster.push_back(return_cloud);
        }
        for(int i=0;i<cluster_indices.size();i++){
            pcl::PointIndices::Ptr temp_indices(new pcl::PointIndices);
            temp_indices->indices = cluster_indices[i].indices;
            point_cloud_cluster.push_back(*pointCloudFromInd(temp_indices,cloud));
        }
        return point_cloud_cluster;
    }



    std::vector<pcl::PointCloud<pcl::PointXYZ>> SemanticMap::getObjectPointsEuc(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
    {

        std::vector<pcl::PointCloud<pcl::PointXYZ>> point_cloud_cluster;
        pcl::PointCloud<pcl::PointXYZ> return_cloud;
        std::vector<pcl::PointIndices> cluster_indices;


        if (cloud->points.size() == 0)
        {
            ROS_WARN("No points in cloud before Euclead and tree");
            return_cloud.points = cloud->points;
            point_cloud_cluster.push_back(return_cloud);
            return point_cloud_cluster;
        }
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud);

        //Do eucledean clustering
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.1);
        ec.setMinClusterSize (double(100));
        ec.setMaxClusterSize (cloud->points.size());
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud);
        ec.extract (cluster_indices);

        if (cluster_indices.size() == 0)
        {
            ROS_WARN("No points in cluster indicies");
            return_cloud.points = cloud->points;
            point_cloud_cluster.push_back(return_cloud);
        }
        for(int i=0;i<cluster_indices.size();i++){
            pcl::PointIndices::Ptr temp_indices(new pcl::PointIndices);
            temp_indices->indices = cluster_indices[i].indices;
            point_cloud_cluster.push_back(*pointCloudFromInd(temp_indices,cloud));
        }
        return point_cloud_cluster;
    }

double calculateMeanHight(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud){
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*input_cloud,centroid);
        return centroid[2];
    }
// Combine shapes
void SemanticMap::updateUnion(size_t id)
{
    SemanticObject &obj = objectList.at(id);
    //ROS_INFO("Calculating union");
    if (obj.shapes.size() < 1)
    {
        ROS_ERROR("Semantic object has no shape");
        return;
    }
    if (obj.times_merged > 5){
        std::vector<pcl::PointCloud<pcl::PointXYZ>> point_cloud_cluster;
        if (!obj.point_cloud->empty()){
//            obj.point_cloud = removeOutliers(obj.point_cloud);
            obj.times_merged = 0;
            point_cloud_cluster = getObjectPointsEuc(obj.point_cloud);
            *obj.point_cloud = point_cloud_cluster[0];
//            if(point_cloud_cluster.size()==1){
//                *obj.point_cloud = point_cloud_cluster[0];
//            }
//            else if (point_cloud_cluster.size()>1){
//                *obj.point_cloud = point_cloud_cluster[0];
//                for(int i=1;i<point_cloud_cluster.size();i++){
//                    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//                    *temp_cloud = point_cloud_cluster[i];
//                    double height = calculateMeanHight(temp_cloud);
//                    addNewObject(obj.name,computeConvexHullPcl(temp_cloud),height,temp_cloud);
//                }
//            }
        }
    }
    //multi_point obj_cloud;
    double biggest_area_size = 0;
    double angle;
    obj.isCombined = true;
    obj.shape_union = computeConvexHullPcl(obj.point_cloud);
    obj.oriented_box = create_oriented_box(obj.shape_union, angle);
    obj.rotation_angle = angle;
    obj.obb = polygonFromBox(obj.oriented_box, obj.rotation_angle);
    bg::centroid(obj.obb, obj.oriented_box_cen);
    bg::centroid( obj.shape_union, obj.shape_union_cen);

    objectRtree.remove(std::make_pair(obj.bounding_box, id));
    obj.bounding_box = bg::return_envelope<box>(obj.shape_union);
    objectRtree.insert(std::make_pair(obj.bounding_box, id));
}

size_t SemanticMap::combineObjects(std::set<size_t> objects)
    {
        ROS_INFO_STREAM("Combining " << objects.size() << " objects");
        auto it = objects.begin();
        size_t combinedId = *it;
        it++;
        SemanticObject &combinedObj = objectList.at(combinedId);
        //pcl::VoxelGrid<pcl::PointXYZ> vox;
        for (; it != objects.end(); it++)
        {

            SemanticObject &toMerge = objectList.at(*it);
            *combinedObj.point_cloud += *toMerge.point_cloud;
            combinedObj.mean_height = double(combinedObj.mean_height+toMerge.mean_height)/2.0;
            if (toMerge.counting_queue.size() > combinedObj.counting_queue.size())
                combinedObj.counting_queue = toMerge.counting_queue;

            removeObject(*it);
        }
        updateUnion(combinedId);
        return combinedId;
    }

// V1
void SemanticMap::addEvidence(const std::string &name, const polygon &pg, double mean_height, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    /*check if polygon lies on table and filter table plan*/
        std::set<size_t> existingObjects = getObjectsByNameInRange(name, pg);


        //filter all neighbors with IoU < 0.2
        //filterIntersectionThresh(existingObjects, pg);
        if (existingObjects.empty()) {
            // no object evidence exists yet, create new
            ROS_INFO("No Neighbors fitting, Create new object");
            addNewObject(name, pg, mean_height, cloud);
        } else {
            //ROS_INFO_STREAM("Number of fitting objects: " << existingObjects.size());
            size_t objectId = *existingObjects.begin();
            // if more than one object fits, combine objects

            if (existingObjects.size() > 1) {
                objectId = combineObjects(existingObjects);
            }

            // combine combinedShape with new one
            SemanticObject &obj = objectList.at(objectId);
            point shape_centroid;
            obj.mean_height = (obj.mean_height + mean_height) / 2;
            *obj.point_cloud += *cloud;

            pcl::VoxelGrid <pcl::PointXYZ> vox;
            vox.setInputCloud(obj.point_cloud);
            vox.setLeafSize(0.02, 0.02, 0.02);
            vox.setSaveLeafLayout(true);
            vox.filter(*obj.point_cloud);

            //if queue size is > 30 pop first entry and push 1
            if (obj.counting_queue.size() > param_config.queue_size) obj.counting_queue.pop();
            obj.counting_queue.push(1);
            obj.times_merged++;

            updateUnion(objectId);

        }

    pcl::PassThrough<pcl::PointXYZ> pass;
    for (size_t table_id : tableList){
        SemanticObject &table = objectList.at(table_id);
        std::set<size_t> tableObjects = getObjectsInRange(table.shape_union);
        for (size_t id : tableObjects)
        {
            SemanticObject &tableObj = objectList.at(id);
            if (tableObj.name == "Table"){
                continue;
            }
            pass.setInputCloud (tableObj.point_cloud);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (table.mean_height-0.20, table.mean_height+0.02);
            pass.setNegative (true);
            pass.filter(*tableObj.point_cloud);
            if(tableObj.point_cloud->points.size() < 5)
            {
                ROS_INFO_STREAM("Object " << tableObj.name << " removed beacause it was in table points");
                removeObject(id);
            }
            else{
                double angel;
                tableObj.shape_union = computeConvexHullPcl(tableObj.point_cloud);
                tableObj.oriented_box = create_oriented_box(tableObj.shape_union, angel);
                tableObj.rotation_angle = angel;
                tableObj.obb = polygonFromBox(tableObj.oriented_box, tableObj.rotation_angle);}
        }
    }
}

double calculateCertainty(std::queue<int> counting_queue){
    int hit = 0;
    int miss = 0;
    std::queue <int> temp = counting_queue;
    while (!temp.empty())
    {
        if (temp.front() == 1) hit+=1;
        else if(temp.front() == 0) miss+=1;
        temp.pop();
    }
    return (double)hit/(double)(hit+ miss + (1./counting_queue.size()));
}

void SemanticMap::removeEvidence(const polygon &visibilityArea, const point &robot)
{
    std::set<size_t> existingObjects = getObjectsWithinRange(visibilityArea);
    for (size_t id : existingObjects)
    {
        SemanticObject &obj = objectList.at(id);
        point cen_point;
        bg::centroid(obj.shape_union, cen_point);
        double dist_to_obj = bg::distance(robot, cen_point);
        if(dist_to_obj >=  2.8 && obj.isCombined == false) continue;
        if (obj.isCombined == true) {
            obj.exist_certainty = calculateCertainty(obj.counting_queue);
            }
        else {
            //if(obj.counting_queue.size() >=  param_config.queue_thresh) {
            if (obj.counting_queue.size() > param_config.queue_size) obj.counting_queue.pop();
            obj.counting_queue.push(0);
            obj.exist_certainty = calculateCertainty(obj.counting_queue);
        }
        obj.isCombined = false;
        if((dist_to_obj >=  0.2 && dist_to_obj <=  1.8) && obj.counting_queue.size() >= param_config.queue_thresh && obj.exist_certainty < 0.15) {
            //if (obj.exist_certainty < 0.1){//} param_config.certainty_thresh) {
            removeObject(id);
            ROS_INFO_STREAM("Object " << obj.name << " removed");
        }
    }
}

void SemanticMap::addNewObject(const std::string &name, const polygon &initial_shape, double &mean_height,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    SemanticObject obj;
    obj.name = name;
    point shape_centroid;
    bg::centroid(initial_shape, shape_centroid);
    obj.shapes.push_back({initial_shape, shape_centroid, 1});
    obj.counting_queue.push(1);
    obj.isCombined = true;
    obj.exist_certainty = 1;
    obj.shape_union = initial_shape;
    obj.centroid_sum = shape_centroid;
    obj.centroid_sum_sq = point_square(shape_centroid);
    obj.centroid_mean = shape_centroid;
    obj.bounding_box = bg::return_envelope<box>(obj.shape_union);
    obj.mean_height = mean_height;
    obj.point_cloud = cloud;
    bg::centroid(obj.shape_union, obj.shape_union_cen);

    double angle;
    obj.oriented_box = create_oriented_box(obj.shape_union, angle);
    obj.rotation_angle = angle;
    obj.obb = polygonFromBox(obj.oriented_box, obj.rotation_angle);
    bg::centroid(obj.obb, obj.oriented_box_cen);
    //std::cout << "obj obb 2 size: " << obj.obb.outer().size() << std::endl;

    if (obj.name == "Table"){
        tableList.insert(next_index);
    }
    ROS_INFO_STREAM("Adding object" << obj.name << "to map");
    objectList[next_index] = obj;
    rtree_entry ent = {obj.bounding_box, next_index};
    objectRtree.insert(ent);
    next_index++;
}

void SemanticMap::addObject(const SemanticObject &obj)
{
    objectList[next_index] = obj;
    rtree_entry ent = {obj.bounding_box, next_index};
    objectRtree.insert(ent);
    //ROS_INFO_STREAM("Succesfully added, size: " << objectRtree.size());
    next_index++;
}

void SemanticMap::removeObject(size_t id)
{
    SemanticObject &obj = objectList.at(id);
    objectRtree.remove(std::make_pair(obj.bounding_box, id));
    objectList.erase(id);
    if (tableList.find(id) != tableList.end()){
        tableList.erase(id);
    }
}

void SemanticMap::clearAll()
{
    objectList.clear();
    objectRtree.clear();
    next_index = 0;
}

std::set<size_t> SemanticMap::getObjectsWithinRange(const polygon &pg)
{
    double coveredAreaPercentage;
    std::vector<rtree_entry> result_obj;
    std::set<size_t> result;
    objectRtree.query(bgi::intersects(pg), std::back_inserter(result_obj));
    for (rtree_entry val : result_obj)
    {
        multi_polygon sect;
        const SemanticObject &foundObject = objectList.at(val.second);
        bg::intersection(pg, foundObject.shape_union, sect);
        coveredAreaPercentage = bg::area(sect)/bg::area(foundObject.shape_union);
        if(coveredAreaPercentage > 0.5)
            result.insert(val.second);
    }
    return result;
}

std::set<size_t> SemanticMap::getObjectsByNameInRange(const std::string &name, const polygon &pg)
{
    std::vector<rtree_entry> result_obj;
    std::set<size_t> result;
    objectRtree.query(bgi::intersects(pg), std::back_inserter(result_obj));
    for (rtree_entry val : result_obj)
    {
        const SemanticObject &foundObject = objectList.at(val.second);
        if (name == "Couch"){
            if ((foundObject.name == name || foundObject.name == "Chair") && (bg::intersects(pg, foundObject.shape_union)
            || bg::within(pg, foundObject.shape_union) || bg::touches(pg, foundObject.shape_union)))
                result.insert(val.second);
        }
        else{
            if (foundObject.name == name && (bg::intersects(pg, foundObject.shape_union)
            || bg::within(pg, foundObject.shape_union) || bg::touches(pg, foundObject.shape_union)))
                result.insert(val.second);
        }
    }
    return result;
}

point SemanticMap::getRobotPosition(){
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transform;
        try{
            transform = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
        }
        catch (tf2::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        point robot;
        robot.x(transform.transform.translation.x);
        robot.y(transform.transform.translation.y);
        return robot;
    }




    std::set<size_t> SemanticMap::getObjectsInRange(const polygon &pg)
    {
        std::vector<rtree_entry> result_obj;
        std::set<size_t> result;
        objectRtree.query(bgi::intersects(pg), std::back_inserter(result_obj));
        for (rtree_entry val : result_obj)
        {
            const SemanticObject &foundObject = objectList.at(val.second);
            if (bg::intersects(pg, foundObject.shape_union) || bg::within(pg, foundObject.shape_union))
                result.insert(val.second);
        }
        return result;
    }

polygon SemanticMap::polygonFromBox(const box &bbox, const double &angle){
    semmapping::point min;
    semmapping::point max;
    semmapping::point maxmin;
    semmapping::point minmax;

    maxmin.x(bbox.max_corner().x()); maxmin.y(bbox.min_corner().y());
    minmax.x(bbox.min_corner().x()); minmax.y(bbox.max_corner().y());

    bg::strategy::transform::rotate_transformer<boost::geometry::degree, double, 2, 2> rotate(angle * (180/M_PI));
    bg::transform(bbox.min_corner(), min, rotate);
    bg::transform(bbox.max_corner(), max, rotate);
    bg::transform(minmax, minmax, rotate);
    bg::transform(maxmin, maxmin, rotate);

    semmapping::polygon pg;
    semmapping::bg::append(pg.outer(), min);
    semmapping::bg::append(pg.outer(), maxmin);
    semmapping::bg::append(pg.outer(), max);
    semmapping::bg::append(pg.outer(), minmax);
    semmapping::bg::correct(pg);
    return pg;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SemanticMap::removeOutliers(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        // Create the filtering object
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK (40);
        sor.setStddevMulThresh (1.0);
        sor.filter (*cloud_filtered);
        return cloud_filtered;
    }

pcl::PointCloud<pcl::PointXYZ>::Ptr SemanticMap::removeRadiusOutliers(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        // Create the filtering object
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(0.08);
        outrem.setMinNeighborsInRadius (1500);
        outrem.setKeepOrganized(true);
        // apply filter
        outrem.filter (*cloud_filtered);
        return cloud_filtered;
    }

mapping_msgs::SemanticMap::Ptr SemanticMap::createMapMessage(const point &robot, double loaded)
{
    bool isChair = false;
    bool isTable = false;
    int chairID = 0;
    int tableID = 0;


    ROS_WARN("erstelle map msg ");
    std::vector<int> remove_ids;

    mapping_msgs::SemanticMap::Ptr map(new mapping_msgs::SemanticMap);
    for (auto &val : objectList)
    {
        viewer_index += 1;
        SemanticObject &obj = val.second;
        if (obj.name == "Chair" && !isChair){
            isChair = true;
            chairID = val.first;
        }
        if (obj.name == "Table" && !isTable){
            isTable = true;
            tableID = val.first;
        }
        if(loaded == false) {
            if (bg::within(robot, obj.shape_union)) {
                remove_ids.push_back(val.first);
                ROS_INFO_STREAM("Object " << obj.name << " removed BECAUSE INTERFERENCE WITH ROBOT");
                std::cout << "REMOVE OBJECT BECAUSE INTERFERENCE WITH ROBOT" << std::endl;
                continue;
            }
        }
        if (obj.exist_certainty > 0.15)// param_config.certainty_thresh)
        {
            mapping_msgs::SemanticObject obj_msg;
            obj_msg.id = val.first;
            obj_msg.name = obj.name;
            obj_msg.exist_certainty = obj.exist_certainty;
            if(!obj.obb.outer().empty()){
                obj_msg.obb = boostToPolygonMsg(obj.obb);
            }
            obj_msg.shape = boostToPolygonMsg(obj.shape_union);
            point centroid;
            bg::centroid(obj.shape_union, centroid);
            obj.shape_union_cen = centroid;
            obj_msg.position = boostToPointMsg(centroid);
            sensor_msgs::PointCloud2 msg_cloud;

            //ROS_WARN_STREAM("CREATING MAP MESSAGE for obj: "<< obj.name);
            if(obj.point_cloud != nullptr){
                pcl::toROSMsg(*obj.point_cloud, msg_cloud);
                msg_cloud.header.frame_id = "map";
                obj_msg.pointcloud = msg_cloud;
            }

            map->objects.push_back(std::move(obj_msg));
        }
    }

//     for likelihood evaluation only
//    if(loaded == false) {
//        if (isChair == true) {

//            likelihood_value_chair.push_back(objectList.at(chairID).exist_certainty);
//            time_value_chair.push_back(current_time);
//        }
//        if (isChair == false) {
//            likelihood_value_chair.push_back(0.0);
//            time_value_chair.push_back(current_time);
//        }
//        if (isTable == true) {
//            likelihood_value_table.push_back(objectList.at(tableID).exist_certainty);
//            time_value_table.push_back(current_time);
//        }
//        if (isTable == false) {
//            likelihood_value_table.push_back(0.0);
//            time_value_table.push_back(current_time);
//        }
//    }
    for(int i= 0; i<remove_ids.size(); i++){
        removeObject(remove_ids[i]);
    }
    map->header.frame_id = "map";
    map->header.stamp = ros::Time::now();
    return map;
}

bool SemanticMap::writeLikelihoodData(std::ostream &output)
{
    YAML::Node map;
    YAML::Node n;
    std::ostringstream sh;
    std::copy(likelihood_value_chair.begin(), likelihood_value_chair.end()-1, std::ostream_iterator<double>(sh, ","));
    sh << likelihood_value_chair.back();
    n["likelihood_chair"] = sh.str();
    std::ostringstream sh1;
    std::copy(time_value_chair.begin(), time_value_chair.end()-1, std::ostream_iterator<double>(sh1, ","));
    sh1 << time_value_chair.back();
    n["time_chair"] = sh1.str();
    std::ostringstream sh2;
    std::copy(likelihood_value_table.begin(), likelihood_value_table.end()-1, std::ostream_iterator<double>(sh2, ","));
    sh << likelihood_value_table.back();
    n["likelihood_table"] = sh2.str();
    std::ostringstream sh3;
    std::copy(time_value_table.begin(), time_value_table.end()-1, std::ostream_iterator<double>(sh3, ","));
    sh1 << time_value_table.back();
    n["time_table"] = sh3.str();
    map.push_back(n);
    output << map;
    return true;
}

    bool SemanticMap::writeMapData(std::ostream &output)
    {
        YAML::Node map;
        for(const auto &map_entry : objectList)
        {
            const SemanticObject &obj = map_entry.second;
            YAML::Node n;
            n["name"] = obj.name;
            n["exist_certainty"] = obj.exist_certainty;
            n["oriented_box_rot"] = obj.rotation_angle;
            std::ostringstream sh;
            sh << bg::wkt(obj.shape_union);
            n["shape_union"] = sh.str();
            std::ostringstream sh_cen;
            sh_cen << bg::wkt(obj.shape_union_cen);
            n["shape_union_cen"] = sh_cen.str();
            std::ostringstream obb;
            obb << bg::wkt(obj.obb);
            n["oriented_box"] = obb.str();
            std::ostringstream obb_cen;
            obb_cen << bg::wkt(obj.oriented_box_cen);
            n["oriented_box_cen"] = obb_cen.str();
            map.push_back(n);
        }
        output << map;
        return true;
    }

bool SemanticMap::readMapData(std::istream &input)
{
    clearAll();
    YAML::Node map = YAML::Load(input);
    for (const YAML::Node &entry : map)
    {
        SemanticObject obj;
        obj.name = entry["name"].as<std::string>();
        obj.exist_certainty = entry["exist_certainty"].as<double>();
        try
        {
            bg::read_wkt(entry["shape_union"].as<std::string>(), obj.shape_union);
        }
        catch (const bg::read_wkt_exception &e)
        {
            ROS_ERROR_STREAM("Error reading object shape: " << e.what());
            return false;
        }
        try
        {
            bg::read_wkt(entry["oriented_box"].as<std::string>(), obj.obb);
        }
        catch (const bg::read_wkt_exception &e)
        {
            ROS_ERROR_STREAM("Error reading obb: " << e.what());
            return false;
        }
        addObject(obj);
    }
    return true;
}

    mapping_msgs::ObjectPositions::Ptr SemanticMap::findObjectPosition(const mapping_msgs::FindObjects &request)
    {
        point robot = getRobotPosition();
        geometry_msgs::Point center;
        point near_to_position;
        near_to_position.x(request.near_to_position.x);
        near_to_position.y(request.near_to_position.y);
        mapping_msgs::ObjectPositions::Ptr position_msg(new mapping_msgs::ObjectPositions);
        if (request.near_to == "" && request.nearest_to_robot == false) {
            for (auto &val : objectList) {
                const SemanticObject obj = val.second;
                if (obj.name == request.name) {
                    position_msg->name = obj.name;
                    center.x = obj.shape_union_cen.x();
                    center.y = obj.shape_union_cen.y();
                    position_msg->positions.push_back(center);
                }
            }
        }
        else if (request.nearest_to_robot == false) {
            std::vector<size_t> find_objects;
            std::vector<size_t> near_objects;
            size_t best_near_object;
            size_t best_find_object;
            for (auto &val : objectList) {
                const SemanticObject &obj = val.second;
                if (obj.name == request.name) {
                    find_objects.push_back(val.first);
                } else if (obj.name == request.near_to) {
                    near_objects.push_back(val.first);
                }
            }
            double best_near_dist = std::numeric_limits<double>::infinity();
            for (size_t id : near_objects) {
                if (bg::distance(robot, objectList.at(id).shape_union_cen) < best_near_dist) {
                    best_near_object = id;
                    best_near_dist = bg::distance(robot, objectList.at(id).shape_union_cen);
                }
            }
            double best_find_dist = std::numeric_limits<double>::infinity();
            for (size_t id : find_objects) {
                if (bg::distance(objectList.at(best_near_object).shape_union_cen, objectList.at(id).shape_union_cen) < best_find_dist) {
                    best_find_object = id;
                    best_find_dist = bg::distance(objectList.at(best_near_object).shape_union_cen,
                                              objectList.at(id).shape_union_cen);
                }
        }
            position_msg->name = objectList.at(best_find_object).name;
            center.x = objectList.at(best_find_object).shape_union_cen.x();
            center.y = objectList.at(best_find_object).shape_union_cen.y();
            position_msg->positions.push_back(center);
        }
        else if (request.nearest_to_robot == true){
            std::vector<size_t> find_objects;
            size_t best_object;
            for (auto &val : objectList) {
                const SemanticObject obj = val.second;
                if (obj.name == request.name)
                    find_objects.push_back(val.first);
            }
            double best_dist = std::numeric_limits<double>::infinity();
            for(size_t id : find_objects){
                if (bg::distance(robot, objectList.at(id).shape_union_cen) < best_dist)
                    best_object = id;
            }
            position_msg->name = objectList.at(best_object).name;
            center.x = objectList.at(best_object).shape_union_cen.x();
            center.y = objectList.at(best_object).shape_union_cen.y();
            position_msg->positions.push_back(center);
        }
        else if (bg::is_empty(near_to_position) == false){
            std::vector<size_t> find_objects;
            size_t best_object;
            for (auto &val : objectList) {
                const SemanticObject obj = val.second;
                if (obj.name == request.name) {
                    find_objects.push_back(val.first);
                }
            }
            double best_dist = std::numeric_limits<double>::infinity();
            for(size_t id : find_objects){
                if (bg::distance(near_to_position, objectList.at(id).shape_union_cen) < best_dist)
                    best_object = id;
            }
            position_msg->name = objectList.at(best_object).name;
            center.x = objectList.at(best_object).shape_union_cen.x();
            center.y = objectList.at(best_object).shape_union_cen.y();
            position_msg->positions.push_back(center);
        }

        return position_msg;

    }

}

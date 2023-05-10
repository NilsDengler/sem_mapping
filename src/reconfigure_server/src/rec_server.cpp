#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <reconfigure_server/ParamsConfig.h>

void callback(reconfigure_server::ParamsConfig &config, uint32_t level) {
    ros::NodeHandle nh("~");
    nh.setParam("/mapping/euc_min_cluster", config.euc_min_cluster);
    nh.setParam("/mapping/euc_tolerance", config.euc_tolerance);
    nh.setParam("/mapping/mps_min_inliers", config.mps_min_inliers);
    nh.setParam("/mapping/mps_angular_threshold", config.mps_angular_threshold);
    nh.setParam("/mapping/mps_distance_threshold", config.mps_distance_threshold);
    nh.setParam("/mapping/ne_depth_change_factor", config.ne_depth_change_factor);
    nh.setParam("/mapping/ne_smoothing_size", config.ne_smoothing_size);
    nh.setParam("/mapping/links_min", config.links_min);
    nh.setParam("/mapping/links_max", config.links_max);
    nh.setParam("/mapping/rechts_min", config.rechts_min);
    nh.setParam("/mapping/rechts_max", config.rechts_max);
    nh.setParam("/mapping/hinten_min", config.hinten_min);
    nh.setParam("/mapping/hinten_max", config.hinten_max);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "reconfigure_server");

    dynamic_reconfigure::Server<reconfigure_server::ParamsConfig> server;
    dynamic_reconfigure::Server<reconfigure_server::ParamsConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::spin();

    return 0;
}
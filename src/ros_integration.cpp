// System
#include <sstream>
#include <string>
#include <vector>

// // Custom
#include <gpg/candidates_generator.h>
#include <gpg/hand_search.h>
#include <gpg/config_file.h>

// ROS
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGB;
PointCloudRGB::Ptr pcl_point_cloud;

bool pt_cloud_received = false;
int count = 0;

void ptCloudCB(const PointCloudRGB::Ptr& msg)
{
  pcl_point_cloud = msg;
  pt_cloud_received = true;
  count = msg->width;
  // BOOST_FOREACH (const pcl::PointXYZRGBA& pt, msg.points)
  ROS_INFO_STREAM(*pcl_point_cloud);
}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ros_integration");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/pcl_object", 1000, ptCloudCB);

  while (!pt_cloud_received)
  {
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
  

  double finger_width = 0.01;     // HARDCODE
  double hand_outer_diameter  = 0.12;     // HARDCODE
  double hand_depth = 0.06;     // HARDCODE
  double hand_height  = 0.02;     // HARDCODE
  double init_bite  = 0.01;     // HARDCODE

  bool voxelize = true;
  bool remove_outliers = false;
  std::vector<double> workspace{-1.0,1.0,-1.0,1.0,-1.0,1.0};
  std::vector<double> camera_pose{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};

  int num_samples = 1000;
  int num_threads = 1;
  double nn_radius = 0.01;
  int num_orientations = 8;
  int rotation_axis = 2;

  bool plot_grasps = true;
  bool plot_normals = false;
  
  // Create object to generate grasp candidates.
  CandidatesGenerator::Parameters generator_params;
  generator_params.num_samples_ = num_samples;
  generator_params.num_threads_ = num_threads;
  generator_params.plot_normals_ = plot_normals;
  generator_params.plot_grasps_ = plot_grasps;
  generator_params.remove_statistical_outliers_ = remove_outliers;
  generator_params.voxelize_ = voxelize;
  generator_params.workspace_ = workspace;

  HandSearch::Parameters hand_search_params;
  hand_search_params.finger_width_ = finger_width;
  hand_search_params.hand_outer_diameter_ = hand_outer_diameter;
  hand_search_params.hand_depth_ = hand_depth;
  hand_search_params.hand_height_ = hand_height;
  hand_search_params.init_bite_ = init_bite;
  hand_search_params.nn_radius_frames_ = nn_radius;
  hand_search_params.num_orientations_ = num_orientations;
  hand_search_params.num_samples_ = num_samples;
  hand_search_params.num_threads_ = num_threads;
  hand_search_params.rotation_axis_ = rotation_axis;

  CandidatesGenerator candidates_generator(generator_params, hand_search_params);

  // Set the camera pose.
  Eigen::Matrix3Xd view_points(3,1);
  view_points << 0,0,0;

  // Create object to load point cloud from file.
  CloudCamera cloud_cam(pcl_point_cloud, count, view_points);

  // Point cloud preprocessing: voxelize, remove statistical outliers, workspace filter, compute normals, subsample.
  candidates_generator.preprocessPointCloud(cloud_cam);

  // Generate a list of grasp candidates.
  std::vector<Grasp> candidates = candidates_generator.generateGraspCandidates(cloud_cam);

  // ros::spin();

  return 0;
}

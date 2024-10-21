/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#pragma once

/* C++ */
#include <Eigen/Eigen>

/* ROS */
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

/* SGT-DV */
#include <sgtdv_msgs/ConeStampedArr.h>
#include <sgtdv_msgs/Point2DStampedArr.h>
#include "data_processing.h"

class DataAcquisition
{
public:
  explicit DataAcquisition(ros::NodeHandle &nh);
  ~DataAcquisition() = default;

  struct Params
  {
    std::string fixed_frame;
    int size_of_set;
    int num_of_cones;

    float dist_th_x;
    float dist_th_y;
    Eigen::MatrixXd real_coords;
  };

  void updateCamera(const sgtdv_msgs::ConeStampedArr::ConstPtr &msg);
  void updateLidar(const sgtdv_msgs::Point2DStampedArr::ConstPtr &msg);

private:
  void loadParams(const ros::NodeHandle &nh);  
  geometry_msgs::PointStamped transformCoords(const geometry_msgs::PointStamped &coords_child_frame) const;
  bool dataVerification(const Eigen::Ref<const Eigen::RowVector2d> &measured_coords) const;
  
  DataProcessing data_processing_obj_;
  Params params_;

  ros::Subscriber camera_sub_, lidar_sub_;
  ros::Publisher cluster_vis_pub_;
  
  tf::TransformListener listener_;
};

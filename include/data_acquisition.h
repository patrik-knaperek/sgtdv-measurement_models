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

private:
  void loadParams(const ros::NodeHandle &nh);  
  
  void cameraCallback(const sgtdv_msgs::ConeStampedArr::ConstPtr &msg);
  void lidarCallback(const sgtdv_msgs::Point2DStampedArr::ConstPtr &msg);

  void update(std::vector<Eigen::RowVector2d>& measurement_set,
              const std::vector<geometry_msgs::PointStamped>& coords_msg_frame, 
              const std::string& sensor_name);

  bool dataVerification(const Eigen::Ref<const Eigen::RowVector2d> &measured_coords) const;

  struct Params
  {
    std::string fixed_frame;
    int size_of_set;
    int num_of_cones;

    float dist_th_x;
    float dist_th_y;
    Eigen::MatrixXd real_coords;
  };
  
  DataProcessing data_processing_obj_;
  Params params_;

  ros::Subscriber camera_sub_, lidar_sub_;
  ros::Publisher cluster_vis_pub_;
  tf::TransformListener listener_;
};

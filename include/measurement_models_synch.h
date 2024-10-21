/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

#include <math.h>
#include <Eigen/Eigen>
#include <XmlRpcException.h>

#include <sgtdv_msgs/ConeStampedArr.h>
#include <sgtdv_msgs/Point2DStampedArr.h>

#include "../include/measurement_models.h"

class MeasurementModelsSynch
{
public:
  MeasurementModelsSynch(const ros::NodeHandle &nh);
  ~MeasurementModelsSynch() = default;

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

  void setClusterPub(const ros::Publisher &cluster_pub)
  {
    obj_.setClusterPub(cluster_pub);
  };

private:
  void loadParams(const ros::NodeHandle &nh);
  template<typename T> bool loadParam(const ros::NodeHandle &handle, const std::string &name, T* storage) const
  {
    if (!handle.getParam(name, *storage))
    {
      ROS_ERROR("Failed to get parameter \"%s\" from server\n", name.data());
      return false;
    }
    return true;
  };
  template<typename T> bool loadParam(const ros::NodeHandle &handle, const std::string &name,
                                      const T &default_value, T* storage) const
  {
    if (!handle.param<T>(name, *storage, default_value))
    {
      ROS_WARN_STREAM("Failed to get parameter " << name.data() << " from server, setting default: " << default_value);
      return false;
    }
    return true;
  };
  Eigen::ArrayXXd 
  readArray(const ros::NodeHandle &handle, const std::string &param_name, const int rows, const int cols) const;
  
  geometry_msgs::PointStamped transformCoords(const geometry_msgs::PointStamped &coords_child_frame) const;
  bool dataVerification(const Eigen::Ref<const Eigen::RowVector2d> &measured_coords) const;
  
  MeasurementModels obj_;
  Params params_;
  
  tf::TransformListener listener_;
};

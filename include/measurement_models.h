/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#pragma once

/* C++ */
#include <fstream>
#include <Eigen/Eigen>

/* ROS */
#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/MarkerArray.h>

/* SGT-DV */
#include "SGT_Macros.h"

class MeasurementModels
{
public:
  struct Params
  {
    Params() {};
    Params(const int num_of_sensors, const int num_of_cones, const int size_of_set, const int size_of_cluster_max,
          const Eigen::Ref<const Eigen::MatrixXd> &real_coords, const std::string &fixed_frame)
      : num_of_sensors(num_of_sensors)
      , num_of_cones(num_of_cones)
      , size_of_set(size_of_set)
      , size_of_cluster_max(size_of_cluster_max)
      , real_coords(real_coords)
      , fixed_frame(fixed_frame)
    {};

    int num_of_sensors;
    int num_of_cones;
    int size_of_set;
    int size_of_cluster_max;
    Eigen::MatrixXd real_coords;
    std::string fixed_frame;
  };

public:
  MeasurementModels();
  ~MeasurementModels() = default;

  void update(const Eigen::Ref<const Eigen::MatrixX2d> &measured_coords, const std::string &sensor_name);
  
  // Setters
  void setParams(const Params &params)
  {
    params_ = params;
  };
  void initOutFiles(const std::string &out_filename);

  void setClusterPub(const ros::Publisher &cluster_pub)
  {
    cluster_pub_ = cluster_pub;
  };

private:
  void kMeansClustering(const Eigen::Ref<const Eigen::MatrixX2d> &measured_coords);
  void clusterAssociation(const Eigen::Ref<const Eigen::MatrixX2d> &measurements);
  double euclideanDist(const double x1, const double x2, const double y1, const double y2) const;
  double updateMeans(Eigen::Ref<Eigen::RowVectorXd> means,
                    const Eigen::Ref<const Eigen::MatrixXd> &clusters,
                    const Eigen::Ref<const Eigen::RowVectorXd> &count_clusters) const;
                    Eigen::Matrix<double,1,6> computeDisp(const Eigen::Ref<const Eigen::MatrixX2d> &cluster,
                    const Eigen::Ref<const Eigen::Vector2d> &mean) const;
  void updateCsv(std::ofstream &csv_file, const Eigen::Ref<const Eigen::Matrix<double, 
                Eigen::Dynamic, 6>> &disp) const;

  void visualizeCluster(const Eigen::Ref<const Eigen::VectorXd> &cluster_x, 
                        const Eigen::Ref<const Eigen::VectorXd> &cluster_y,
                        const int count_cluster);
  void visualizeMeans();
  
  ros::Publisher log_publisher_;
  Params params_;
  int counter_;

  Eigen::RowVectorXd means_x_, means_y_;
  Eigen::MatrixXd clusters_x_, clusters_y_;
  Eigen::RowVectorXd clusters_size_;

  std::ofstream out_csv_file_lid_;
  std::ofstream out_csv_file_cam_;

  ros::Publisher cluster_pub_;
  visualization_msgs::MarkerArray clusters_vis_;
};
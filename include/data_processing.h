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

class DataProcessing
{
public:
  struct Params
  {
    Params() {};
    Params(const int n_of_cones, const Eigen::Ref<const Eigen::MatrixXd> &real_coords,
          const std::string &fixed_frame)
      : n_of_cones(n_of_cones)
      , real_coords(real_coords)
      , fixed_frame(fixed_frame)
    {};

    int n_of_cones;
    Eigen::MatrixXd real_coords;
    std::string fixed_frame;
  };

public:
  DataProcessing() = default;
  ~DataProcessing()
  {
    out_csv_file_cam_.close();
    out_csv_file_lid_.close();
  };

  void update(const std::vector<Eigen::RowVector2d> &measured_coords, const std::string &sensor_name);
  
  void setParams(const Params &params)
  {
    params_ = params;
  };
  void initMeans(void);
  void initOutFiles(const std::string &out_filename);
  void setClusterPub(const ros::Publisher &cluster_vis_pub)
  {
    cluster_vis_pub_ = cluster_vis_pub;
  };

private:
  void kMeansClustering(const std::vector<Eigen::RowVector2d> &measured_coords);
  void clusterAssociation(const std::vector<Eigen::RowVector2d> &measurements);
  Eigen::Array2d updateMeans(void);
  Eigen::Matrix<double,1,6> computeDisp(const Eigen::Ref<const Eigen::MatrixX2d> &cluster,
  const Eigen::Ref<const Eigen::RowVector2d> &mean) const;
  void updateCsv(std::ofstream &csv_file, const Eigen::Ref<const Eigen::Matrix<double, 
                Eigen::Dynamic, 6>> &disp) const;

  void initClusterMarkers(void);
  void initMeansMarkers(void);
  
  ros::Publisher log_publisher_;
  Params params_;

  Eigen::RowVectorXd clusters_size_;

  std::vector<Eigen::RowVector2d> means_;
  std::vector<std::vector<Eigen::RowVector2d>> clusters_;

  std::ofstream out_csv_file_lid_;
  std::ofstream out_csv_file_cam_;

  ros::Publisher cluster_vis_pub_;
  visualization_msgs::MarkerArray clusters_msg_;
};
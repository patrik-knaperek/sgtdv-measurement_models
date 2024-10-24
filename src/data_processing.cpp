/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

/* Header */
#include "data_processing.h"

/// @brief Initialization of cluster means vector
void DataProcessing::initMeans(void)
{
  means_.reserve(params_.n_of_cones);

  for(size_t i = 0; i < params_.n_of_cones; i++)
  {
    means_.emplace_back(params_.real_coords.row(i));
  }
}

/// @brief Open log file streams.
/// @param out_filename Filename identificator element.
void DataProcessing::initLogFiles(const std::string &out_filename)
{
  std::string path_to_package = ros::package::getPath("measurement_models");
  
  std::string path_to_matrix_file_cam = path_to_package + std::string("/data/" + out_filename + "_camera.csv");
  std::string path_to_matrix_file_lid = path_to_package + std::string("/data/" + out_filename + "_lidar.csv");
  
  camera_log_.open(path_to_matrix_file_cam, std::ios::app);
  if (!camera_log_.is_open())
  {
    ROS_ERROR_STREAM("Could not open file " << path_to_matrix_file_cam << std::endl);
    ros::shutdown();
    return;
  }

  lidar_log_.open(path_to_matrix_file_lid, std::ios::app);
  if (!lidar_log_.is_open())
  {
    ROS_ERROR_STREAM("Could not open file " << path_to_matrix_file_lid << std::endl);
    ros::shutdown();
    return;
  }
}

/// @brief Computes and exports mean, covariances of measurement and distance between mean of measurement 
///        and real coordinates for each cone using K-Means clustering.
/// @param measured_coords 
/// @param sensor_name measurement source specification; "lidar" or "camera"
void DataProcessing::process_data(const std::vector<Eigen::RowVector2d> &measured_coords, 
                              const std::string &sensor_name)
{
  kMeansClustering(measured_coords);
  initMeansMarkers();

  // compute covariance of clusters
  Eigen::Matrix<double, Eigen::Dynamic, 6> covariances 
    = Eigen::Matrix<double, Eigen::Dynamic, 6>::Zero(params_.n_of_cones, 6);
  for (int i = 0; i < params_.n_of_cones; i++)
  {
    const int cluster_size = clusters_[i].size();
    Eigen::MatrixX2d cluster = Eigen::MatrixX2d::Zero(cluster_size,2);

    for (int j = 0; j < cluster_size; j++)
    {
      cluster.row(j) = clusters_[i][j];
    }
    covariances.block<1,6>(i,0) = 
      computeCov(cluster, means_[i]);
  }
  initClusterMarkers();
  cluster_vis_pub_.publish(clusters_msg_);

  // export CSV data
  if (sensor_name.compare(std::string("camera")) == 0)
  {
    updateCsv(camera_log_, covariances);
  }
  else if (sensor_name.compare(std::string("lidar")) == 0)
  {
    updateCsv(lidar_log_, covariances);
  }
}

/// @brief Groups measured data into clusters with K-Means clustering method
/// @param measured_coords 
void DataProcessing::kMeansClustering(const std::vector<Eigen::RowVector2d> &measured_coords)
{
  static bool finished = false;
  do {
    finished = true;
    
    clusters_.clear();
    clusters_.resize(params_.n_of_cones);

    clusterAssociation(measured_coords);

    const auto mean_shift = updateMeans();
    if (mean_shift(0) > params_.n_of_cones * 0.01)
    {
      finished = false;
    }
    if (mean_shift(1) > params_.n_of_cones * 0.01)
    {
      finished = false;
    }
  } while (!finished);
  ROS_INFO("K-Means completed");
}

/// @brief Groupd measurements into clusters based on the closest mean
/// @param measurements
void DataProcessing::clusterAssociation(const std::vector<Eigen::RowVector2d> &measurements)
{
  double closest;
  size_t closest_idx;
  double dist;
  for (const auto& measurement : measurements)
  {
    closest = std::numeric_limits<double>::max();
    for (size_t j = 0; j < params_.n_of_cones; j++)
    {
      dist = (measurement - means_.at(j)).norm();
      if (dist < closest)
      {
        closest = dist;
        closest_idx = j;
      }
    }
    clusters_.at(closest_idx).push_back(measurement);
  }
}

/// @brief Recomputes new means of clusters.
/// @return rate of shift of the means
Eigen::Array2d DataProcessing::updateMeans(void)
{
  Eigen::RowVector2d new_mean, cluster_sum;
  Eigen::Array2d mean_shift = Eigen::Array2d::Zero();
  for (size_t i = 0; i < params_.n_of_cones; i++)
  {
    cluster_sum.setZero();
    for (const auto& measurement : clusters_.at(i))
    {
      cluster_sum += measurement;
    }
    new_mean = cluster_sum / clusters_.at(i).size();
    mean_shift += (means_.at(i) - new_mean).array().abs();
    means_.at(i) = new_mean;
  }
  return mean_shift;
}

/// @brief Computes covariance matrices values.
/// @param cluster set of measurements
/// @param mean mean of measurements
/// @return Values in format `cov_xx`, `cov_yy`, `cov_xy`, `cov_rr`, `cov_tt`, `cov_rt`
Eigen::Matrix<double,1,6> DataProcessing::computeCov(const Eigen::Ref<const Eigen::MatrixX2d> &cluster, 
                                                        const Eigen::Ref<const Eigen::RowVector2d> &mean) const
{
  Eigen::Matrix<double,1,6> covariance;
  const int N = cluster.rows();

  /* carthesian-dependent covariance */

  Eigen::ArrayXd diff_x, diff_y;

  diff_x = cluster.col(0).array() - mean(0);
  covariance(0) = diff_x.matrix().transpose() * diff_x.matrix();
  covariance(0) /= static_cast<double>(N - 1);
  
  diff_y = cluster.col(1).array() - mean(1);
  covariance(1) = diff_y.matrix().transpose() * diff_y.matrix();
  covariance(1) /= static_cast<double>(N - 1);

  covariance(2) = diff_x.matrix().transpose() * diff_y.matrix();
  covariance(2) /= static_cast<double>(N - 1);

  const auto mean_radius = std::sqrt(std::pow(mean(0),2) + std::pow(mean(1),2));
  const auto mean_angle = std::atan2(mean(1), mean(0));
  
  /* polar-dependent variance */

  Eigen::VectorXd cluster_radius(N);
  Eigen::VectorXd cluster_angle(N);
  for (int i = 0; i < N; i++)
  {
    cluster_radius(i) = std::sqrt(std::pow(cluster(i,0),2) + std::pow(cluster(i,1),2));
    
    cluster_angle(i) = std::atan2(cluster(i,1), cluster(i,0));
    if (std::abs(cluster_angle(i)) > M_PI)
    {
      cluster_angle(i) = std::pow(-1,static_cast<int>(cluster_angle(i) > 0) 
                      * (2 * M_PI - std::abs(cluster_angle(i))));
    }
  }
  
  Eigen::ArrayXd diff_r, diff_phi;

  diff_r = cluster_radius.array() - mean_radius;
  covariance(3) = diff_r.matrix().transpose() * diff_r.matrix();
  covariance(3) /= static_cast<double>(N - 1);
  
  diff_phi = cluster_angle.array() - mean_angle;
  covariance(4) = diff_phi.matrix().transpose() * diff_phi.matrix();
  covariance(4) /= static_cast<double>(N - 1);
  
  covariance(5) = diff_r.matrix().transpose() * diff_phi.matrix();
  covariance(5) /= static_cast<double>(N - 1);

  return covariance;
}

/// @brief Log into CSV file. Every cone is represented in format `real_x`, `real_y`, `measured_mean_x`,
///        `measured_mean_y`, `offset_x`, `offset_y`, `cov_xx`, `cov_yy`, `cov_xy`, `cov_rr`, `cov_tt`, 
///        `cov_rt` (t - theta (bearing) angle)
/// @param csv_file log file object
/// @param covariance covariance matrices data
void DataProcessing::updateCsv(std::ofstream &csv_file, const Eigen::Ref<const Eigen::Matrix<double, 
                                  Eigen::Dynamic, 6>> &covariance) const
{
  double offset_x, offset_y;
  for (size_t i = 0; i < params_.n_of_cones; i++)
  {
    offset_x = params_.real_coords(i,0) - means_[i](0);
    offset_y = params_.real_coords(i,1) - means_[i](1);

    csv_file << params_.real_coords(i,0) << "," << params_.real_coords(i,1) << "," << means_[i](0) << ","
            << means_[i](1) << "," << offset_x << "," << offset_y << "," 
            << covariance(i,0) << "," << covariance(i,1) << "," << covariance(i,2) << ","
            << covariance(i,3) << "," << covariance(i,4) << "," << covariance(i,5) << ";" << std::endl;
  }
}

/// @brief Load cluster elements into visualization message
void DataProcessing::initClusterMarkers(void)
{
  for (const auto& cluster : clusters_)
  {
    if (cluster.size() == 0) return;
    
    static int seq = 1;
    visualization_msgs::Marker cluster_marker;
    cluster_marker.ns = "CLUSTER " + std::to_string(seq);
    cluster_marker.header.seq = seq++;
    cluster_marker.header.frame_id = params_.fixed_frame;
    
    cluster_marker.type = visualization_msgs::Marker::POINTS;
    cluster_marker.action = visualization_msgs::Marker::ADD;
    cluster_marker.scale.x = 0.03;
    cluster_marker.scale.y = 0.03;
    cluster_marker.color.a = 1.0;
    cluster_marker.points.reserve(cluster.size());
    cluster_marker.colors.reserve(params_.n_of_cones);
    
    std_msgs::ColorRGBA color;
    color.b = static_cast <float> (std::rand()) / static_cast <float> (RAND_MAX);
    color.r = static_cast <float> (std::rand()) / static_cast <float> (RAND_MAX);
    color.g = static_cast <float> (std::rand()) / static_cast <float> (RAND_MAX);
    color.a = 1.0;

    geometry_msgs::Point point;

    for (const auto& c : cluster)
    {
      point.x = c[0];
      point.y = c[1];
      cluster_marker.points.push_back(point);
      cluster_marker.colors.push_back(color);
    }
    clusters_msg_.markers.push_back(cluster_marker);
  }
}

/// @brief Load cluster means into visualization message
void DataProcessing::initMeansMarkers(void)
{
  visualization_msgs::Marker cluster;
  cluster.ns = "MEANS";
  cluster.header.seq = 0;
  cluster.header.frame_id = params_.fixed_frame;
  
  cluster.type = visualization_msgs::Marker::POINTS;
  cluster.action = visualization_msgs::Marker::ADD;
  cluster.scale.x = 0.06;
  cluster.scale.y = 0.06;
  cluster.color.a = 1.0;
  
  cluster.points.reserve(params_.n_of_cones);
  cluster.colors.reserve(params_.n_of_cones);

  std_msgs::ColorRGBA color;
  color.b = 0.0;
  color.r = 1.0;
  color.g = 0.0;
  color.a = 1.0;

  geometry_msgs::Point point;
  point.z = 0.01;

  for (const auto& mean : means_)
  {
    point.x = mean(0);
    point.y = mean(1);
    cluster.points.push_back(point);
    cluster.colors.push_back(color);
  }
  clusters_msg_.markers.push_back(cluster);
}

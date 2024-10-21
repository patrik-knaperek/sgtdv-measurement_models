/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/measurement_models.h"

#include <XmlRpcException.h>

MeasurementModels::MeasurementModels()
{
  counter_ = 0;
}

MeasurementModels::~MeasurementModels()
{
}

void MeasurementModels::initOutFiles(const std::string &out_filename)
{
  std::string path_to_package = ros::package::getPath("measurement_models");
  
  std::string path_to_matrix_file_cam = path_to_package + std::string("/data/" + out_filename + "_camera.csv");
  std::string path_to_matrix_file_lid = path_to_package + std::string("/data/" + out_filename + "_lidar.csv");
  
  out_csv_file_cam_.open(path_to_matrix_file_cam, std::ios::app);
  if (!out_csv_file_cam_.is_open())
    ROS_ERROR_STREAM("Could not open file " << path_to_matrix_file_cam << std::endl);

  out_csv_file_lid_.open(path_to_matrix_file_lid, std::ios::app);
  if (!out_csv_file_lid_.is_open())
    ROS_ERROR_STREAM("Could not open file " << path_to_matrix_file_lid << std::endl);
}

// compute and export mean, dispersions of measurement and distance between mean of measurement 
// and real coordinates for each cone using K-Means clustering
void MeasurementModels::update(const Eigen::Ref<const Eigen::MatrixX2d> &measured_coords, 
                              const std::string &sensor_name)
{
  kMeansClustering(measured_coords);
  visualizeMeans();

  // compute dispersion of clusters
  Eigen::Matrix<double, Eigen::Dynamic, 6> dispersions 
    = Eigen::Matrix<double, Eigen::Dynamic, 6>::Zero(params_.num_of_cones, 6);
  for (int i = 0; i < params_.num_of_cones; i++)
  {
    int cluster_size = clusters_size_(i);
    Eigen::MatrixX2d cluster = Eigen::MatrixX2d::Zero(cluster_size,2);
    cluster.col(0) = clusters_x_.block(0,i, cluster_size,1);
    cluster.col(1) =  clusters_y_.block(0,i, cluster_size,1);
    
    dispersions.block<1,6>(i,0) = 
      computeDisp(cluster, Eigen::Vector2d(means_x_(i), means_y_(i)));
      visualizeCluster(cluster.col(0), cluster.col(1), cluster_size);
  }

  cluster_pub_.publish(clusters_vis_);

  // export CSV data
  if (sensor_name.compare(std::string("camera")) == 0)
  {
    updateCsv(out_csv_file_cam_, dispersions);
  }
  else if (sensor_name.compare(std::string("lidar")) == 0)
  {
    updateCsv(out_csv_file_lid_, dispersions);
  }

  // calibration completed
  if (++counter_ >= params_.num_of_sensors)
  {
    out_csv_file_cam_.close();
    out_csv_file_lid_.close();
    ros::shutdown();
  } 
}

void MeasurementModels::kMeansClustering(const Eigen::Ref<const Eigen::MatrixX2d> &measured_coords)
{
  // initialization of means
  means_x_ = params_.real_coords.col(0);
  means_y_ = params_.real_coords.col(1);

  static bool finished = false;
  do {
    finished = true;
    clusters_x_ =  clusters_y_ = Eigen::MatrixXd::Zero(params_.size_of_cluster_max, params_.num_of_cones);
    clusters_size_ = Eigen::RowVectorXd::Zero(params_.num_of_cones);

    clusterAssociation(measured_coords);

    if (updateMeans(means_x_, clusters_x_, clusters_size_) > params_.num_of_cones * 0.01)
    {
      finished = false;
    }
    if (updateMeans(means_y_, clusters_y_, clusters_size_) > params_.num_of_cones * 0.01)
    {
      finished = false;
    }
    ROS_INFO_STREAM("new means\n" << means_x_ << "\n" << means_y_);
  } while (!finished);
  ROS_INFO("K-Means completed");
}

// asociate points to clusters based on the closest mean
void MeasurementModels::clusterAssociation(const Eigen::Ref<const Eigen::MatrixX2d> &measurements)
{
  double closest;
  size_t closest_idx;
  double dist;
  for (size_t i = 0; i < params_.size_of_set; i++)
  {
    closest = std::numeric_limits<double>::max();
    for (size_t j = 0; j < params_.num_of_cones; j++)
    {
      dist = euclideanDist(measurements(i,0), means_x_(j), measurements(i,1), means_y_(j));
      if (dist < closest)
      {
        closest = dist;
        closest_idx = j;
      }
    }
    clusters_x_(clusters_size_(closest_idx), closest_idx) = measurements(i,0);
    clusters_y_(clusters_size_(closest_idx)++, closest_idx) = measurements(i,1);
  }
  ROS_INFO_STREAM("clusters X:\n" << clusters_size_ << "\n" << clusters_x_);
  ROS_INFO_STREAM("clusters Y:\n" << clusters_size_ << "\n" << clusters_y_);
}

// euclidean distance of 2D vectors
double MeasurementModels::euclideanDist(const double x1, const double x2, const double y1, const double y2) const
{
  return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2)); 
}

// recomputes new means of clusters, returns rate of shift of the means
double MeasurementModels::updateMeans(Eigen::Ref<Eigen::RowVectorXd> means,
                                      const Eigen::Ref<const Eigen::MatrixXd> &clusters,
                                      const Eigen::Ref<const Eigen::RowVectorXd> &count_clusters) const
{
  double new_mean, mean_shift = 0;
  for (size_t i = 0; i < params_.num_of_cones; i++)
  {
    new_mean = clusters.col(i).sum() / count_clusters(i);
    mean_shift += std::abs(means(i) - new_mean);
    means(i) = new_mean;
  }
  return mean_shift;
}

// compute x and y dispersion of cluster
Eigen::Matrix<double,1,6> MeasurementModels::computeDisp(const Eigen::Ref<const Eigen::MatrixX2d> &cluster, 
                                                        const Eigen::Ref<const Eigen::Vector2d> &mean) const
{
  Eigen::Matrix<double,1,6> disp;
  Eigen::ArrayXd diff_x, diff_y, diff_r, diff_phi;

  const int N = cluster.rows();

      
  diff_x = cluster.col(0).array() - mean(0);
  disp(0) = diff_x.matrix().transpose() * diff_x.matrix();
  disp(0) /= static_cast<double>(N - 1);
  
  diff_y = cluster.col(1).array() - mean(1);
  disp(1) = diff_y.matrix().transpose() * diff_y.matrix();
  disp(1) /= static_cast<double>(N - 1);

  disp(2) = diff_x.matrix().transpose() * diff_y.matrix();
  disp(2) /= static_cast<double>(N - 1);

  const auto mean_radius = std::sqrt(std::pow(mean(0),2) + std::pow(mean(1),2));
  const auto mean_angle = std::atan2(mean(1), mean(0));
  
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
  
  diff_r = cluster_radius.array() - mean_radius;
  disp(3) = diff_r.matrix().transpose() * diff_r.matrix();
  disp(3) /= static_cast<double>(N - 1);
  
  diff_phi = cluster_angle.array() - mean_angle;
  disp(4) = diff_phi.matrix().transpose() * diff_phi.matrix();
  disp(4) /= static_cast<double>(N - 1);
  
  disp(5) = diff_r.matrix().transpose() * diff_phi.matrix();
  disp(5) /= static_cast<double>(N - 1);

  return disp;
}

void MeasurementModels::updateCsv(std::ofstream &csv_file, const Eigen::Ref<const Eigen::Matrix<double, 
                                  Eigen::Dynamic, 6>> &disp) const
{
  double offset_x, offset_y;
  for (size_t i = 0; i < params_.num_of_cones; i++)
  {
    offset_x = params_.real_coords(i,0) - means_x_(i);
    offset_y = params_.real_coords(i,1) - means_y_(i);

    // fill matrix row (CSV format): real_x, real_y, measured_mean_x, measured_mean_y, offset_x, offset_y, 
    // cov_xx, cov_yy, cov_xy, cov_rr, cov_tt, cov_rt (t - theta angle)
    csv_file << params_.real_coords(i,0) << "," << params_.real_coords(i,1) << "," << means_x_(i) << ","
            << means_y_(i) << "," << offset_x << "," << offset_y << "," 
            << disp(i,0) << "," << disp(i,1) << "," << disp(i,2) << ","
            << disp(i,3) << "," << disp(i,4) << "," << disp(i,5) << ";" << std::endl;
  }
}

void MeasurementModels::visualizeCluster(const Eigen::Ref<const Eigen::VectorXd> &cluster_x, 
                                        const Eigen::Ref<const Eigen::VectorXd> &cluster_y, const int cluster_size)
{
  if (cluster_size == 0) return;
  
  static int seq = 1;
  visualization_msgs::Marker cluster;
  cluster.ns = "CLUSTER " + std::to_string(seq);
  cluster.header.seq = seq++;
  cluster.header.frame_id = params_.fixed_frame;
  
  cluster.type = visualization_msgs::Marker::POINTS;
  cluster.action = visualization_msgs::Marker::ADD;
  cluster.scale.x = 0.03;
  cluster.scale.y = 0.03;
  cluster.color.a = 1.0;
  cluster.points.reserve(cluster_size);
  cluster.colors.reserve(params_.num_of_cones);
  
  std_msgs::ColorRGBA color;
  color.b = static_cast <float> (std::rand()) / static_cast <float> (RAND_MAX);
  color.r = static_cast <float> (std::rand()) / static_cast <float> (RAND_MAX);
  color.g = static_cast <float> (std::rand()) / static_cast <float> (RAND_MAX);
  color.a = 1.0;

  geometry_msgs::Point point;
  point.z = 0.0;

  for (int i = 0; i < cluster_size; i++)
  {
    point.x = cluster_x(i);
    point.y = cluster_y(i);
    cluster.points.push_back(point);
    cluster.colors.push_back(color);
  }
  clusters_vis_.markers.push_back(cluster);
}

void MeasurementModels::visualizeMeans()
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
  
  cluster.points.reserve(params_.num_of_cones);
  cluster.colors.reserve(params_.num_of_cones);

  std_msgs::ColorRGBA color;
  color.b = 0.0;
  color.r = 1.0;
  color.g = 0.0;
  color.a = 1.0;

  geometry_msgs::Point point;
  point.z = 0.01;

  for (int i = 0; i < params_.num_of_cones; i++)
  {
    point.x = means_x_(i);
    point.y = means_y_(i);
    cluster.points.push_back(point);
    cluster.colors.push_back(color);
  }
  clusters_vis_.markers.push_back(cluster);
}
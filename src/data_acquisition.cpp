/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "SGT_Utils.h"
#include "data_acquisition.h"

DataAcquisition::DataAcquisition(ros::NodeHandle &nh)
  : camera_sub_(nh.subscribe("/camera/cones", 1, &DataAcquisition::cameraCallback, this))
  , lidar_sub_(nh.subscribe("/lidar/cones", 1, &DataAcquisition::lidarCallback, this))
  , cluster_vis_pub_(nh.advertise<visualization_msgs::MarkerArray>("clusters_visualize", 1, true))
{   
  loadParams(nh);
  data_processing_obj_.setClusterPub(cluster_vis_pub_);
}

/// @brief Load and set parameters from server.
/// @param nh ROS node handle
void DataAcquisition::loadParams(const ros::NodeHandle &nh)
{
  Utils::loadParam(nh, "/fixed_frame", &params_.fixed_frame);
  int num_of_measurements;
  Utils::loadParam(nh, "/number_of_measurements", &num_of_measurements);
  Utils::loadParam(nh, "/number_of_cones", &params_.n_of_cones);
  params_.size_of_set = num_of_measurements * params_.n_of_cones;
  Utils::loadParam(nh, "/number_of_sensors", &params_.n_of_sensors);
  Utils::loadParam(nh, "/distance_threshold_x", &params_.dist_th_x);
  Utils::loadParam(nh, "/distance_threshold_y", &params_.dist_th_y);

  params_.real_coords = Eigen::MatrixX2d::Zero(params_.n_of_cones, 2);
  float x;
  if (Utils::loadParam(nh, "/cone_coords_x", &x))
  {
    params_.real_coords.col(0).setConstant(x);
    params_.real_coords.col(1) = Utils::loadArray(nh, "/cone_coords_y", params_.n_of_cones,1);
  }
  else
  {
    params_.real_coords = Utils::loadArray(nh, "/cone_coords", params_.n_of_cones, 2);
  }
  ROS_INFO_STREAM("real_coords:\n" << params_.real_coords);
  
  data_processing_obj_.setParams(
    DataProcessing::Params(params_.n_of_cones, params_.real_coords, params_.fixed_frame)
  );
  data_processing_obj_.initMeans();

  std::string out_filename;
  Utils::loadParam(nh, "/output_filename", &out_filename);
  data_processing_obj_.initLogFiles(out_filename);
}

/// @brief Camera topic callback, transforms topic message to `std::vector<geometry_msgs::PointStamped>` 
/// and sends for further processing.
/// @param msg topic message
void DataAcquisition::cameraCallback(const sgtdv_msgs::ConeStampedArr::ConstPtr &msg)
{
  static std::vector<Eigen::RowVector2d> measurement_set;

  if (msg->cones.size() == 0 || measurement_set.size() >= params_.size_of_set) return;

  ROS_INFO_STREAM("collected measurements from camera: " << measurement_set.size());
  
  std::vector<geometry_msgs::PointStamped> coords_msg_frame;
  for (const auto& cone : msg->cones)
  {
    // check if data is valid
    if (std::isnan(cone.coords.x) || std::isnan(cone.coords.y))
      continue;
    
    geometry_msgs::PointStamped point;
    point.header = cone.coords.header;
    point.point.x = cone.coords.x;
    point.point.y = cone.coords.y;
    coords_msg_frame.emplace_back(point);
  }

  update(measurement_set, coords_msg_frame, "camera");
}

/// @brief Lidar topic callback, transforms topic message to `std::vector<geometry_msgs::PointStamped>` 
///        and sends for further processing.
/// @param msg topic message
void DataAcquisition::lidarCallback(const sgtdv_msgs::Point2DStampedArr::ConstPtr &msg)
{
  static std::vector<Eigen::RowVector2d> measurement_set;
  
  if (msg->points.size() == 0 || measurement_set.size() >= params_.size_of_set) return;

  std::cout << "collected measurements from lidar: " << measurement_set.size() << std::endl;

  std::vector<geometry_msgs::PointStamped> coords_msg_frame;
  for (const auto& cone : msg->points)
  {
    // check if data is valid
    if (std::isnan(cone.x) || std::isnan(cone.y))
      continue;
    
    geometry_msgs::PointStamped point;
    point.header = cone.header;
    point.point.x = cone.x;
    point.point.y = cone.y;
    coords_msg_frame.emplace_back(point);
  }

  update(measurement_set, coords_msg_frame, "lidar");
}

/// @brief Transform measurement to the desired reference frame and add to the measurement set; 
///        send to the data processing object after reaching desired number of measurements.
/// @param measurement_set cumulative container of collected measurements
/// @param coords_msg_frame new measurements from sensor
/// @param sensor_name specifies the source of measurements; valid values: "camera" or "lidar"
void DataAcquisition::update(std::vector<Eigen::RowVector2d>& measurement_set,
                              const std::vector<geometry_msgs::PointStamped>& coords_msg_frame, 
                              const std::string& sensor_name)
{
  geometry_msgs::PointStamped coords_fixed_frame;
  for (const auto& point : coords_msg_frame)
  {
    // transformation to common frame
    if (point.header.frame_id.compare(params_.fixed_frame) != 0)
      coords_fixed_frame = Utils::transformCoords(listener_, params_.fixed_frame, point);
    else
      coords_fixed_frame = point;    
    Eigen::RowVector2d measured_coords(coords_fixed_frame.point.x, coords_fixed_frame.point.y);

    // add new measurement to the set
    if (distanceValidation(measured_coords))
    {
      measurement_set.emplace_back(measured_coords);
    }

    // send completed set to the data processing object
    if (measurement_set.size() == params_.size_of_set)
    {
      data_processing_obj_.process_data(measurement_set, sensor_name);
      
      // data acquisition completed
      if (++counter_ >= params_.n_of_sensors)
        ros::shutdown();
        
      break;
    }
  }
}

/// @brief Checks if the measurement lies within specified distance from some of the real cones.
/// @param measured_coords measurement data from sensor
/// @return `true` if the measurement is close enough to a real cone
bool DataAcquisition::distanceValidation(const Eigen::Ref<const Eigen::RowVector2d> &measured_coords) const
{
  for (int i = 0; i < params_.n_of_cones; i++)
  {
    if (abs(params_.real_coords(i,0) - measured_coords(0)) < params_.dist_th_x
    &&  abs(params_.real_coords(i,1) - measured_coords(1)) < params_.dist_th_y)
    {   
      return true;
    }
  }
  return false;
}

/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "SGT_Utils.h"
#include "data_acquisition.h"

DataAcquisition::DataAcquisition(ros::NodeHandle &nh)
  : camera_sub_(nh.subscribe("/camera_cones", 1, &DataAcquisition::cameraCallback, this))
  , lidar_sub_(nh.subscribe("/lidar_cones", 1, &DataAcquisition::lidarCallback, this))
  , cluster_vis_pub_(nh.advertise<visualization_msgs::MarkerArray>("clusters_visualize", 1, true))
{   
  loadParams(nh);
  data_processing_obj_.setClusterPub(cluster_vis_pub_);
}

void DataAcquisition::loadParams(const ros::NodeHandle &nh)
{
  Utils::loadParam(nh, "/fixed_frame", &params_.fixed_frame);
  int num_of_measurements, num_of_cones;
  Utils::loadParam(nh, "/number_of_measurements", &num_of_measurements);
  Utils::loadParam(nh, "/number_of_cones", &num_of_cones);
  params_.size_of_set = num_of_measurements * num_of_cones;
  params_.num_of_cones = num_of_cones;
  
  Utils::loadParam(nh, "/distance_treshold_x", &params_.dist_th_x);
  Utils::loadParam(nh, "/distance_treshold_y", &params_.dist_th_y);

  params_.real_coords = Eigen::MatrixX2d::Zero(num_of_cones, 2);
  float x;
  if (Utils::loadParam(nh, "/cone_coords_x", &x))
  {
    params_.real_coords.col(0).setConstant(x);
    params_.real_coords.col(1) = Utils::loadArray(nh, "/cone_coords_y", num_of_cones,1);
  }
  else
  {
    params_.real_coords = Utils::loadArray(nh, "/cone_coords", num_of_cones, 2);
  }
  std::cout << "real_coords:\n" << params_.real_coords << std::endl;
  
  int num_of_sensors;
  Utils::loadParam(nh, "/number_of_sensors", &num_of_sensors);
  data_processing_obj_.setParams(DataProcessing::Params(num_of_sensors, num_of_cones, params_.size_of_set, 
                                          num_of_measurements * 3, params_.real_coords, params_.fixed_frame)
  );
  data_processing_obj_.initMeans();

  std::string out_filename;
  Utils::loadParam(nh, "/output_filename", &out_filename);
  data_processing_obj_.initOutFiles(out_filename);
}

// get measurement from camera
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

// get measurement from lidar
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

// transform coordinates and add to the measurement set or send to data processing object
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
    if (dataVerification(measured_coords))
    {
      measurement_set.emplace_back(measured_coords);
    }

    // send completed set to the data processing object
    if (measurement_set.size() == params_.size_of_set)
    {
      data_processing_obj_.update(measurement_set, sensor_name);
      break;
    }
  }
}

bool DataAcquisition::dataVerification(const Eigen::Ref<const Eigen::RowVector2d> &measured_coords) const
{
  for (int i = 0; i < params_.num_of_cones; i++)
  {
    if (abs(params_.real_coords(i,0) - measured_coords(0)) < params_.dist_th_x &&
      abs(params_.real_coords(i,1) - measured_coords(1)) < params_.dist_th_y)
    {   
      return true;
    }
  }
  return false;
}

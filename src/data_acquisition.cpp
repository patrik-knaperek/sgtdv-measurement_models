/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "SGT_Utils.h"
#include "data_acquisition.h"

DataAcquisition::DataAcquisition(ros::NodeHandle &nh)
  : camera_sub_(nh.subscribe("/camera_cones", 1, &DataAcquisition::updateCamera, this))
  , lidar_sub_(nh.subscribe("/lidar_cones", 1, &DataAcquisition::updateLidar, this))
  , cluster_vis_pub_(nh.advertise<visualization_msgs::MarkerArray>("clusters_visualize", 1, true))
{   
  loadParams(nh);
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

  std::string out_filename;
  Utils::loadParam(nh, "/output_filename", &out_filename);
  data_processing_obj_.initOutFiles(out_filename);
}

// get measurement from camera
void DataAcquisition::updateCamera(const sgtdv_msgs::ConeStampedArr::ConstPtr &msg)
{
  static Eigen::MatrixX2d measurement_set(params_.size_of_set, 2);
  static int count = 0;

  int msg_size = msg->cones.size();
  if (msg_size == 0 || count >= params_.size_of_set) return;

  std::cout << "collected measurements from camera: " << count << std::endl;
  
  geometry_msgs::PointStamped coords_msg_frame = geometry_msgs::PointStamped();
  geometry_msgs::PointStamped coords_fixed_frame = geometry_msgs::PointStamped();
  for (int i = 0; i < msg_size; i++)
  {
    // check if data is valid
    if (std::isnan(msg->cones[i].coords.x) || std::isnan(msg->cones[i].coords.y))
      continue;
        
    // transformation to common frame
    coords_msg_frame.header = msg->cones[i].coords.header;
    coords_msg_frame.point.x = msg->cones[i].coords.x;
    coords_msg_frame.point.y = msg->cones[i].coords.y;
    coords_msg_frame.point.z = 0;

    if (coords_msg_frame.header.frame_id.compare(params_.fixed_frame) == 0)
      coords_fixed_frame = coords_msg_frame;
    else
      coords_fixed_frame = transformCoords(coords_msg_frame);
        
    Eigen::RowVector2d measured_coords(coords_fixed_frame.point.x, coords_fixed_frame.point.y);
    if (dataVerification(measured_coords))
    {
      measurement_set.row(count++) = measured_coords;
    }
    if (count == params_.size_of_set)
    {
      data_processing_obj_.update(measurement_set, "camera");
      break;
    }
  }
}

// get measurement from lidar
void DataAcquisition::updateLidar(const sgtdv_msgs::Point2DStampedArr::ConstPtr &msg)
{
  static Eigen::MatrixX2d measurement_set(params_.size_of_set, 2);
  static int count = 0;
  
  int msg_size = msg->points.size();
  if (msg_size == 0 || count >= params_.size_of_set) return;

  std::cout << "collected measurements from lidar: " << count << std::endl;
      
  geometry_msgs::PointStamped coords_msg_frame = geometry_msgs::PointStamped();
  geometry_msgs::PointStamped coords_fixed_frame = geometry_msgs::PointStamped();
  for (int i = 0; i < msg_size; i++)
  {
    // check if data is valid
    if (std::isnan(msg->points[i].x) || std::isnan(msg->points[i].y))
      continue;

    // transformation to common frame
    coords_msg_frame.header = msg->points[i].header;
    coords_msg_frame.point.x = msg->points[i].x;
    coords_msg_frame.point.y = msg->points[i].y;
    coords_msg_frame.point.z = 0;

    if (coords_msg_frame.header.frame_id.compare(params_.fixed_frame) != 0)
      coords_fixed_frame = transformCoords(coords_msg_frame);
    else
      coords_fixed_frame = coords_msg_frame;

    Eigen::RowVector2d measured_coords(coords_fixed_frame.point.x, coords_fixed_frame.point.y);
    if (dataVerification(measured_coords))
    {
      measurement_set.row(count++) = measured_coords;
    }
    if (count == params_.size_of_set)
    {
      data_processing_obj_.update(measurement_set, "lidar");
      break;
    }
  }
}

geometry_msgs::PointStamped 
DataAcquisition::transformCoords(const geometry_msgs::PointStamped &coords_child_frame) const
{
  geometry_msgs::PointStamped coords_parent_frame = geometry_msgs::PointStamped();
  try
  {
    listener_.transformPoint(params_.fixed_frame, coords_child_frame, coords_parent_frame);
  }
  catch (tf::TransformException &e)
  {
    std::cout << e.what();
  }
  return coords_parent_frame;
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

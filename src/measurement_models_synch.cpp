/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "measurement_models_synch.h"

MeasurementModelsSynch::MeasurementModelsSynch(const ros::NodeHandle &nh)
{   
  loadParams(nh);
}

void MeasurementModelsSynch::loadParams(const ros::NodeHandle &nh)
{
  loadParam(nh, "/fixed_frame", &params_.fixed_frame);
  int num_of_measurements, num_of_cones;
  loadParam(nh, "/number_of_measurements", &num_of_measurements);
  loadParam(nh, "/number_of_cones", &num_of_cones);
  params_.size_of_set = num_of_measurements * num_of_cones;
  params_.num_of_cones = num_of_cones;
  
  loadParam(nh, "/distance_treshold_x", &params_.dist_th_x);
  loadParam(nh, "/distance_treshold_y", &params_.dist_th_y);

  params_.real_coords = Eigen::MatrixX2d::Zero(num_of_cones, 2);
  float x;
  if (loadParam(nh, "/cone_coords_x", &x))
  {
    params_.real_coords.col(0).setConstant(x);
    params_.real_coords.col(1) = readArray(nh, "/cone_coords_y", num_of_cones,1);
    // float y_max, y_min;
    // loadParam(nh, "/cone_coords_y_max", &y_max);
    // loadParam(nh, "/cone_coords_y_min", &y_min);
    // for (int i = 0; i < y_max - y_min + 1; i++)
    // {
    //     params_.real_coords(i,0) = x;
    //     params_.real_coords(i,1) = y_min + i;
    // }
  }
  else
  {
    params_.real_coords = readArray(nh, "/cone_coords", num_of_cones, 2);
  }
  std::cout << "real_coords:\n" << params_.real_coords << std::endl;
  
  MeasurementModels::Params calibration_params;
  calibration_params.real_coords = params_.real_coords;

  calibration_params.num_of_cones = num_of_cones;
  calibration_params.size_of_set = params_.size_of_set;
  calibration_params.size_of_cluster_max = num_of_measurements * 3;
  calibration_params.fixed_frame = params_.fixed_frame;
  loadParam(nh, "/number_of_sensors", &calibration_params.num_of_sensors);
  obj_.setParams(calibration_params);

  std::string out_filename;
  loadParam(nh, "/output_filename", &out_filename);
  obj_.initOutFiles(out_filename);
}

// read multidimensional array from parameter server
Eigen::ArrayXXd MeasurementModelsSynch::readArray(const ros::NodeHandle &handle, const std::string &param_name, 
                                                  const int rows, const int cols) const
{
  XmlRpc::XmlRpcValue param_value;
  Eigen::ArrayXXd array_value = Eigen::ArrayXXd::Zero(rows, cols);
  if (loadParam(handle, param_name, &param_value))
  {
    try
    {
      ROS_ASSERT(param_value.getType() == XmlRpc::XmlRpcValue::TypeArray);
      for (int i = 0; i < rows; i++)
      {
        for (int j = 0; j < cols; j++)
        {
          try
          {
            std::ostringstream ostr;
            ostr << param_value[cols * i  + j];
            std::istringstream istr(ostr.str());
            istr >> array_value(i, j);
          }
          catch(XmlRpc::XmlRpcException &e)
          {
            throw e;;
          }
          catch(...)
          {
            throw;
          }
        }
      }
    }
      catch(XmlRpc::XmlRpcException &e)
      {
        ROS_ERROR_STREAM("ERROR reading from server: " << e.getMessage() <<
                        " for " << param_name << "(type: " << param_value.getType() << ")");
      }
  }
  return array_value;
}

// get measurement from camera
void MeasurementModelsSynch::updateCamera(const sgtdv_msgs::ConeStampedArr::ConstPtr &msg)
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
      obj_.update(measurement_set, "camera");
      break;
    }
  }
}

// get measurement from lidar
void MeasurementModelsSynch::updateLidar(const sgtdv_msgs::Point2DStampedArr::ConstPtr &msg)
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
      obj_.update(measurement_set, "lidar");
      break;
    }
  }
}

geometry_msgs::PointStamped 
MeasurementModelsSynch::transformCoords(const geometry_msgs::PointStamped &coords_child_frame) const
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

bool MeasurementModelsSynch::dataVerification(const Eigen::Ref<const Eigen::RowVector2d> &measured_coords) const
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

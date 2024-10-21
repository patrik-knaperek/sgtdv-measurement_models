/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/measurement_models_synch.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "measurement_models");
  ros::NodeHandle handle;

  ros::Publisher cluster_pub = handle.advertise<visualization_msgs::MarkerArray>("clusters_visualize", 1, true);

  MeasurementModelsSynch synch_obj(handle);
  synch_obj.setClusterPub(cluster_pub);
  
  ros::Subscriber camera_sub = handle.subscribe("/camera_cones", 1, &MeasurementModelsSynch::updateCamera, &synch_obj);
  ros::Subscriber lidar_sub = handle.subscribe("/lidar_cones", 1, &MeasurementModelsSynch::updateLidar, &synch_obj);

  ros::spin();

  return 0;
}


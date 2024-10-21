/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "data_acquisition.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "measurement_models");
  ros::NodeHandle handle;

  DataAcquisition obj(handle);
  
  ros::spin();

  return 0;
}


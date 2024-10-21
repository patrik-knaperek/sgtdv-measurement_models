/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "measurement_models_synch.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "measurement_models");
  ros::NodeHandle handle;

  MeasurementModelsSynch synch_obj(handle);
  
  ros::spin();

  return 0;
}


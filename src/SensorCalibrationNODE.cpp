/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/SensorCalibrationSynch.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_calibration");
    ros::NodeHandle handle;

    SensorCalibrationSynch synchObj(handle);
    
    ros::Subscriber cameraSub = handle.subscribe("/camera_cones", 1, &SensorCalibrationSynch::DoCamera, &synchObj);
    ros::Subscriber lidarSub = handle.subscribe("/lidar_cones", 1, &SensorCalibrationSynch::DoLidar, &synchObj);

    ros::spin();

    return 0;
}


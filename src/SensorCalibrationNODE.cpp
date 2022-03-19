/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/SensorCalibrationSynch.h"
#include <sgtdv_msgs/SensorCalibrationMsg.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_calibration");
    ros::NodeHandle handle;

    SensorCalibrationSynch synchObj;
    synchObj.SetHandle(handle);

    std::string fixedFrame;
    if (!handle.getParam("/fixed_frame", fixedFrame))
        ROS_ERROR("Failed to get parameter from server.\n");
    synchObj.SetFixedFrame(fixedFrame);

    int dataSize;
    if (!handle.getParam("/number_of_meassurements", dataSize))
        ROS_ERROR("Failed to get parameter from server.\n");
    synchObj.SetDataSize(dataSize);
    
    std::string lidarTopicName;
    std::string cameraTopicName;
    std::string logTopicName;

    if (!handle.getParam("/topic_name/lidar", lidarTopicName))
        ROS_ERROR("Failed to get parameter from server.\n");
    if (!handle.getParam("/topic_name/camera", cameraTopicName))
        ROS_ERROR("Failed to get parameter from server.\n");
    if (!handle.getParam("topic_name/log", logTopicName))
        ROS_ERROR("Failed to get parameter from server\n");

    ros::Subscriber cameraSub = handle.subscribe(cameraTopicName, 1, &SensorCalibrationSynch::DoCamera, &synchObj);
    ros::Subscriber lidarSub = handle.subscribe(lidarTopicName, 1, &SensorCalibrationSynch::DoLidar, &synchObj);
    ros::Publisher logPub = handle.advertise<sgtdv_msgs::SensorCalibrationMsg>(logTopicName, 1);
    synchObj.SetPublisher(logPub);

    ros::spin();

    return 0;
}
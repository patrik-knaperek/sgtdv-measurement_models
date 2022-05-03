/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/SensorCalibrationSynch.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_calibration");
    ros::NodeHandle handle;

    SensorCalibrationSynch synchObj;
    
    // get parameters
    std::string fixedFrame;
    if (!handle.getParam("/fixed_frame", fixedFrame))
        ROS_ERROR("Failed to get parameter from server.\n");
    synchObj.SetFixedFrame(fixedFrame);

    int numOfSensors;
    if (!handle.getParam("/number_of_sensors", numOfSensors))
        ROS_ERROR("Failed to get parameter from server.\n");
    synchObj.SetNumOfSensors(numOfSensors);

    int dataSize;
    if (!handle.getParam("/number_of_meassurements", dataSize))
        ROS_ERROR("Failed to get parameter from server.\n");
    synchObj.SetDataSize(dataSize);

    float distTH;
    if (!handle.getParam("/distance_treshold", distTH))
        ROS_ERROR("Failed to get parameter from server.\n");
    synchObj.SetDistTH(distTH);

    std::string outFilename;
    if (!handle.getParam("/output_filename", outFilename))
        ROS_ERROR("Failed to get parameter from server.\n");
    synchObj.SetOutFilename(outFilename);

    // get real coords of cones from parameter server
    Eigen::MatrixX2d realCoords = Eigen::RowVector2d::Zero(2);
    XmlRpc::XmlRpcValue realCoordsParam;

    try
    {
        if (!handle.getParam("/cone_coords", realCoordsParam))
            ROS_ERROR("Failed to get parameter from server \n");

        ROS_ASSERT(realCoordsParam.getType() == XmlRpc::XmlRpcValue::TypeArray);

        for (int j = 0; j < 2; j++)
        {
            try
            {
                std::ostringstream ostr;
                ostr << realCoordsParam[j];
                std::istringstream istr(ostr.str());
                istr >> realCoords(j);
            }
            catch(XmlRpc::XmlRpcException &e)
            {
                throw e;
            }
            catch(...)
            {
                throw;
            }
                
        }
    }
    catch(XmlRpc::XmlRpcException &e)
    {
        ROS_ERROR_STREAM("ERROR reading from server: " <<
                        e.getMessage() <<
                        " for cones_coords (type: " <<
                        realCoordsParam.getType() << ")");
    }
    synchObj.SetRealCoords(realCoords);
    

    std::string lidarTopicName;
    std::string cameraTopicName;
    std::string logTopicName;

    // init topics
    if (!handle.getParam("/topic_name/lidar", lidarTopicName))
        ROS_ERROR("Failed to get parameter from server.\n");
    if (!handle.getParam("/topic_name/camera", cameraTopicName))
        ROS_ERROR("Failed to get parameter from server.\n");
    
    ros::Subscriber cameraSub = handle.subscribe(cameraTopicName, 1, &SensorCalibrationSynch::DoCamera, &synchObj);
    ros::Subscriber lidarSub = handle.subscribe(lidarTopicName, 1, &SensorCalibrationSynch::DoLidar, &synchObj);

    ros::spin();

    return 0;
}
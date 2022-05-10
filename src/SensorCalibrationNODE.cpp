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
    
    // get parameters from parameter server
    std::string fixedFrame;
    if (!handle.getParam("/fixed_frame", fixedFrame))
        ROS_ERROR("Failed to get parameter from server.\n");
    synchObj.SetFixedFrame(fixedFrame);

    int numOfSensors;
    if (!handle.getParam("/number_of_sensors", numOfSensors))
        ROS_ERROR("Failed to get parameter from server.\n");
    synchObj.SetNumOfSensors(numOfSensors);

    float distTH;
    if (!handle.getParam("/distance_treshold", distTH))
        ROS_ERROR("Failed to get parameter from server.\n");
    synchObj.SetDistTH(distTH);

    std::string outFilename;
    if (!handle.getParam("/output_filename", outFilename))
        ROS_ERROR("Failed to get parameter from server.\n");
    synchObj.InitOutFiles(outFilename);

    // get real coords of cones from parameter server
    int numOfMeassurements;
    if (!handle.getParam("/number_of_meassurements", numOfMeassurements))
        ROS_ERROR("Failed to get parameter from server.\n");
    
    int numOfCones;
    if (!handle.getParam("/number_of_cones", numOfCones))
        ROS_ERROR("Failed to get parameter from server\n");
    synchObj.SetDataSize(numOfMeassurements, numOfCones);

    MatrixX2d realCoords = MatrixX2d::Zero(numOfCones, 2);
    XmlRpc::XmlRpcValue realCoordsParam;

    if (!handle.getParam("/cone_coords", realCoordsParam))
    {
        float x;
        if (!handle.getParam("/cone_coords_x", x))
            ROS_ERROR("Failed to get parameter from server \n");
            
        for (int i = 0; i < numOfCones; i++)
        {
            realCoords(i,0) = x;
            realCoords(i,1) = -(numOfCones - 1) / 2 + i;
        }
    }
    else    
    {
        try
        {
            ROS_ASSERT(realCoordsParam.getType() == XmlRpc::XmlRpcValue::TypeArray);

            for (int i = 0; i < numOfCones; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    try
                    {
                        std::ostringstream ostr;
                        ostr << realCoordsParam[2 * i + j];
                        std::istringstream istr(ostr.str());
                        istr >> realCoords(i, j);
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
        }
        catch(XmlRpc::XmlRpcException &e)
        {
            ROS_ERROR_STREAM("ERROR reading from server: " <<
                            e.getMessage() <<
                            " for cones_coords (type: " <<
                            realCoordsParam.getType() << ")");
        }
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
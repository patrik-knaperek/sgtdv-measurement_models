/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/SensorCalibrationSynch.h"

Eigen::ArrayXXd readArray(ros::NodeHandle handle, std::string paramName, int rows, int cols);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_calibration");
    ros::NodeHandle handle;

    SensorCalibrationSynch synchObj;
    
    
    // get parameters from parameter server
    std::string fixedFrame;
    if (!handle.getParam("/fixed_frame", fixedFrame))
        ROS_ERROR("Failed to get parameter \"/fixed_frame\" from server.\n");
    synchObj.SetFixedFrame(fixedFrame);

    int numOfSensors;
    if (!handle.getParam("/number_of_sensors", numOfSensors))
        ROS_ERROR("Failed to get parameter \"/number_of_sensors\" from server.\n");
    synchObj.SetNumOfSensors(numOfSensors);

    float distTHx;
    if (!handle.getParam("/distance_treshold_x", distTHx))
        ROS_ERROR("Failed to get parameter \"/distance_treshold_x\" from server.\n");
    float distTHy;
    if (!handle.getParam("/distance_treshold_y", distTHy))
	    ROS_ERROR("Failed to get parameter \"/distance_treshold_y\" from server.\n");
    synchObj.SetDistTH(distTHx, distTHy);

    std::string outFilename;
    if (!handle.getParam("/output_filename", outFilename))
        ROS_ERROR("Failed to get parameter \"/output_filename\" from server.\n");
    
    
    // get real coords of cones from parameter server
    int numOfMeassurements;
    if (!handle.getParam("/number_of_meassurements", numOfMeassurements))
        ROS_ERROR("Failed to get parameter \"/number_of_meassurements\" from server.\n");
    
    int numOfCones;
    if (!handle.getParam("/number_of_cones", numOfCones))
        ROS_ERROR("Failed to get parameter \"/number_of_cones\" from server\n");
    synchObj.SetDataSize(numOfMeassurements, numOfCones);

    Eigen::MatrixX2d realCoords = Eigen::MatrixX2d::Zero(numOfCones, 2);
    float x;
    if (handle.getParam("/cone_coords_x", x))
    {
     	for (int i = 0; i < numOfCones; i++)
        {
            realCoords(i,0) = x;
            realCoords(i,1) = -(numOfCones - 1) / 2 + i;
        }
    }
    else
    {
        realCoords = readArray(handle, "/cone_coords", numOfCones, 2);
    }
	std::cout << "real_coords:\n" << realCoords << std::endl;
    synchObj.SetRealCoords(realCoords);

    
    // get average values of sensor models
    int modelNumber;
    if (!handle.getParam("/model_number", modelNumber))
        ROS_ERROR("Failed to get parameter \"/model_number\" from server.\n");
    synchObj.SetModelNumber(--modelNumber);

    Eigen::Array<double, N_OF_MODELS, 5> avgValuesCam = Eigen::Array<double, N_OF_MODELS, 5>::Zero();
    avgValuesCam.block<N_OF_MODELS, 1>(0, 0) = readArray(handle, std::string("/camera/number_of_meassurements"), N_OF_MODELS, 1);
    if (avgValuesCam(0, 0) != 0.0)
    {
        avgValuesCam.block<N_OF_MODELS, 2>(0, 1) = readArray(handle, std::string("/camera/offset"), N_OF_MODELS, 2);
        avgValuesCam.block<N_OF_MODELS, 2>(0, 3) = readArray(handle, std::string("/camera/covariance"), N_OF_MODELS, 2);
    }

    Eigen::Array<double, N_OF_MODELS, 5> avgValuesLid = Eigen::Array<double, N_OF_MODELS, 5>::Zero();
    avgValuesLid.block<N_OF_MODELS, 1>(0, 0) = readArray(handle, std::string("/lidar/number_of_meassurements"), N_OF_MODELS, 1);
    if (avgValuesLid(0, 0) != 0.0)
    {
        avgValuesLid.block<N_OF_MODELS, 2>(0, 1) = readArray(handle, std::string("/lidar/offset"), N_OF_MODELS, 2);
        avgValuesLid.block<N_OF_MODELS, 2>(0, 3) = readArray(handle, std::string("/lidar/covariance"), N_OF_MODELS, 2);
    }

    synchObj.SetAvgValues(avgValuesCam, avgValuesLid);

    synchObj.InitOutFiles(outFilename);
    
    ros::Subscriber cameraSub = handle.subscribe("/camera_cones", 1, &SensorCalibrationSynch::DoCamera, &synchObj);
    ros::Subscriber lidarSub = handle.subscribe("/lidar_cones", 1, &SensorCalibrationSynch::DoLidar, &synchObj);

    ros::spin();

    return 0;
}

// read multidimensional array from parameter server
Eigen::ArrayXXd readArray(ros::NodeHandle handle, std::string paramName, int rows, int cols)
{
    XmlRpc::XmlRpcValue paramValue;
    Eigen::ArrayXXd arrayValue = Eigen::ArrayXXd::Zero(rows, cols);
    if (!handle.getParam(paramName, paramValue))
        ROS_ERROR_STREAM("Failed to get parameter " << paramName << " from server\n");
    else
    {
        try
        {
            ROS_ASSERT(paramValue.getType() == XmlRpc::XmlRpcValue::TypeArray);
            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                    try
                    {
                        std::ostringstream ostr;
                        ostr << paramValue[cols * i  + j];
                        std::istringstream istr(ostr.str());
                        istr >> arrayValue(i, j);
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
            ROS_ERROR_STREAM("ERROR reading from sercer: " << e.getMessage() <<
                            " for " << paramName << "(type: " << paramValue.getType() << ")");
        }
    }
    return arrayValue;
}
/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/SensorCalibrationSynch.h"

SensorCalibrationSynch::SensorCalibrationSynch(const ros::NodeHandle &nh)
{   
    LoadParams(nh);
}

void SensorCalibrationSynch::LoadParams(const ros::NodeHandle &nh)
{
    loadParam(nh, "/fixed_frame", &m_params.fixedFrame);
    int numOfMeasurements, numOfCones;
    loadParam(nh, "/number_of_measurements", &numOfMeasurements);
    loadParam(nh, "/number_of_cones", &numOfCones);
    m_params.sizeOfSet = numOfMeasurements * numOfCones;
    m_params.numOfCones = numOfCones;
    
    loadParam(nh, "/distance_treshold_x", &m_params.distTHx);
    loadParam(nh, "/distance_treshold_y", &m_params.distTHy);

    m_params.realCoords = Eigen::MatrixX2d::Zero(numOfCones, 2);
    float x;
    if (loadParam(nh, "/cone_coords_x", &x))
    {
     	float y_max, y_min;
        loadParam(nh, "/cone_coords_y_max", &y_max);
        loadParam(nh, "/cone_coords_y_min", &y_min);
        for (int i = 0; i < y_max - y_min + 1; i++)
        {
            m_params.realCoords(i,0) = x;
            m_params.realCoords(i,1) = y_min + i;
        }
    }
    else
    {
        m_params.realCoords = readArray(nh, "/cone_coords", numOfCones, 2);
    }
	std::cout << "real_coords:\n" << m_params.realCoords << std::endl;
    
    SensorCalibration::CalibrationParams calibrationParams;
    calibrationParams.realCoords = m_params.realCoords;

    calibrationParams.numOfCones = numOfCones;
    calibrationParams.sizeOfSet = m_params.sizeOfSet;
    calibrationParams.sizeOfClusterMax = numOfMeasurements * 3;
    calibrationParams.fixedFrame = m_params.fixedFrame;
    loadParam(nh, "/number_of_sensors", &calibrationParams.numOfSensors);
    m_calibrationObj.SetParams(calibrationParams);

    std::string outFilename;
    loadParam(nh, "/output_filename", &outFilename);
    m_calibrationObj.InitOutFiles(outFilename);
}

// read multidimensional array from parameter server
Eigen::ArrayXXd SensorCalibrationSynch::readArray(const ros::NodeHandle &handle, const std::string &paramName, const int rows, const int cols) const
{
    XmlRpc::XmlRpcValue paramValue;
    Eigen::ArrayXXd arrayValue = Eigen::ArrayXXd::Zero(rows, cols);
    if (loadParam(handle, paramName, &paramValue))
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
            ROS_ERROR_STREAM("ERROR reading from server: " << e.getMessage() <<
                            " for " << paramName << "(type: " << paramValue.getType() << ")");
        }
    }
    return arrayValue;
}

// get measurement from camera
void SensorCalibrationSynch::DoCamera(const sgtdv_msgs::ConeArr::ConstPtr &msg)
{
    static Eigen::MatrixX2d measurementSet(m_params.sizeOfSet, 2);
    static int count = 0;

    int msgSize = msg->cones.size();
    if (msgSize == 0) return;

    std::cout << "collected measurements from camera: " << count << std::endl;
    
    geometry_msgs::PointStamped coordsMsgFrame = geometry_msgs::PointStamped();
    geometry_msgs::PointStamped coordsFixedFrame = geometry_msgs::PointStamped();
    for (int i = 0; i < msgSize; i++)
    {
        // check if data is valid
        if (std::isnan(msg->cones[i].coords.x) || std::isnan(msg->cones[i].coords.y))
            continue;
            
        // transformation to common frame
        coordsMsgFrame.header = msg->cones[i].coords.header;
        coordsMsgFrame.point.x = msg->cones[i].coords.x;
        coordsMsgFrame.point.y = msg->cones[i].coords.y;
        coordsMsgFrame.point.z = 0;

        if (coordsMsgFrame.header.frame_id.compare(m_params.fixedFrame) == 0)
            coordsFixedFrame = coordsMsgFrame;
        else
            coordsFixedFrame = TransformCoords(coordsMsgFrame);
            
        Eigen::RowVector2d measuredCoords(coordsFixedFrame.point.x, coordsFixedFrame.point.y);
        if (DataVerification(measuredCoords))
        {
            measurementSet.row(count++) = measuredCoords;
        }
        if (count == m_params.sizeOfSet)
        {
            m_calibrationObj.Do(measurementSet, "camera");
        }
    }
}

// get measurement from lidar
void SensorCalibrationSynch::DoLidar(const sgtdv_msgs::Point2DArr::ConstPtr &msg)
{
    static Eigen::MatrixX2d measurementSet(m_params.sizeOfSet, 2);
    static int count = 0;
    
    int msgSize = msg->points.size();
    if (msgSize == 0) return;

    std::cout << "collected measurements from lidar: " << count << std::endl;
       
    geometry_msgs::PointStamped coordsMsgFrame = geometry_msgs::PointStamped();
    geometry_msgs::PointStamped coordsFixedFrame = geometry_msgs::PointStamped();
    for (int i = 0; i < msgSize; i++)
    {
        // check if data is valid
        if (std::isnan(msg->points[i].x) || std::isnan(msg->points[i].y))
            continue;

        // transformation to common frame
        coordsMsgFrame.header = msg->points[i].header;
        coordsMsgFrame.point.x = msg->points[i].x;
        coordsMsgFrame.point.y = msg->points[i].y;
        coordsMsgFrame.point.z = 0;

        if (coordsMsgFrame.header.frame_id.compare(m_params.fixedFrame) != 0)
            coordsFixedFrame = TransformCoords(coordsMsgFrame);
        else
            coordsFixedFrame = coordsMsgFrame;

        Eigen::RowVector2d measuredCoords(coordsFixedFrame.point.x, coordsFixedFrame.point.y);
        if (DataVerification(measuredCoords))
        {
            measurementSet.row(count++) = measuredCoords;
        }
        if (count == m_params.sizeOfSet)
        {
            m_calibrationObj.Do(measurementSet, "lidar");
        }
    }
}

geometry_msgs::PointStamped SensorCalibrationSynch::TransformCoords(const geometry_msgs::PointStamped &coordsChildFrame) const
{
    geometry_msgs::PointStamped coordsParentFrame = geometry_msgs::PointStamped();
    try
    {
        m_listener.transformPoint(m_params.fixedFrame, coordsChildFrame, coordsParentFrame);
    }
    catch (tf::TransformException &e)
    {
        std::cout << e.what();
    }
    return coordsParentFrame;
}

bool SensorCalibrationSynch::DataVerification(const Eigen::Ref<const Eigen::RowVector2d> &measuredCoords) const 
{
    for (int i = 0; i < m_params.numOfCones; i++)
    {
        if (abs(m_params.realCoords(i,0) - measuredCoords(0)) < m_params.distTHx &&
            abs(m_params.realCoords(i,1) - measuredCoords(1)) < m_params.distTHy)
        {   
            return true;
        }
    }
    return false;
}

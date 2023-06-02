/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/SensorCalibrationSynch.h"

SensorCalibrationSynch::SensorCalibrationSynch(const ros::NodeHandle &nh)
{   
    LoadParams(nh);
    Init();
}

void SensorCalibrationSynch::LoadParams(const ros::NodeHandle &nh)
{
    loadParam(nh, "/fixed_frame", &m_params.fixedFrame);
    loadParam(nh, "/number_of_measurements", &m_params.numOfMeasurements);
    loadParam(nh, "/number_of_cones", &m_params.numOfCones);
    loadParam(nh, "/distance_treshold_x", &m_params.distTHx);
    loadParam(nh, "/distance_treshold_y", &m_params.distTHy);
    
    m_params.realCoords = Eigen::MatrixX2d::Zero(m_params.numOfCones, 2);
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
        m_params.realCoords = readArray(nh, "/cone_coords", m_params.numOfCones, 2);
    }
	std::cout << "real_coords:\n" << m_params.realCoords << std::endl;

    SensorCalibration::CalibrationParams calibrationParams;
    calibrationParams.numOfCones = m_params.numOfCones;
    loadParam(nh, "/number_of_sensors", &calibrationParams.numOfSensors);
    m_calibrationObj.SetParams(calibrationParams);

    std::string outFilename;
    loadParam(nh, "/output_filename", &outFilename);
    m_calibrationObj.InitOutFiles(outFilename);
}

// read multidimensional array from parameter server
Eigen::ArrayXXd SensorCalibrationSynch::readArray(const ros::NodeHandle &handle, const std::string &paramName, int rows, int cols)
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

void SensorCalibrationSynch::Init()
{
    m_cameraObsX = Eigen::MatrixXd::Zero(m_params.numOfMeasurements,m_params.numOfCones);
    m_cameraObsY = Eigen::MatrixXd::Zero(m_params.numOfMeasurements,m_params.numOfCones);
    m_cameraCount = Eigen::RowVectorXi::Zero(m_params.numOfCones);

    m_lidarObsX = Eigen::MatrixXd::Zero(m_params.numOfMeasurements,m_params.numOfCones);
    m_lidarObsY = Eigen::MatrixXd::Zero(m_params.numOfMeasurements,m_params.numOfCones);
    m_lidarCount = Eigen::RowVectorXi::Zero(m_params.numOfCones);
}

// get measurement from camera
void SensorCalibrationSynch::DoCamera(const sgtdv_msgs::ConeArr::ConstPtr &msg)
{
    int msgSize = msg->cones.size();
    if (msgSize == 0) return;

    std::cout << "collected measurements from camera: " << m_cameraCount << std::endl;
    
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
            
        // data association
        Eigen::RowVector2d measuredCoords(coordsFixedFrame.point.x, coordsFixedFrame.point.y);
        int idx = DataAssociation(measuredCoords, m_cameraObsX, m_cameraObsY, m_cameraCount);
        if (idx < 0) continue;

        if (m_cameraCount(idx) >= m_params.numOfMeasurements)
        { 
            Eigen::MatrixX2d measurementSet(m_params.numOfMeasurements,2);
            measurementSet.col(0) = m_cameraObsX.col(idx);
            measurementSet.col(1) = m_cameraObsY.col(idx);
            m_calibrationObj.Do(measurementSet, m_params.realCoords.row(idx), "camera");
        }
    }
}

// get measurement from lidar
void SensorCalibrationSynch::DoLidar(const sgtdv_msgs::Point2DArr::ConstPtr &msg)
{
    int msgSize = msg->points.size();
    if (msgSize == 0) return;

    std::cout << "collected measurements from lidar: " << m_lidarCount << std::endl;
       
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

        // data association
        Eigen::RowVector2d measuredCoords(coordsFixedFrame.point.x, coordsFixedFrame.point.y);
        int idx = DataAssociation(measuredCoords, m_lidarObsX, m_lidarObsY, m_lidarCount);
        if (idx < 0) continue;
            
        if (m_lidarCount(idx) >= m_params.numOfMeasurements)
        {
            Eigen::MatrixX2d measurementSet(m_params.numOfMeasurements,2);
            measurementSet.col(0) = m_lidarObsX.col(idx);
            measurementSet.col(1) = m_lidarObsY.col(idx);
            m_calibrationObj.Do(measurementSet, m_params.realCoords.row(idx), "lidar");
        }
    }
}

geometry_msgs::PointStamped SensorCalibrationSynch::TransformCoords(geometry_msgs::PointStamped coordsChildFrame)
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

int SensorCalibrationSynch::DataAssociation(const Eigen::Ref<const Eigen::RowVector2d> &measuredCoords, Eigen::Ref<Eigen::MatrixXd> obsX,
                                            Eigen::Ref<Eigen::MatrixXd> obsY, Eigen::Ref<Eigen::RowVectorXi> obsCount)
{
    for (int i = 0; i < m_params.numOfCones; i++)
    {
        if (obsCount(i) < m_params.numOfMeasurements)
        {
            if (abs(m_params.realCoords(i,0) - measuredCoords(0)) < m_params.distTHx &&
		        abs(m_params.realCoords(i,1) - measuredCoords(1)) < m_params.distTHy)
            {   
                obsX(obsCount(i),i) = measuredCoords(0);
                obsY(obsCount(i),i) = measuredCoords(1);
                obsCount(i)++;
                return i;
            }
        }
    }
    return -1;
}

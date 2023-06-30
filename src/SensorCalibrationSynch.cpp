/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/SensorCalibrationSynch.h"

SensorCalibrationSynch::SensorCalibrationSynch()
{   

}

SensorCalibrationSynch::~SensorCalibrationSynch()
{

}

void SensorCalibrationSynch::SetDataSize(int numOfMeassurements, int numOfCones)
{
    m_numOfMeassurements = numOfMeassurements;
    m_numOfCones = numOfCones;
    
    m_calibrationObj.SetNumOfCones(numOfCones);
    
    this->Init();
}

void SensorCalibrationSynch::Init()
{
    m_cameraObsX = Eigen::MatrixXd::Zero(m_numOfMeassurements,m_numOfCones);
    m_cameraObsY = Eigen::MatrixXd::Zero(m_numOfMeassurements,m_numOfCones);
    m_cameraCount = Eigen::RowVectorXi::Zero(m_numOfCones);

    m_lidarObsX = Eigen::MatrixXd::Zero(m_numOfMeassurements,m_numOfCones);
    m_lidarObsY = Eigen::MatrixXd::Zero(m_numOfMeassurements,m_numOfCones);
    m_lidarCount = Eigen::RowVectorXi::Zero(m_numOfCones);
}

// get measurement from camera
void SensorCalibrationSynch::DoCamera(const sgtdv_msgs::ConeStampedArr::ConstPtr &msg)
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

        if (coordsMsgFrame.header.frame_id.compare(m_fixedFrame) == 0)
            coordsFixedFrame = coordsMsgFrame;
        else
            coordsFixedFrame = TransformCoords(coordsMsgFrame);
            
        // data association
        Eigen::RowVector2d measuredCoords(coordsFixedFrame.point.x, coordsFixedFrame.point.y);
        int idx = DataAssociation(measuredCoords, m_cameraObsX, m_cameraObsY, m_cameraCount);
        if (idx < 0) continue;

        if (m_cameraCount(idx) >= m_numOfMeassurements)
        { 
            Eigen::MatrixX2d measurementSet(m_numOfMeassurements,2);
            measurementSet.col(0) = m_cameraObsX.col(idx);
            measurementSet.col(1) = m_cameraObsY.col(idx);
            m_calibrationObj.Do(measurementSet, m_realCoords.row(idx), "camera");
        }
    }
}

// get measurement from lidar
void SensorCalibrationSynch::DoLidar(const sgtdv_msgs::Point2DStampedArr::ConstPtr &msg)
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

        if (coordsMsgFrame.header.frame_id.compare(m_fixedFrame) != 0)
            coordsFixedFrame = TransformCoords(coordsMsgFrame);
        else
            coordsFixedFrame = coordsMsgFrame;

        // data association
        Eigen::RowVector2d measuredCoords(coordsFixedFrame.point.x, coordsFixedFrame.point.y);
        int idx = DataAssociation(measuredCoords, m_lidarObsX, m_lidarObsY, m_lidarCount);
        if (idx < 0) continue;
            
        if (m_lidarCount(idx) >= m_numOfMeassurements)
        {
            Eigen::MatrixX2d measurementSet(m_numOfMeassurements,2);
            measurementSet.col(0) = m_lidarObsX.col(idx);
            measurementSet.col(1) = m_lidarObsY.col(idx);
            m_calibrationObj.Do(measurementSet, m_realCoords.row(idx), "lidar");
        }
    }
}

geometry_msgs::PointStamped SensorCalibrationSynch::TransformCoords(geometry_msgs::PointStamped coordsChildFrame)
{
    geometry_msgs::PointStamped coordsParentFrame = geometry_msgs::PointStamped();
    try
    {
        m_listener.transformPoint(m_fixedFrame, coordsChildFrame, coordsParentFrame);
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
    for (int i = 0; i < m_numOfCones; i++)
    {
        if (obsCount(i) < m_numOfMeassurements)
        {
            if (abs(m_realCoords(i,0) - measuredCoords(0)) < m_distTHx &&
		        abs(m_realCoords(i,1) - measuredCoords(1)) < m_distTHy)
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

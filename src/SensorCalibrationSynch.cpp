/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/SensorCalibrationSynch.h"

SensorCalibrationSynch::SensorCalibrationSynch()
{   
    m_cameraCount = 0;
    m_lidarCount = 0;
}

SensorCalibrationSynch::~SensorCalibrationSynch()
{

}

void SensorCalibrationSynch::SetDataSize(int dataSize)
{
    m_dataSize = dataSize;
    m_cameraObs = Eigen::MatrixXd::Zero(m_dataSize,2);
    m_lidarObs = Eigen::MatrixXd::Zero(m_dataSize,2);
}

// get meassurement from camera
void SensorCalibrationSynch::DoCamera(const sgtdv_msgs::ConeArr::ConstPtr &msg)
{
    int conesCount = msg->cones.size();
    if (conesCount == 0) return;
    
    if (m_cameraCount < m_dataSize)
    {
        geometry_msgs::PointStamped coordsMsgFrame = geometry_msgs::PointStamped();
        geometry_msgs::PointStamped coordsFixedFrame = geometry_msgs::PointStamped();
        int msgSize = msg->cones.size();
        for (int i = 0; i < msgSize; i++)
        {
            if (std::isnan(msg->cones[i].coords.x) || std::isnan(msg->cones[i].coords.y))
                continue;
            
            coordsMsgFrame.header = msg->cones[i].coords.header;
            coordsMsgFrame.point.x = msg->cones[i].coords.x;
            coordsMsgFrame.point.y = msg->cones[i].coords.y;
            coordsMsgFrame.point.z = 0;

            // transformation to common frame
            if (coordsMsgFrame.header.frame_id.compare(m_fixedFrame) != 0)
                coordsFixedFrame = TransformCoords(coordsMsgFrame);
            else
                coordsFixedFrame = coordsMsgFrame;
            
            // data association
            Eigen::RowVector2d meassuredCoords(coordsFixedFrame.point.x, coordsFixedFrame.point.y);
            if (m_calibrationObj.euclidDist(m_realCoords, meassuredCoords) < m_distTH)
            {
                m_cameraObs.row(m_cameraCount) = meassuredCoords;
                m_cameraCount++;
            }
            
            if (m_cameraCount >= m_dataSize)
            {    
                std::cout << "camera break\n";
                
                m_calibrationObj.Do(m_cameraObs, m_realCoords, "camera");
                break;
            }
        }
    }
}

// get meassurement from lidar
void SensorCalibrationSynch::DoLidar(const sgtdv_msgs::Point2DArr::ConstPtr &msg)
{
    int pointsCount = msg->points.size();
    if (pointsCount == 0) return;
    
    if (m_lidarCount < m_dataSize)
    {    
        geometry_msgs::PointStamped coordsMsgFrame = geometry_msgs::PointStamped();
        geometry_msgs::PointStamped coordsFixedFrame = geometry_msgs::PointStamped();
        int msgSize = msg->points.size();
        for (int i = 0; i < msgSize; i++)
        {
            if (std::isnan(msg->points[i].x) || std::isnan(msg->points[i].y))
                continue;
            
            coordsMsgFrame.header = msg->points[i].header;
            coordsMsgFrame.point.x = msg->points[i].x;
            coordsMsgFrame.point.y = msg->points[i].y;
            coordsMsgFrame.point.z = 0;

            // transformation to common frame
            if (coordsMsgFrame.header.frame_id.compare(m_fixedFrame) != 0)
                coordsFixedFrame = TransformCoords(coordsMsgFrame);
            else
                coordsFixedFrame = coordsMsgFrame;

            // data association
            Eigen::RowVector2d meassuredCoords(coordsFixedFrame.point.x, coordsFixedFrame.point.y);
            if (m_calibrationObj.euclidDist(m_realCoords, meassuredCoords) < m_distTH)
            {
                m_lidarObs.row(m_lidarCount) = meassuredCoords;
                m_lidarCount++;
            }
            
            if (m_lidarCount >= m_dataSize)
            {
                std::cout << "lidar break\n";
                m_calibrationObj.Do(m_lidarObs, m_realCoords, "lidar");
                break;
            }
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
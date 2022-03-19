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

// get meassurement from camera
void SensorCalibrationSynch::DoCamera(const sgtdv_msgs::ConeArr::ConstPtr &msg)
{
    if (m_cameraCount < m_dataSize)
    {
        int msgSize = msg->cones.size();
        for (int i = 0; i < msgSize; i++)
        {
            m_cameraObs(m_cameraCount,0) = msg->cones[i].coords.x;
            m_cameraObs(m_cameraCount,1) = msg->cones[i].coords.y;
            m_cameraCount ++;
            
            if (m_cameraCount >= m_dataSize)
            {    
                std::cout << "camera break\n";
                
                m_calibration.Do(m_cameraObs, "camera");
                break;
            }
        }
    }
}

// get meassurement from lidar
void SensorCalibrationSynch::DoLidar(const sgtdv_msgs::Point2DArr::ConstPtr &msg)
{
    if (m_lidarCount < m_dataSize)
    {    
        int msgSize = msg->points.size();
        for (int i = 0; i < msgSize; i++)
        {
            m_lidarObs(m_lidarCount,0) = msg->points[i].x;
            m_lidarObs(m_lidarCount,1) = msg->points[i].y;

            m_lidarCount++;
            
            if (m_lidarCount >= m_dataSize)
            {
                std::cout << "lidar break\n";
                m_calibration.Do(m_lidarObs, "lidar");
                break;
            }
        }
    }
}
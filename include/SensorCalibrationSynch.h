/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

#include <Eigen/Eigen>

#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/Point2DArr.h>


#include "../include/SensorCalibration.h"



class SensorCalibrationSynch
{
    public:
        SensorCalibrationSynch();
        ~SensorCalibrationSynch();

        // Setters
        void SetPublisher(ros::Publisher pub) { m_calibration.SetPublisher(pub); };
        void SetHandle(ros::NodeHandle handle) 
        {
            m_handle = handle;
            m_calibration.SetHandle(handle);
        };
        void SetFixedFrame(std::string fixedFrame) { m_fixedFrame = fixedFrame; };
        void SetDataSize(int dataSize)
        {
            m_dataSize = dataSize;
            m_cameraObs = Eigen::MatrixXd::Zero(m_dataSize,2);
            m_lidarObs = Eigen::MatrixXd::Zero(m_dataSize,2);
        }

        void DoCamera(const sgtdv_msgs::ConeArr::ConstPtr &msg);
        void DoLidar(const sgtdv_msgs::Point2DArr::ConstPtr &msg);
        
        

    private:
        SensorCalibration m_calibration;
        Eigen::MatrixXd m_cameraObs;
        Eigen::MatrixXd m_lidarObs;

        ros::NodeHandle m_handle;
        
        int m_dataSize;
        int m_cameraCount;
        int m_lidarCount;

        std::string m_fixedFrame;

        tf::TransformListener m_listener;

};
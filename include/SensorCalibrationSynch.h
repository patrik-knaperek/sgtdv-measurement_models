/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

#include <Eigen/Eigen>
#include <XmlRpcException.h>

#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/Point2DArr.h>


#include "../include/SensorCalibration.h"



class SensorCalibrationSynch
{
    public:
        SensorCalibrationSynch();
        ~SensorCalibrationSynch();

        // Setters
        void SetPublisher(ros::Publisher pub) { m_calibrationObj.SetPublisher(pub); };
        void SetOutFilename(std::string outFilename) { m_calibrationObj.SetOutFilename(outFilename); };
        void SetNumOfSensors(int numOfSensors) { m_calibrationObj.SetNumOfSensors(numOfSensors); };

        void SetFixedFrame(std::string fixedFrame) { m_fixedFrame = fixedFrame; };
        void SetDataSize(int dataSize);
        void SetDistTH(float distTH) { m_distTH = distTH; };
        void SetRealCoords(const Ref<const RowVector2d> &realCoords) { m_realCoords = realCoords; };

        void DoCamera(const sgtdv_msgs::ConeArr::ConstPtr &msg);
        void DoLidar(const sgtdv_msgs::Point2DArr::ConstPtr &msg);

        double euclidDist(const Ref<const RowVector2d> &v1, const Ref<const RowVector2d> &v2);
        
        

    private:
        geometry_msgs::PointStamped TransformCoords(geometry_msgs::PointStamped coordsChildFrame);
        
        SensorCalibration m_calibrationObj;
        Eigen::MatrixXd m_cameraObs;
        Eigen::MatrixXd m_lidarObs;

        int m_dataSize;
        int m_cameraCount;
        int m_lidarCount;

        std::string m_fixedFrame;

        float m_distTH;
        RowVector2d m_realCoords;

        tf::TransformListener m_listener;
};
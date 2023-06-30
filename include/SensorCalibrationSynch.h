/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

#include <math.h>
#include <Eigen/Eigen>
#include <XmlRpcException.h>

#include <sgtdv_msgs/ConeStampedArr.h>
#include <sgtdv_msgs/Point2DStampedArr.h>

#include "../include/SensorCalibration.h"

class SensorCalibrationSynch
{
    public:
        SensorCalibrationSynch();
        ~SensorCalibrationSynch();

        // Pass through
        void SetPublisher(ros::Publisher pub) { m_calibrationObj.SetPublisher(pub); };
        void InitOutFiles(std::string outFilename) { m_calibrationObj.InitOutFiles(outFilename); };
        void SetNumOfSensors(int numOfSensors) { m_calibrationObj.SetNumOfSensors(numOfSensors); };
        void SetAvgValues(const Eigen::Ref<const Eigen::Array<double, N_OF_MODELS, 5>> &avgValuesCamera,
                        const Eigen::Ref<const Eigen::Array<double, N_OF_MODELS, 5>> &avgValuesLidar)
        {
            m_calibrationObj.SetAvgValues(avgValuesCamera, avgValuesLidar);
        };
        void SetModelNumber(int modelNumber) { m_calibrationObj.SetModelNumber(modelNumber); };
        
        // Setters
        void SetFixedFrame(std::string fixedFrame) { m_fixedFrame = fixedFrame; };
        void SetDataSize(int numOfMeassurements, int numOfCones);
        void SetDistTH(float distTHx, float distTHy) 
	{
		m_distTHx = distTHx;
		m_distTHy = distTHy;
       	};
        void SetRealCoords(const Eigen::Ref<const Eigen::MatrixX2d> &realCoords) { m_realCoords = realCoords; };

        void DoCamera(const sgtdv_msgs::ConeStampedArr::ConstPtr &msg);
        void DoLidar(const sgtdv_msgs::Point2DStampedArr::ConstPtr &msg);      

    private:
        void Init();
        geometry_msgs::PointStamped TransformCoords(geometry_msgs::PointStamped coordsChildFrame);
        int DataAssociation(const Eigen::Ref<const Eigen::RowVector2d> &measuredCoords, Eigen::Ref<Eigen::MatrixXd> obsX,
                            Eigen::Ref<Eigen::MatrixXd> obsY, Eigen::Ref<Eigen::RowVectorXi> obsCount);
        
        SensorCalibration m_calibrationObj;
        Eigen::MatrixX2d m_realCoords;
        Eigen::MatrixXd m_cameraObsX;
        Eigen::MatrixXd m_cameraObsY;
        Eigen::MatrixXd m_lidarObsX;
        Eigen::MatrixXd m_lidarObsY;

        int m_numOfMeassurements;
        int m_numOfCones;
        Eigen::RowVectorXi m_cameraCount;
        Eigen::RowVectorXi m_lidarCount;

        std::string m_fixedFrame;

        float m_distTHx;
	    float m_distTHy;

        tf::TransformListener m_listener;
};

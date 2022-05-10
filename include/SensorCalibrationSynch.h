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

#include <math.h>


#include "../include/SensorCalibration.h"

class SensorCalibrationSynch
{
    public:
        SensorCalibrationSynch();
        ~SensorCalibrationSynch();

        // Setters
        void SetPublisher(ros::Publisher pub) { m_calibrationObj.SetPublisher(pub); };
        void InitOutFiles(std::string outFilename) { m_calibrationObj.InitOutFiles(outFilename); };
        void SetNumOfSensors(int numOfSensors) { m_calibrationObj.SetNumOfSensors(numOfSensors); };

        void SetFixedFrame(std::string fixedFrame) { m_fixedFrame = fixedFrame; };
        void SetDataSize(int numOfMeassurements, int numOfCones);
        void SetDistTH(float distTH) { m_distTH = distTH; };
        void SetRealCoords(const Ref<const MatrixX2d> &realCoords) { m_realCoords = realCoords; };

        void DoCamera(const sgtdv_msgs::ConeArr::ConstPtr &msg);
        void DoLidar(const sgtdv_msgs::Point2DArr::ConstPtr &msg);      

    private:
        void Init();
        geometry_msgs::PointStamped TransformCoords(geometry_msgs::PointStamped coordsChildFrame);
        int DataAssociation(const Ref<const RowVector2d> &meassuredCoords, Ref<MatrixXd> obsX,
                            Ref<MatrixXd> obsY, Ref<RowVectorXi> obsCount);
        double euclidDist(const Ref<const RowVector2d> &v1, const Ref<const RowVector2d> &v2); 
        
        SensorCalibration m_calibrationObj;
        MatrixX2d m_realCoords;
        MatrixXd m_cameraObsX;
        MatrixXd m_cameraObsY;
        MatrixXd m_lidarObsX;
        MatrixXd m_lidarObsY;

        int m_numOfMeassurements;
        int m_numOfCones;
        RowVectorXi m_cameraCount;
        RowVectorXi m_lidarCount;

        std::string m_fixedFrame;

        float m_distTH;

        tf::TransformListener m_listener;
};
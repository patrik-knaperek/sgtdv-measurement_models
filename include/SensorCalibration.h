/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#pragma once

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <Eigen/Core>

using namespace Eigen;

class SensorCalibration
{
    public:
        SensorCalibration();
        ~SensorCalibration();

        void Do(const Ref<const MatrixX2d> &data, std::string sensorName);

        // Setters
        void SetHandle(ros::NodeHandle handle) { m_handle = handle; };
        void SetPublisher(ros::Publisher publisher) { m_logPublisher; };
        
    private:
        void GetRealCoords();
        double euclidDist(const Ref<const RowVector2d> &v1, const Ref<const RowVector2d> &v2);
        void meanAndCov(const Ref<const MatrixX2d> &obs, Ref<RowVector2d> mean, Ref<Matrix2d> cov);
        
        std::string m_sensorName;

        ros::NodeHandle m_handle;
        ros::Publisher m_logPublisher;

        float m_distTH;
        int m_conesCount;
        MatrixX2d m_realCoords;
        MatrixX2d m_meassuredCoords;
};
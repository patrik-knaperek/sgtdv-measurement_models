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

        void Do(const Ref<const MatrixX2d> &meassuredCoords, const Ref<const RowVector2d> &realCoords, std::string sensorName);
        double euclidDist(const Ref<const RowVector2d> &v1, const Ref<const RowVector2d> &v2);

        // Setters
        void SetPublisher(ros::Publisher publisher) { m_logPublisher = publisher; };
        
    private:
        void meanAndCov(const Ref<const MatrixX2d> &obs, Ref<RowVector2d> mean, Ref<Matrix2d> cov);
        
        ros::Publisher m_logPublisher;
        int m_counter;
};
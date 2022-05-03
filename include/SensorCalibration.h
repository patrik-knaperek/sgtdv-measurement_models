/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Core>

using namespace Eigen;

class SensorCalibration
{
    public:
        SensorCalibration();
        ~SensorCalibration();

        void Do(const Ref<const MatrixX2d> &meassuredCoords, const Ref<const RowVector2d> &realCoords, std::string sensorName);
        
        // Setters
        void SetPublisher(ros::Publisher publisher) { m_logPublisher = publisher; };
        void SetNumOfSensors(int numOfSensors) { m_numOfSensors = numOfSensors; };
        void SetOutFilename(std::string outFilename);
        
    private:
        void MeanAndCov(const Ref<const MatrixX2d> &obs, Ref<RowVector2d> mean, Ref<Matrix2d> cov);
        void OpenFile(std::string path);
        
        ros::Publisher m_logPublisher;
        int m_counter;
        int m_numOfSensors;
        std::ofstream m_outFile;
};
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
#include <Eigen/Eigen>

#include "../../SGT_Macros.h"

class SensorCalibration
{
    public:
        SensorCalibration();
        ~SensorCalibration();

        void Do(const Eigen::Ref<const Eigen::MatrixX2d> &measuredCoords, const Eigen::Ref<const Eigen::RowVector2d> &realCoords,
                std::string sensorName);

        struct CalibrationParams
        {
            int numOfSensors;
            int numOfCones;
        };
        
        // Setters
        void SetPublisher(ros::Publisher publisher) { m_logPublisher = publisher; };
        void SetParams(const CalibrationParams &params)
        {
            m_params = params;
        };
        void InitOutFiles(std::string outFilename);
       
    private:
        void MeanAndCov(const Eigen::Ref<const Eigen::MatrixX2d> &obs, Eigen::Ref<Eigen::RowVector2d> mean, Eigen::Ref<Eigen::Matrix2d> cov);
        void UpdateCsv(std::ofstream &csvFile,
                            const Eigen::Ref<const Eigen::RowVector2d> &realCoords, const Eigen::Ref<const Eigen::RowVector2d> &mean,
                            const Eigen::Ref<const Eigen::Matrix2d> &covariance
                        );
        
        ros::Publisher m_logPublisher;
        CalibrationParams m_params;
        int m_counter;
        
        std::ofstream m_outCsvFileLid;
        std::ofstream m_outCsvFileCam;
};
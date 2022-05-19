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

#define N_OF_MODELS 2

class SensorCalibration
{
    public:
        SensorCalibration();
        ~SensorCalibration();

        void Do(const Eigen::Ref<const Eigen::MatrixX2d> &meassuredCoords, const Eigen::Ref<const Eigen::RowVector2d> &realCoords,
                std::string sensorName);
        
        // Setters
        void SetPublisher(ros::Publisher publisher) { m_logPublisher = publisher; };
        void SetNumOfSensors(int numOfSensors) { m_numOfSensors = numOfSensors; };
        void SetNumOfCones(int numOfCones) { m_numOfCones = numOfCones; };
        void SetModelNumber(int modelNumber) { m_modelNumber = modelNumber; };
        void InitOutFiles(std::string outFilename);
        void SetAvgValues(const Eigen::Ref<const Eigen::Array<double, N_OF_MODELS, 5>> &avgValuesCamera,
                        const Eigen::Ref<const Eigen::Array<double, N_OF_MODELS, 5>> &avgValuesLidar)
        {
            m_avgValuesCam = avgValuesCamera;
            m_avgValuesLid = avgValuesLidar;
        };
       
    private:
        void MeanAndCov(const Eigen::Ref<const Eigen::MatrixX2d> &obs, Eigen::Ref<Eigen::RowVector2d> mean, Eigen::Ref<Eigen::Matrix2d> cov);
        void UpdateAvgValues(Eigen::Ref<Eigen::Array<double, N_OF_MODELS, 5>> avgValues, 
                            const Eigen::Ref<const Eigen::RowVector2d> &realCoords, const Eigen::Ref<const Eigen::RowVector2d> &mean,
                            const Eigen::Ref<const Eigen::Matrix2d> &covariance, int numOfNewMeassurements
                        #ifdef SGT_EXPORT_DATA_CSV
                            , std::ofstream &csvFile
                        #endif
                        );
        void WriteToFile(std::ofstream &paramFile, const Eigen::Ref<const Eigen::Array<double, N_OF_MODELS, 5>> &avgValues);
        
        ros::Publisher m_logPublisher;
        int m_counter;
        int m_numOfSensors;
        int m_numOfCones;
        int m_modelNumber;
        std::ofstream m_outParamFileCam;
        std::ofstream m_outParamFileLid;

        // avgValues = [number_of_meassurements, average_offset_x, average_offset_y, covariance_xx, covariance_yy]
        Eigen::Array<double, N_OF_MODELS, 5> m_avgValuesCam;
        Eigen::Array<double, N_OF_MODELS, 5> m_avgValuesLid; 
        
    #ifdef SGT_EXPORT_DATA_CSV
        std::ofstream m_outCsvFileLid;
        std::ofstream m_outCsvFileCam;
    #endif
};
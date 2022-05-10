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

#define EXPORT_AS_MATRIX_FILE

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
        void SetNumOfCones(int numOfCones) { m_numOfCones = numOfCones; };
        void InitOutFiles(std::string outFilename);
        
    private:
        void MeanAndCov(const Ref<const MatrixX2d> &obs, Ref<RowVector2d> mean, Ref<Matrix2d> cov);
        void WriteToFile(std::ofstream &paramFile, const Ref<const RowVector2d> &realCoords,
                        const Ref<const RowVector2d> &mean,
                        const Ref<const Matrix2d> &covariance
                    #ifdef EXPORT_AS_MATRIX_FILE
                        , std::ofstream &matrixFile
                    #endif
        );
        
        ros::Publisher m_logPublisher;
        int m_counter;
        int m_numOfSensors;
        int m_numOfCones;
        std::ofstream m_outParamFileLid;
        std::ofstream m_outParamFileCam;

    #ifdef EXPORT_AS_MATRIX_FILE
        std::ofstream m_outMatrixFileLid;
        std::ofstream m_outMatrixFileCam;
    #endif
};
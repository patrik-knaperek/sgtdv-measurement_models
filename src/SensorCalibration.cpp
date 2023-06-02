/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/SensorCalibration.h"

#include <XmlRpcException.h>

SensorCalibration::SensorCalibration()
{
    m_counter = 0;
}

SensorCalibration::~SensorCalibration()
{
    m_outCsvFileCam.close();
    m_outCsvFileLid.close();
}

void SensorCalibration::InitOutFiles(std::string outFilename)
{
    std::string pathToPackage = ros::package::getPath("calibration");
    
    std::string pathToMatrixFileCam = pathToPackage + std::string("/data/" + outFilename + "_camera.csv");
    std::string pathToMatrixFileLid = pathToPackage + std::string("/data/" + outFilename + "_lidar.csv");
    
    m_outCsvFileCam.open(pathToMatrixFileCam, std::ios::app);
    if (!m_outCsvFileCam.is_open())
        ROS_ERROR_STREAM("Could not open file " << pathToMatrixFileCam << std::endl);

    m_outCsvFileLid.open(pathToMatrixFileLid, std::ios::app);
    if (!m_outCsvFileLid.is_open())
        ROS_ERROR_STREAM("Could not open file " << pathToMatrixFileLid << std::endl);

}

// compute and export mean, covariance of measurement and distance between mean of measurement 
// and real coordinates for each cone
void SensorCalibration::Do(const Eigen::Ref<const Eigen::MatrixX2d> &measuredCoords, const Eigen::Ref<const Eigen::RowVector2d> &realCoords,
                        std::string sensorName)
{
    Eigen::RowVector2d mean(2);
    Eigen::Matrix2d cov(2,2);
    MeanAndCov(measuredCoords, mean, cov);


    // update average values of sensor models
    
    if (sensorName.compare(std::string("camera")) == 0)
    {
        UpdateCsv(m_outCsvFileCam, realCoords, mean, cov);
    }
    else if (sensorName.compare(std::string("lidar")) == 0)
    {
        UpdateCsv( m_outCsvFileLid, realCoords, mean, cov);
    }

    // calibration completed
    if (++m_counter >= m_params.numOfSensors * m_params.numOfCones)
    {
        ros::shutdown();
    } 
}

void SensorCalibration::MeanAndCov(const Eigen::Ref<const Eigen::MatrixX2d> &obs, Eigen::Ref<Eigen::RowVector2d> mean,
                                    Eigen::Ref<Eigen::Matrix2d> cov)
{
    const double obsNum = static_cast<double>(obs.rows());
    mean = obs.colwise().sum() / obsNum;
    cov = (obs.rowwise() - mean).transpose() * (obs.rowwise() - mean) / (obsNum - 1);
}

void SensorCalibration::UpdateCsv(std::ofstream &csvFile, const Eigen::Ref<const Eigen::RowVector2d> &realCoords, 
                            const Eigen::Ref<const Eigen::RowVector2d> &mean,
                            const Eigen::Ref<const Eigen::Matrix2d> &covariance
                        )
{
    Eigen::RowVector2d offset = realCoords - mean;

    // fill matrix row (CSV format)
    csvFile << realCoords(0) << "," << realCoords(1) << "," << mean(0) << ","
            << mean(1) << "," << offset(0) << "," << offset(1) << "," 
            << covariance(0,0) << "," << covariance(1,1) << ";" << std::endl;
}
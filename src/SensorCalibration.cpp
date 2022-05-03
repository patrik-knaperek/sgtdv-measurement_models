#include "../include/SensorCalibration.h"

#include <XmlRpcException.h>

SensorCalibration::SensorCalibration()
{
    m_counter = 0;
}

SensorCalibration::~SensorCalibration()
{
    m_outFile.close();
}

void SensorCalibration::SetOutFilename(std::string outFilename)
{
    std::string pathToPackage = ros::package::getPath("calibration");
    std::string pathToFile = pathToPackage + std::string("/params/" + outFilename + ".yaml");
    OpenFile(pathToFile);
}

void SensorCalibration::OpenFile(std::string path)
{
    m_outFile.open(path);
    if (!m_outFile.is_open())
        ROS_ERROR("Could not open file.\n");
}

// compute and publish mean, covariance of meassurement and distance between mean of meassurement 
// and real coordinates for each cone
void SensorCalibration::Do(const Ref<const MatrixX2d> &meassuredCoords, const Ref<const RowVector2d> &realCoords, std::string sensorName)
{
    std::cout << "real coordinates:\n" << realCoords << std::endl;
    std::cout << "meassured coords from " << sensorName << ": \n" << meassuredCoords << std::endl;
    
    Eigen::RowVector2d mean(2);
    Eigen::Matrix2d cov(2,2);
    MeanAndCov(meassuredCoords, mean, cov);

    std::cout << "meassured mean:\n" << mean << std::endl;
    std::cout << "meassured covariance:\n" << cov << std::endl;

    Eigen::RowVector2d offset(2);
    offset = realCoords - mean;

    // write to file
    m_outFile << sensorName << ":\n";
    m_outFile << "  real_coords: [" << realCoords(0) << ", " << realCoords(1) << "]\n";
    m_outFile << "  meassured_mean: [" << mean(0) << ", " << mean(1) << "]\n";
    m_outFile << "  offset: [" << offset(0) << ", " << offset(1) << "]\n";
    m_outFile << "  meassured_covariance: [" << cov(0,0) << ", " << cov(0,1) 
                                    << ", " << cov(1,0) << ", " << cov(1,1) << "]\n\n";
    
    if (++m_counter >= m_numOfSensors)
    {
        std::cout << "numOfSensors: " << m_numOfSensors << std::endl;
        ros::shutdown();
    } 
}

void SensorCalibration::MeanAndCov(const Ref<const MatrixX2d> &obs, Ref<RowVector2d> mean, Ref<Matrix2d> cov)
{
    const double obsNum = static_cast<double>(obs.rows());
    mean = obs.colwise().sum() / obsNum;
    cov = (obs.rowwise() - mean).transpose() * (obs.rowwise() - mean) / (obsNum - 1);
}
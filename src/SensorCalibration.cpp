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
    m_outParamFileLid.close();
    m_outParamFileCam.close();

#ifdef EXPORT_AS_MATRIX_FILE
    m_outMatrixFileLid.close();
    m_outMatrixFileCam.close();
#endif
}

void SensorCalibration::InitOutFiles(std::string outFilename)
{
    std::string pathToPackage = ros::package::getPath("calibration");
    std::string pathToParamFileLid = pathToPackage + std::string("/params/" + outFilename + "_lidar.yaml");
    std::string pathToParamFileCam = pathToPackage + std::string("/params/" + outFilename + "_camera.yaml");
    
    m_outParamFileLid.open(pathToParamFileLid, std::ios::app);
    if (!m_outParamFileLid.is_open())
        ROS_ERROR("Could not open file.\n");

    m_outParamFileCam.open(pathToParamFileCam, std::ios::app);
    if (!m_outParamFileCam.is_open())
        ROS_ERROR("Could not open file.\n");

    
#ifdef EXPORT_AS_MATRIX_FILE
    std::string pathToMatrixFileLid = pathToPackage + std::string("/data/" + outFilename + "_lidar.txt");
    std::string pathToMatrixFileCam = pathToPackage + std::string("/data/" + outFilename + "_camera.txt");

    m_outMatrixFileLid.open(pathToMatrixFileLid, std::ios::app);
    if (!m_outMatrixFileLid.is_open())
        ROS_ERROR("Could not open file.\n");

    m_outMatrixFileCam.open(pathToMatrixFileCam, std::ios::app);
    if (!m_outMatrixFileCam.is_open())
        ROS_ERROR("Could not open file.\n");
#endif // EXPORT_AS_MATRIX_FILE
}

void::SensorCalibration::WriteToFile(std::ofstream &paramFile, const Ref<const RowVector2d> &realCoords,
                                    const Ref<const RowVector2d> &mean,
                                    const Ref<const Matrix2d> &covariance
                                #ifdef EXPORT_AS_MATRIX_FILE
                                    , std::ofstream &matrixFile
                                #endif
)
{
    paramFile << "\n\nreal_coords_x" << realCoords(0) << "_y" << realCoords(1) << ":\n";
    paramFile << "  meassured_mean: [" << mean(0) << ", " << mean(1) << "]\n";
    RowVector2d offset(2);
    offset = realCoords - mean;
    paramFile << "  offset: [" << offset(0) << ", " << offset(1) << "]\n";
    paramFile << "  meassured_covariance: [" << covariance(0,0) << ", " << covariance(0,1) 
                                    << ", " << covariance(1,0) << ", " << covariance(1,1) << "]\n\n";

#ifdef EXPORT_AS_MATRIX_FILE
    // fill matrix row (Matlab format)
    matrixFile << realCoords(0) << "," << realCoords(1) << "," << mean(0) << ","
            << mean(1) << "," << covariance(0,0) << "," << covariance(1,1) << ";" << std::endl;
#endif
}

// compute and publish mean, covariance of meassurement and distance between mean of meassurement 
// and real coordinates for each cone
void SensorCalibration::Do(const Ref<const MatrixX2d> &meassuredCoords, const Ref<const RowVector2d> &realCoords, std::string sensorName)
{
    std::cout << "real coordinates:\n" << realCoords << std::endl;
    std::cout << "meassured coords from " << sensorName << ": \n" << meassuredCoords << std::endl;
    
    RowVector2d mean(2);
    Matrix2d cov(2,2);
    MeanAndCov(meassuredCoords, mean, cov);
    
    std::cout << "meassured mean:\n" << mean << std::endl;
    std::cout << "meassured covariance:\n" << cov << std::endl;

    RowVector2d offset(2);
    offset = realCoords - mean;

    // write to file
    if (sensorName.compare(std::string("lidar")) == 0)
    {
        WriteToFile(m_outParamFileLid, realCoords, mean, cov
            #ifdef EXPORT_AS_MATRIX_FILE
                , m_outMatrixFileLid
            #endif
            );
    }
    else if (sensorName.compare(std::string("camera")) == 0)
    {
        WriteToFile(m_outParamFileCam, realCoords, mean, cov
            #ifdef EXPORT_AS_MATRIX_FILE
                , m_outMatrixFileCam
            #endif
            );
    }
    
    // calibration completed
    if (++m_counter >= m_numOfSensors * m_numOfCones)
    {
        ros::shutdown();
    } 
}

void SensorCalibration::MeanAndCov(const Ref<const MatrixX2d> &obs, Ref<RowVector2d> mean, Ref<Matrix2d> cov)
{
    const double obsNum = static_cast<double>(obs.rows());
    mean = obs.colwise().sum() / obsNum;
    cov = (obs.rowwise() - mean).transpose() * (obs.rowwise() - mean) / (obsNum - 1);
}
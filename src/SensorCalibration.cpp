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
    m_outParamFileCam.close();
    m_outParamFileLid.close();

#ifdef SGT_EXPORT_DATA_CSV
    m_outCsvFileCam.close();
    m_outCsvFileLid.close();
#endif
}

void SensorCalibration::InitOutFiles(std::string outFilename)
{
    std::string pathToPackage = ros::package::getPath("calibration");
    std::string pathToParamFileCam = pathToPackage + std::string("/params/" + outFilename + "_camera.yaml");
    std::string pathToParamFileLid = pathToPackage + std::string("/params/" + outFilename + "_lidar.yaml");
    
     m_outParamFileCam.open(pathToParamFileCam);
    if (!m_outParamFileCam.is_open())
        ROS_ERROR_STREAM("Could not open file " << pathToParamFileCam << std::endl);

    m_outParamFileLid.open(pathToParamFileLid);
    if (!m_outParamFileLid.is_open())
        ROS_ERROR_STREAM("Could not open file " << pathToParamFileLid << std::endl);
    
#ifdef SGT_EXPORT_DATA_CSV
    std::string pathToMatrixFileCam = pathToPackage + std::string("/data/" + outFilename + "_camera.csv");
    std::string pathToMatrixFileLid = pathToPackage + std::string("/data/" + outFilename + "_lidar.csv");
    
    m_outCsvFileCam.open(pathToMatrixFileCam, std::ios::app);
    if (!m_outCsvFileCam.is_open())
        ROS_ERROR_STREAM("Could not open file " << pathToMatrixFileCam << std::endl);

    m_outCsvFileLid.open(pathToMatrixFileLid, std::ios::app);
    if (!m_outCsvFileLid.is_open())
        ROS_ERROR_STREAM("Could not open file " << pathToMatrixFileLid << std::endl);
#endif // SGT_EXPORT_DATA_CSV
}

// compute and export mean, covariance of meassurement and distance between mean of meassurement 
// and real coordinates for each cone
void SensorCalibration::Do(const Eigen::Ref<const Eigen::MatrixX2d> &meassuredCoords, const Eigen::Ref<const Eigen::RowVector2d> &realCoords,
                        std::string sensorName)
{
    Eigen::RowVector2d mean(2);
    Eigen::Matrix2d cov(2,2);
    MeanAndCov(meassuredCoords, mean, cov);


    // update average values of sensor models
    
    if (sensorName.compare(std::string("camera")) == 0)
    {
        UpdateAvgValues(m_avgValuesCam, realCoords, mean, cov, meassuredCoords.rows()
            #ifdef SGT_EXPORT_DATA_CSV
                , m_outCsvFileCam
            #endif
            );
    }
    else if (sensorName.compare(std::string("lidar")) == 0)
    {
        UpdateAvgValues(m_avgValuesLid, realCoords, mean, cov, meassuredCoords.rows()
            #ifdef SGT_EXPORT_DATA_CSV
                , m_outCsvFileLid
            #endif
            );
    }

    // calibration completed
    if (++m_counter >= m_numOfSensors * m_numOfCones)
    {
        // write to parameter files
        m_outParamFileCam << "camera:" << std::endl;
        WriteToFile(m_outParamFileCam, m_avgValuesCam);

        m_outParamFileLid << "lidar:" << std::endl;
        WriteToFile(m_outParamFileLid, m_avgValuesLid);
        
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

void SensorCalibration::UpdateAvgValues(Eigen::Ref<Eigen::Array<double, N_OF_MODELS, 5>> avgValues, 
                            const Eigen::Ref<const Eigen::RowVector2d> &realCoords, const Eigen::Ref<const Eigen::RowVector2d> &mean,
                            const Eigen::Ref<const Eigen::Matrix2d> &covariance, int numOfNewMeassurements
                        #ifdef SGT_EXPORT_DATA_CSV
                            , std::ofstream &csvFile
                        #endif
                        )
{
    Eigen::RowVector2d offset = realCoords - mean;

#ifdef SGT_EXPORT_DATA_CSV
    // fill matrix row (CSV format)
    csvFile << realCoords(0) << "," << realCoords(1) << "," << mean(0) << ","
            << mean(1) << "," << offset(0) << "," << offset(1) << "," 
            << covariance(0,0) << covariance(1,1) << ";" << std::endl;
#endif
    
    double N = avgValues(m_modelNumber, 0) + 1;
    // offset x, y
    avgValues.block<1, 2>(m_modelNumber, 1).array() *= avgValues(m_modelNumber, 0);
    avgValues.block<1, 2>(m_modelNumber, 1).rowwise() += offset.array();
    avgValues.block<1, 2>(m_modelNumber, 1).array() /= N;
    
    // covariance xx, yy
    avgValues.block<1, 2>(m_modelNumber, 3).array() *= avgValues(m_modelNumber, 0);
    avgValues(m_modelNumber, 3) += covariance(0,0);
    avgValues(m_modelNumber, 4) += covariance(1,1);
    avgValues.block<1, 2>(m_modelNumber, 3).array() /= N;

    avgValues(m_modelNumber, 0) = N;
}

// export to YAML file
void::SensorCalibration::WriteToFile(std::ofstream &paramFile, const Eigen::Ref<const Eigen::Array<double, N_OF_MODELS, 5>> &avgValues)
{
    paramFile << "  number_of_meassurements: [";
    for (int i = 0; i < N_OF_MODELS; i++)
    {
        paramFile << avgValues(i, 0) << ", ";
    } 
    paramFile << "]\n";
    
    paramFile << "  offset: [";
    for (int i = 0; i < N_OF_MODELS; i++)
    {
        paramFile << "\n    ";
        for (int j = 0; j < 2; j++)
        {
            paramFile << avgValues(i, j + 1) << ", ";
        }
    }
    paramFile << "\n  ]\n";
    
    paramFile << "  covariance: [";
    for (int i = 0; i < N_OF_MODELS; i++)
    {
        paramFile << "\n    ";
        for (int j = 0; j < 2; j++)
        {
            paramFile << avgValues(i, j + 3) << ", ";
        }
    }
    paramFile << "\n  ]";
}

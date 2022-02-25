#include "../include/SensorCalibration.h"

#include <XmlRpcException.h>
#include <sgtdv_msgs/SensorCalibrationMsg.h>

SensorCalibration::SensorCalibration()
{
    m_counter = 0;
}

SensorCalibration::~SensorCalibration()
{

}

// compute and publish mean, covariance of meassurement and distance between mean of meassurement 
// and real coordinates for each cone
void SensorCalibration::Do(const Ref<const MatrixX2d> &meassuredCoords, const Ref<const RowVector2d> &realCoords, std::string sensorName)
{
    std::cout << "real coordinates:\n" << realCoords << std::endl;
    std::cout << "meassured coords from " << sensorName << ": \n" << meassuredCoords << std::endl;
    
    Eigen::RowVector2d mean(2);
    Eigen::Matrix2d cov(2,2);
    meanAndCov(meassuredCoords, mean, cov);

    sgtdv_msgs::SensorCalibrationMsg logMsg;
    logMsg.sensor = sensorName;

    logMsg.realCoords.x = realCoords(0);
    logMsg.realCoords.y = realCoords(1);

    logMsg.meassuredMean.x = mean(0);
    logMsg.meassuredMean.y = mean(1);

    logMsg.distance = euclidDist(realCoords, mean);

    logMsg.meassuredCov[0] = cov(0,0);
    logMsg.meassuredCov[1] = cov(0,1);
    logMsg.meassuredCov[2] = cov(1,0);
    logMsg.meassuredCov[3] = cov(1,1);

    m_logPublisher.publish(logMsg);

    if (++m_counter >= 2)
    {
        ros::shutdown();
    } 
}

double SensorCalibration::euclidDist(const Ref<const RowVector2d> &v1, const Ref<const RowVector2d> &v2)
{
    Eigen::RowVector2d diff(2);
    diff(0) = v1(0) - v2(0);
    diff(1) = v1(1) - v2(1);

    //std::cout << "\ndist: " << diff.norm() << std::endl;
    return diff.norm();
}

void SensorCalibration::meanAndCov(const Ref<const MatrixX2d> &obs, Ref<RowVector2d> mean, Ref<Matrix2d> cov)
{
    const double obsNum = static_cast<double>(obs.rows());
    mean = obs.colwise().sum() / obsNum;
    cov = (obs.rowwise() - mean).transpose() * (obs.rowwise() - mean) / (obsNum - 1);
}
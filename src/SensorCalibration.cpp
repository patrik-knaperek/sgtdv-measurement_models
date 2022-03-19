#include "../include/SensorCalibration.h"

#include <XmlRpcException.h>
#include <sgtdv_msgs/SensorCalibrationMsg.h>

SensorCalibration::SensorCalibration()
{
    GetRealCoords();

    if (!m_handle.getParam("/distance_treshold", m_distTH))
        ROS_ERROR("Failed to get parameter from server.\n");
}

SensorCalibration::~SensorCalibration()
{

}

// get real coordinates of cones from parameter server
void SensorCalibration::GetRealCoords()
{
    if (!m_handle.getParam("/cones_count", m_conesCount))
        ROS_ERROR("Failed to get parameter from server\n");
    
    this->m_realCoords = Eigen::MatrixX2d::Zero(m_conesCount,2);
    XmlRpc::XmlRpcValue realCoordsParam;

    try
    {
        if (!m_handle.getParam("/cones_coords", realCoordsParam))
            ROS_ERROR("Failed to get parameter from server \n");

        ROS_ASSERT(realCoordsParam.getType() == XmlRpc::XmlRpcValue::TypeArray);

        for (int i = 0; i < m_conesCount; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                try
                {
                    std::ostringstream ostr;
                    ostr << realCoordsParam[2 * i + j];
                    std::istringstream istr(ostr.str());
                    istr >> m_realCoords(i, j);
                }
                catch(XmlRpc::XmlRpcException &e)
                {
                    throw e;
                }
                catch(...)
                {
                    throw;
                }
                
            }
        }
    }
    catch(XmlRpc::XmlRpcException &e)
    {
        ROS_ERROR_STREAM("ERROR reading from server: " <<
                        e.getMessage() <<
                        " for cones_coords (type: " <<
                        realCoordsParam.getType() << ")");
    }
    
}

// compute and publish mean, covariance of meassurement and distance between mean of meassurement 
// and real coordinates for each cone
void SensorCalibration::Do(const Ref<const MatrixX2d> &meassuredCoords, std::string sensorName)
{
    std::cout << "real coordinates:\n" << m_realCoords << std::endl;
    std::cout << "meassured coords from " << sensorName << ": \n" << meassuredCoords << std::endl;
    
    int nMeassurements = meassuredCoords.rows();
    Eigen::VectorXi nearbyMeassurements = Eigen::VectorXi::Zero(nMeassurements);
    for (int i = 0; i < m_conesCount; i++)
    {
        for (int j = 0; j < meassuredCoords.rows(); j++)
        {
            if (euclidDist(m_realCoords.row(i), meassuredCoords.row(j)) < m_distTH)
            {
                nearbyMeassurements(j) = 1;
            }
        }
        
        Eigen::MatrixX2d obs(nearbyMeassurements.sum(),2);
        int obsCount = 0;
        for (int j = 0; j < nMeassurements; j++)
        {
            if (nearbyMeassurements(j))
            {
                obs.row(obsCount) << meassuredCoords.row(j);
                obsCount++;
            }
        }
        nearbyMeassurements.setZero();

        if (obsCount > 0)
        {
            Eigen::RowVector2d mean(2);
            Eigen::Matrix2d cov(2,2);
            meanAndCov(obs, mean, cov);

            sgtdv_msgs::SensorCalibrationMsg logMsg;
            logMsg.sensor = sensorName;

            logMsg.realCoords.x = m_realCoords(i,0);
            logMsg.realCoords.y = m_realCoords(i,1);

            logMsg.meassuredMean.x = mean(0);
            logMsg.meassuredMean.y = mean(1);

            logMsg.distance = euclidDist(m_realCoords.row(i), mean);

            logMsg.meassuredCov[0] = cov(0,0);
            logMsg.meassuredCov[1] = cov(0,1);
            logMsg.meassuredCov[2] = cov(1,0);
            logMsg.meassuredCov[3] = cov(1,1);

            m_logPublisher.publish(logMsg);
        }
    }
   
}

double SensorCalibration::euclidDist(const Ref<const RowVector2d> &v1, const Ref<const RowVector2d> &v2)
{
    Eigen::RowVector2d diff(2);
    diff(0) = v1(0) - v2(0);
    diff(1) = v1(1) - v2(1);

    return diff.norm();
}

void SensorCalibration::meanAndCov(const Ref<const MatrixX2d> &obs, Ref<RowVector2d> mean, Ref<Matrix2d> cov)
{
    const double obsNum = static_cast<double>(obs.rows());
    mean = obs.colwise().sum() / obsNum;
    cov = (obs.rowwise() - mean).transpose() * (obs.rowwise() - mean) / (obsNum - 1);
}
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
}

void SensorCalibration::InitOutFiles(const std::string &outFilename)
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

// compute and export mean, dispersions of measurement and distance between mean of measurement 
// and real coordinates for each cone using K-Means clustering
void SensorCalibration::Do(const Eigen::Ref<const Eigen::MatrixX2d> &measuredCoords, const std::string &sensorName)
{
    KMeansClustering(measuredCoords);
    VisualizeMeans();

    // compute dispersion of clusters
    Eigen::MatrixX2d dispersions = Eigen::MatrixX2d::Zero(m_params.numOfCones, 2);
    for (size_t i = 0; i < m_params.numOfCones; i++)
    {
        int clusterSize = m_clustersSize(i);
        Eigen::MatrixX2d cluster = Eigen::MatrixX2d::Zero(clusterSize,2);
        cluster.col(0) = m_clustersX.block(0,i, clusterSize,1);
        cluster.col(1) =  m_clustersY.block(0,i, clusterSize,1);
        
        dispersions.block<1,2>(i,0) = 
            ComputeDisp(cluster, Eigen::Vector2d(m_meansX(i), m_meansY(i)));
            VisualizeCluster(cluster.col(0), cluster.col(1), clusterSize);
    }

    m_clusterPub.publish(m_clustersVis);

    // export CSV data
    if (sensorName.compare(std::string("camera")) == 0)
    {
        UpdateCsv(m_outCsvFileCam, dispersions);
    }
    else if (sensorName.compare(std::string("lidar")) == 0)
    {
        UpdateCsv( m_outCsvFileLid, dispersions);
    }

    // calibration completed
    if (++m_counter >= m_params.numOfSensors)
    {
        m_outCsvFileCam.close();
        m_outCsvFileLid.close();
        ros::shutdown();
    } 
}

void SensorCalibration::KMeansClustering(const Eigen::Ref<const Eigen::MatrixX2d> &measuredCoords)
{
    // initialization of means
    m_meansX = m_params.realCoords.col(0);
    m_meansY = m_params.realCoords.col(1);

    static bool finished = false;
    do {
        finished = true;
        m_clustersX =  m_clustersY = Eigen::MatrixXd::Zero(m_params.sizeOfClusterMax, m_params.numOfCones);
        m_clustersSize = Eigen::RowVectorXd::Zero(m_params.numOfCones);

        ClusterAssociation(measuredCoords);

        if (UpdateMeans(m_meansX, m_clustersX, m_clustersSize) > m_params.numOfCones * 0.01)
        {
            finished = false;
        }
        if (UpdateMeans(m_meansY, m_clustersY, m_clustersSize) > m_params.numOfCones * 0.01)
        {
            finished = false;
        }
        ROS_INFO_STREAM("new means\n" << m_meansX << "\n" << m_meansY);
    } while (!finished);
    ROS_INFO ("K-Means completed");
}

// asociate points to clusters based on the closest mean
void SensorCalibration::ClusterAssociation(const Eigen::Ref<const Eigen::MatrixX2d> &measurements)
{
    double closest;
    size_t closestIdx;
    double dist;
    for (size_t i = 0; i < m_params.sizeOfSet; i++)
    {
        closest = std::numeric_limits<double>::max();
        for (size_t j = 0; j < m_params.numOfCones; j++)
        {
            dist = EuclideanDist(measurements(i,0), m_meansX(j), measurements(i,1), m_meansY(j));
            if (dist < closest)
            {
                closest = dist;
                closestIdx = j;
            }
        }
        m_clustersX(m_clustersSize(closestIdx), closestIdx) = measurements(i,0);
        m_clustersY(m_clustersSize(closestIdx)++, closestIdx) = measurements(i,1);
    }
    ROS_INFO_STREAM("clusters X:\n" << m_clustersSize << "\n" << m_clustersX);
    ROS_INFO_STREAM("clusters Y:\n" << m_clustersSize << "\n" << m_clustersY);
}

// euclidean distance of 2D vectors
double SensorCalibration::EuclideanDist(const double x1, const double x2, const double y1, const double y2) const
{
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2)); 
}

// recomputes new means of clusters, returns rate of shift of the means
double SensorCalibration::UpdateMeans(Eigen::Ref<Eigen::RowVectorXd> means,
                                    const Eigen::Ref<const Eigen::MatrixXd> &clusters,
                                    const Eigen::Ref<const Eigen::RowVectorXd> &countClusters) const
{
    double newMean, meanShift = 0;
    for (size_t i = 0; i < m_params.numOfCones; i++)
    {
        newMean = clusters.col(i).sum() / countClusters(i);
        meanShift += std::abs(means(i) - newMean);
        means(i) = newMean;
    }
    return meanShift;
}

// compute x and y dispersion of cluster
Eigen::RowVector2d SensorCalibration::ComputeDisp(const Eigen::Ref<const Eigen::MatrixX2d> &cluster, 
                                                const Eigen::Ref<const Eigen::Vector2d> &mean) const
{
    Eigen::RowVector2d disp;
    Eigen::ArrayXd diff;
        
    diff = cluster.col(0).array() - mean(0);
    disp(0) = diff.matrix().transpose() * diff.matrix();
    disp(0) /= (cluster.size() - 1);
    
    diff = cluster.col(1).array() - mean(1);
    disp(1) = diff.matrix().transpose() * diff.matrix();
    disp(1) /= (cluster.size() - 1);

    return disp;
}

void SensorCalibration::UpdateCsv(std::ofstream &csvFile, const Eigen::Ref<const Eigen::MatrixX2d> &disp) const
{
    double offsetX, offsetY;
    for (size_t i = 0; i < m_params.numOfCones; i++)
    {
        offsetX = m_params.realCoords(i,0) - m_meansX(i);
        offsetY = m_params.realCoords(i,1) - m_meansY(i);

        // fill matrix row (CSV format)
        csvFile << m_params.realCoords(i,0) << "," << m_params.realCoords(i,1) << "," << m_meansX(i) << ","
                << m_meansY(i) << "," << offsetX << "," << offsetY << "," 
                << disp(i,0) << "," << disp(i,1) << ";" << std::endl;
    }
}

void SensorCalibration::VisualizeCluster(const Eigen::Ref<const Eigen::VectorXd> &clusterX, 
                                        const Eigen::Ref<const Eigen::VectorXd> &clusterY, const int clusterSize)
{
    if (clusterSize == 0) return;
    
    static int seq = 1;
    visualization_msgs::Marker cluster;
    cluster.ns = "CLUSTER " + std::to_string(seq);
    cluster.header.seq = seq++;
    cluster.header.frame_id = m_params.fixedFrame;
    
    cluster.type = visualization_msgs::Marker::POINTS;
    cluster.action = visualization_msgs::Marker::ADD;
    cluster.scale.x = 0.03;
    cluster.scale.y = 0.03;
    cluster.color.a = 1.0;
    cluster.points.reserve(clusterSize);
    cluster.colors.reserve(m_params.numOfCones);
    
    std_msgs::ColorRGBA color;
    color.b = static_cast <float> (std::rand()) / static_cast <float> (RAND_MAX);
    color.r = static_cast <float> (std::rand()) / static_cast <float> (RAND_MAX);
    color.g = static_cast <float> (std::rand()) / static_cast <float> (RAND_MAX);
    color.a = 1.0;

    geometry_msgs::Point point;
    point.z = 0.0;

    for (int i = 0; i < clusterSize; i++)
    {
        point.x = clusterX(i);
        point.y = clusterY(i);
        cluster.points.push_back(point);
        cluster.colors.push_back(color);
        
    }
    m_clustersVis.markers.push_back(cluster);
}

void SensorCalibration::VisualizeMeans()
{
    visualization_msgs::Marker cluster;
    cluster.ns = "MEANS";
    cluster.header.seq = 0;
    cluster.header.frame_id = m_params.fixedFrame;
    
    cluster.type = visualization_msgs::Marker::POINTS;
    cluster.action = visualization_msgs::Marker::ADD;
    cluster.scale.x = 0.06;
    cluster.scale.y = 0.06;
    cluster.color.a = 1.0;
    
    cluster.points.reserve(m_params.numOfCones);
    cluster.colors.reserve(m_params.numOfCones);

    std_msgs::ColorRGBA color;
    color.b = 0.0;
    color.r = 1.0;
    color.g = 0.0;
    color.a = 1.0;

    geometry_msgs::Point point;
    point.z = 0.01;

    for (int i = 0; i < m_params.numOfCones; i++)
    {
        point.x = m_meansX(i);
        point.y = m_meansY(i);
        cluster.points.push_back(point);
        cluster.colors.push_back(color);
        
    }
    m_clustersVis.markers.push_back(cluster);
}
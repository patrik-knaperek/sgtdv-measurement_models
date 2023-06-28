/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/MarkerArray.h>


#include "../../SGT_Macros.h"

class SensorCalibration
{
    public:
        SensorCalibration();
        ~SensorCalibration();

        void Do(const Eigen::Ref<const Eigen::MatrixX2d> &measuredCoords, const std::string &sensorName);

        struct CalibrationParams
        {
            int numOfSensors;
            int numOfCones;
            int sizeOfSet;
            int sizeOfClusterMax;
            Eigen::MatrixXd realCoords;
            std::string fixedFrame;
        };
        
        // Setters
        void SetParams(const CalibrationParams &params)
        {
            m_params = params;
        };
        void InitOutFiles(const std::string &outFilename);

        void SetClusterPub(const ros::Publisher &cluster_pub)
        {
            m_clusterPub = cluster_pub;
        };

    private:
        void KMeansClustering(const Eigen::Ref<const Eigen::MatrixX2d> &measuredCoords);
        void ClusterAssociation(const Eigen::Ref<const Eigen::MatrixX2d> &measurements);
        double EuclideanDist(const double x1, const double x2, const double y1, const double y2) const;
        double UpdateMeans(Eigen::Ref<Eigen::RowVectorXd> means,
                                    const Eigen::Ref<const Eigen::MatrixXd> &clusters,
                                    const Eigen::Ref<const Eigen::RowVectorXd> &countClusters) const;
        Eigen::RowVector3d ComputeDisp(const Eigen::Ref<const Eigen::MatrixX2d> &cluster, const Eigen::Ref<const Eigen::Vector2d> &mean) const;
        void UpdateCsv(std::ofstream &csvFile, const Eigen::Ref<const Eigen::MatrixX3d> &disp) const;

        void VisualizeCluster(const Eigen::Ref<const Eigen::VectorXd> &clusterX, const Eigen::Ref<const Eigen::VectorXd> &clusterY,
                            const int countCluster);
        void VisualizeMeans();
        
        ros::Publisher m_logPublisher;
        CalibrationParams m_params;
        int m_counter;

        Eigen::RowVectorXd m_meansX, m_meansY;
        Eigen::MatrixXd m_clustersX, m_clustersY;
        Eigen::RowVectorXd m_clustersSize;

        std::ofstream m_outCsvFileLid;
        std::ofstream m_outCsvFileCam;

        ros::Publisher m_clusterPub;
        visualization_msgs::MarkerArray m_clustersVis;
};
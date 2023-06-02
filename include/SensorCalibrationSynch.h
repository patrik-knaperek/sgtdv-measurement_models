/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

#include <math.h>
#include <Eigen/Eigen>
#include <XmlRpcException.h>

#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/Point2DArr.h>

#include "../include/SensorCalibration.h"

class SensorCalibrationSynch
{
    public:
        SensorCalibrationSynch(const ros::NodeHandle &nh);
        ~SensorCalibrationSynch() = default;

        struct CalibrationSynchParams
        {
            std::string fixedFrame;
            int numOfMeasurements;
            int numOfCones;
            float distTHx;
            float distTHy;
            Eigen::MatrixXd realCoords;
        };

        void DoCamera(const sgtdv_msgs::ConeArr::ConstPtr &msg);
        void DoLidar(const sgtdv_msgs::Point2DArr::ConstPtr &msg);      

    private:
        void LoadParams(const ros::NodeHandle &nh);
        template<typename T> bool loadParam(const ros::NodeHandle &handle, const std::string &name, T* storage) const
        {
            if (!handle.getParam(name, *storage))
            {
                ROS_ERROR("Failed to get parameter \"%s\" from server\n", name.data());
                return false;
            }
            return true;
        };
        template<typename T> bool loadParam(const ros::NodeHandle &handle, const std::string &name,
                                        const T &defaultValue, T* storage) const
        {
            if (!handle.param<T>(name, *storage, defaultValue))
            {
                ROS_WARN_STREAM("Failed to get parameter " << name.data() << " from server, setting default: " << defaultValue);
                return false;
            }
            return true;
        };
        Eigen::ArrayXXd readArray(const ros::NodeHandle &handle, const std::string &paramName, int rows, int cols);
        
        void Init();
        geometry_msgs::PointStamped TransformCoords(geometry_msgs::PointStamped coordsChildFrame);
        int DataAssociation(const Eigen::Ref<const Eigen::RowVector2d> &measuredCoords, Eigen::Ref<Eigen::MatrixXd> obsX,
                            Eigen::Ref<Eigen::MatrixXd> obsY, Eigen::Ref<Eigen::RowVectorXi> obsCount);
        
        SensorCalibration m_calibrationObj;
        CalibrationSynchParams m_params;
        Eigen::MatrixXd m_cameraObsX;
        Eigen::MatrixXd m_cameraObsY;
        Eigen::MatrixXd m_lidarObsX;
        Eigen::MatrixXd m_lidarObsY;

        Eigen::RowVectorXi m_cameraCount;
        Eigen::RowVectorXi m_lidarCount;

        tf::TransformListener m_listener;
};

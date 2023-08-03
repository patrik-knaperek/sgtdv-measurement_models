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

#include <sgtdv_msgs/ConeStampedArr.h>
#include <sgtdv_msgs/Point2DStampedArr.h>

#include "../include/SensorCalibration.h"

class SensorCalibrationSynch
{
    public:
        SensorCalibrationSynch(const ros::NodeHandle &nh);
        ~SensorCalibrationSynch() = default;

        struct CalibrationSynchParams
        {
            std::string fixedFrame;
            int sizeOfSet;
            int numOfCones;

            float distTHx;
            float distTHy;
            Eigen::MatrixXd realCoords;

        };

        void DoCamera(const sgtdv_msgs::ConeStampedArr::ConstPtr &msg);
        void DoLidar(const sgtdv_msgs::Point2DStampedArr::ConstPtr &msg);

        void SetClusterPub(const ros::Publisher &cluster_pub)
        {
            m_calibrationObj.SetClusterPub(cluster_pub);
        };

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
        Eigen::ArrayXXd readArray(const ros::NodeHandle &handle, const std::string &paramName, const int rows, const int cols) const;
        
        geometry_msgs::PointStamped TransformCoords(const geometry_msgs::PointStamped &coordsChildFrame) const;
        bool DataVerification(const Eigen::Ref<const Eigen::RowVector2d> &measuredCoords) const;
        
        SensorCalibration m_calibrationObj;
        CalibrationSynchParams m_params;
        
        tf::TransformListener m_listener;
};

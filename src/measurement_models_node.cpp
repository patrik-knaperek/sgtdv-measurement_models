/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/measurement_models_synch.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "measurement_models");
    ros::NodeHandle handle;

    ros::Publisher cluster_pub = handle.advertise<visualization_msgs::MarkerArray>("clusters_visualize", 1, true);

    MeasurementModelsSynch synchObj(handle);
    synchObj.SetClusterPub(cluster_pub);
    
    ros::Subscriber cameraSub = handle.subscribe("/camera_cones", 1, &MeasurementModelsSynch::DoCamera, &synchObj);
    ros::Subscriber lidarSub = handle.subscribe("/lidar_cones", 1, &MeasurementModelsSynch::DoLidar, &synchObj);

    ros::spin();

    return 0;
}


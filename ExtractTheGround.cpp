#include "ExtractTheGround.h"

/*
* Remove the ground with pcl method of plane segmentation
* */
const float THRESHOLD_GROUND = 3;

ExtractTheGround::ExtractTheGround(): threshold_ground_(THRESHOLD_GROUND){}

void ExtractTheGround::compute(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudFiltered,pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudGround){
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(pointCloud);
    vg.setLeafSize(0.1, 0.1, 0.1);
    vg.filter(*pointCloudFiltered);

    std::cout << "PointCloud after filtering has: " << pointCloudFiltered->points.size() << " data points." << std::endl; //*

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    //seg.setMaxIterations (100);
    seg.setDistanceThreshold(threshold_ground_);

    int i = 0, nr_points = (int) pointCloudFiltered->points.size();
    /*while (cloud_filtered->points.size () > 0.3 * nr_points)
    {*/
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(pointCloudFiltered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        //break;
    }
    // cloud->points(inliers->index[])
    /* uint32_t rgb5 = (static_cast<uint32_t>(255) << 16 |
                static_cast<uint32_t>(0) << 8 | static_cast<uint32_t>(0));
     for (size_t i = 0; i < inliers->indices.size (); ++i)
        point_cloud_ptr->points[inliers->indices[i]].rgb = *reinterpret_cast<float*>(&rgb5);*/
    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(pointCloudFiltered);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);
    //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_f);
    *pointCloudFiltered = *cloud_f;
}

#include "RGBSegmentation.h"

/*
* RGBsegmentation from pcl library
* */
RGBSegmentation::RGBSegmentation() {}

void RGBSegmentation::compute(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_entry,pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud_colored){
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
   /*pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (pointcloud_entry);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    pass.filter (*indices);*/

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud (pointcloud_entry);
    //reg.setIndices (indices);
    reg.setSearchMethod (tree);
    reg.setDistanceThreshold (20);
    reg.setPointColorThreshold (1);
    reg.setRegionColorThreshold (1);
    reg.setMinClusterSize (200);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    *pointCloud_colored = *reg.getColoredCloud ();

}
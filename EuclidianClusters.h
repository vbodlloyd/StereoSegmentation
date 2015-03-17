#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

class EuclidianClusters {

public :
    EuclidianClusters();
    void compute(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud,std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& vec_clusters);

private:
    float threshold_ground_;
};
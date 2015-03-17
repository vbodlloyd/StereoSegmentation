#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/search/kdtree.h>
class RGBSegmentation {

public:
    RGBSegmentation();
    void compute(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud_colored);

};
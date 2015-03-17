#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include "EuclidianClusters.h"

/*
* Make clusters with the euclidian clusters method from pcl library
* */
EuclidianClusters::EuclidianClusters() {}

void EuclidianClusters::compute(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud,std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& vec_clusters){
// Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(pointcloud);

    std::vector <pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(2);
    ec.setMinClusterSize(400);
    ec.setMaxClusterSize(100000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(pointcloud);
    ec.extract(cluster_indices);
    int j = 0;
    int r = 200, g = 200, b = 0;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb_memory (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<std::vector<float>> point2D;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud_rgb->points.resize(cloud_cluster->points.size());
        int min = 50000, max = -5000;
        //pcl::PointCloud<pcl::PointXYZ>::Ptr vecxyz(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<float> pointF;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {

            cloud_cluster->points.push_back(pointcloud->points[*pit]);
            pcl::PointXYZRGB p;
            uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                    static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            p.x = pointcloud->points[*pit].x;
            p.y = pointcloud->points[*pit].y;
            p.z = pointcloud->points[*pit].z;

            p.rgb = *reinterpret_cast<float*>(&rgb);
            if(p.y<min)
                min = p.y;
            if(p.y > max)
                max = p.y;
            pointF.push_back(p.x);
            pointF.push_back(p.y);
            //if(p.z > biggerZ/3)
            cloud_rgb->points.push_back(p);
        }
        point2D.push_back(pointF);
        //arrayVector.push_back(vecxyz);
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_rgb->width = cloud_cluster->points.size ();
        cloud_rgb->height = 1;
        cloud_cluster->is_dense = true;


        *cloud_rgb_memory += *cloud_rgb;
        vec_clusters.push_back(cloud_rgb);

        j++;
    }

}


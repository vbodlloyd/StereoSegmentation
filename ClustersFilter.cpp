#include "ClustersFilter.h"

ClustersFilter::ClustersFilter(){}

void ClustersFilter::compute(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& clusters) {
    for (int i = 0; i < clusters.size(); i++) {
        float minX, minY, minZ, maxZ, maxX, maxY, memY, memZ;
        int nbrY, nbrZ, ancienY,plane;
        bool dontgo = false;
        bool zminbool = false;

        for (int j = 0; j < clusters.at(i)->points.size(); j++) {
            //pôur chaque points du cluster en question, eh ouais !
            float xpoint = clusters.at(i)->points[j].x;
            float ypoint = clusters.at(i)->points[j].y;
            float zpoint = clusters.at(i)->points[j].z;
            if (j == 0) {
                minX = xpoint;
                minY = ypoint;
                minZ = zpoint;
                maxX = xpoint;
                maxY = ypoint;
                maxZ = zpoint;
                memY = ypoint;
                memZ = zpoint;
                plane=0;
                nbrZ=0;
                ancienY = 0;
            }
            if (memZ != zpoint) {
                memZ = zpoint;
                nbrZ ++;
            }
            if (memY != ypoint) {
                if(ypoint >= memY -3 && ypoint <= memY +3){
                    plane ++;
                }
                memY = ypoint;
            }
        }
        if( plane >= nbrZ *0.95){
            clusters.erase(clusters.begin()+i-1);
        }
    }
}
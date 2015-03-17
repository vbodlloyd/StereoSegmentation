/*
    Copyright (C) 2014  Koichiro Yamaguchi

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "STVFlow.h"
#include <fstream>
#include <png++/png.hpp>
#include "SPSStereo.h"
#include "defParameter.h"
#include "StereoRectifier.h"
#include "DTSStereo.h"
#include "ExtractTheGround.h"
#include "EuclidianClusters.h"
#include "RGBSegmentation.h"
#include "ClustersFilter.h"


bool is_border(cv::Mat& edge, cv::Vec3b color)
{
	cv::Mat im = edge.clone().reshape(0,1);

	bool res = true;
	for (int i = 0; i < im.cols; ++i)
		res &= (color == im.at<cv::Vec3b>(0,i));

	return res;
}

void makeSegmentBoundaryImage(const png::image<png::rgb_pixel>& inputImage,
							  const png::image<png::gray_pixel_16>& segmentImage,
							  std::vector< std::vector<int> >& boundaryLabels,
							  png::image<png::rgb_pixel>& segmentBoundaryImage);
void writeDisparityPlaneFile(const std::vector< std::vector<double> >& disparityPlaneParameters, const std::string outputDisparityPlaneFilename);
void writeBoundaryLabelFile(const std::vector< std::vector<int> >& boundaryLabels, const std::string outputBoundaryLabelFilename);

boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "reconstruction");
	//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "reconstruction");
	viewer->addCoordinateSystem ( 1.0 );
	viewer->initCameraParameters ();
	return (viewer);
}


int main(int argc, char* argv[]) {
	//Load Matrix Q
	cv::FileStorage fs("/home/bodereau/Bureau/OpenCVReprojectImageToPointCloud/Q.xml", cv::FileStorage::READ);
	cv::Mat Q;

	pcl::visualization::CloudViewer viewer ("3D Viewer");

	fs["Q"] >> Q;

	//If size of Q is not 4x4 exit
	if (Q.cols != 4 || Q.rows != 4)
	{
		std::cerr << "ERROR: Could not read matrix Q (doesn't exist or size is not 4x4)" << std::endl;
		return 1;
	}


	//Get the interesting parameters from Q
	double Q03, Q13, Q23, Q32, Q33;
	Q03 = Q.at<double>(0,3);
	Q13 = Q.at<double>(1,3);
	Q23 = Q.at<double>(2,3);
	Q32 = Q.at<double>(3,2);
	Q33 = Q.at<double>(3,3);

	std::cout << "Q(0,3) = "<< Q03 <<"; Q(1,3) = "<< Q13 <<"; Q(2,3) = "<< Q23 <<"; Q(3,2) = "<< Q32 <<"; Q(3,3) = "<< Q33 <<";" << std::endl;

	cv::Size size(752, 480);
	vision::StereoRectifier rectifier(size);
	rectifier.load_intrinsic_params("/home/bodereau/Bureau/intrinsics.yml");
	rectifier.load_extrinsic_params("/home/bodereau/Bureau/extrinsics.yml");
	rectifier.stereo_rectify();
	rectifier.dump_rectification_info();
	int idx = 220;
	 //time
	clock_t start,other_clock,clock2,clock4;
	float time,time2,timtotal,time3,time4;
	timtotal = 0.0;
	start = clock();
	//~time
	cv::namedWindow("vigne");
	cv::namedWindow("bord");
	while(idx++ < 500) {
		//printf("boucle : %d \n",idx);
		std::string ref = std::to_string(idx);


		std::string leftImageFilename = "/home/bodereau/Bureau/tiff2/" + ref + "_l.tiff";

		std::string rightImageFilename = "/home/bodereau/Bureau/tiff2/" + ref + "_r.tiff";
		cv::Mat g1, g2;
		clock2 = clock();
		cv::Size size_rectified = rectifier.get_recified_size();
		cv::Mat3b mat_r = cv::imread(rightImageFilename);
		cv::Mat3b mat_l = cv::imread(leftImageFilename);
		cv::Mat3b rectified_l = cv::Mat3b::zeros(size_rectified);
		cv::Mat3b rectified_r = cv::Mat3b::zeros(size_rectified);
		rectifier.generate_rectified_mat(mat_l, mat_r, rectified_l, rectified_r);

		DTSStereo dtsStereo;
		/*int test = dtsStereo.computeLab(rectified_l.data, size_rectified.area(), size_rectified.width);
			int test2 = dtsStereo.compute(rectified_l.data, size_rectified.area(), size_rectified.width);*/
		int height_without_sky = dtsStereo.computeHisto(rectified_l, size_rectified.area(), size_rectified.width);

		cv::namedWindow("img test1", CV_WINDOW_AUTOSIZE);

		printf("height without sky %d \n", height_without_sky);
		clock2 = clock() - clock2;
		time3 = ((float) (clock2)) / CLOCKS_PER_SEC;
		printf("time for rectifie : %f  \n", time3);
		other_clock = clock();

		cv::cvtColor(rectified_l, g1, CV_BGR2GRAY);
		cv::cvtColor(rectified_r, g2, CV_BGR2GRAY);

		cv::Mat left_image_rectified_crop, right_image_rectified_crop;
		/*cv::Rect win(1, 339-test, 560, test);
		cv::Rect win2(1,339-test2,560,test2);*/
		cv::Rect win3(1, 339 - height_without_sky, 560, height_without_sky);
		rectified_l(win3).copyTo(left_image_rectified_crop);
		rectified_r(win3).copyTo(right_image_rectified_crop);



		//printf("convert to png::image left done \n");
		//png::image<png::rgb_pixel> rightImage("right.png");
		other_clock = clock() - other_clock;
		time2 = ((float) (other_clock)) / CLOCKS_PER_SEC;
		timtotal = timtotal + time2;

		printf("time lost in png make : %f \n", time2);
		//~time*/
		/*idxVect = 0;

		rightImage.write("rgb2.png");*/
		//printf("convert to png::image right done \n");
		SPSStereo sps;
		sps.setIterationTotal(outerIterationTotal, innerIterationTotal);
		sps.setWeightParameter(lambda_pos, lambda_depth, lambda_bou, lambda_smo);
		sps.setInlierThreshold(lambda_d);
		sps.setPenaltyParameter(lambda_hinge, lambda_occ, lambda_pen);

		cv::Mat segmentImage(600, 600, CV_16UC1);

		std::vector <std::vector<double>> disparityPlaneParameters;
		std::vector <std::vector<int>> boundaryLabels;
		//printf("go to compute \n");
		other_clock = clock();
		cv::Mat disparity(600, 600, CV_16UC1);
		sps.compute(superpixelTotal, left_image_rectified_crop, right_image_rectified_crop, segmentImage, disparity, disparityPlaneParameters, boundaryLabels);
		/*cv::StereoSGBM sgbm;
		sgbm.SADWindowSize = 5;
		sgbm.numberOfDisparities = 256;
		sgbm.preFilterCap = 0;
		sgbm.minDisparity = 0;
		sgbm.uniquenessRatio = 1;
		sgbm.speckleWindowSize = 150;
		sgbm.speckleRange = 2;
		sgbm.disp12MaxDiff = 10;
		sgbm.fullDP = false;
		sgbm.P1 = 1000;
		sgbm.P2 = 2400;
		cv::Mat disper;
		sgbm(dst, dst2, disper);
		normalize(disper, disparity, 0, 255, CV_MINMAX, CV_8U);*/
		other_clock = clock() - other_clock;
		time2 = ((float) (other_clock)) / CLOCKS_PER_SEC;
		printf("time to compute  : %f \n", time2);
		//printf("compute done \n");
		other_clock = clock();
		/*png::image<png::rgb_pixel> segmentBoundaryImage;
					makeSegmentBoundaryImage(dst, segmentImage, boundaryLabels, segmentBoundaryImage);
					segmentBoundaryImage.write(ref + "__bound.png");*/
		//disparityImage.write(ref + "__disparity.png");
		other_clock = clock() - other_clock;

		cv::imshow("vigne", disparity);
		/*cv::Mat copy;
		disparity.copyTo(copy);
		STVFlow stvflow;
		stvflow.compute(disparity,dst);*/
		//copy.convertTo(copy, CV_8U,1/255.0,1/255.0);
		/*cv::Mat dst455;
		cv::threshold(disparity,dst455,50, 255, cv::THRESH_BINARY);*/

		/*cv::imshow("bord",disparity);
		cv::threshold(copy,copy,100, 255, cv::THRESH_BINARY);
		cv::imshow("img test1",dst);*/
		cv::waitKey(100);
		cv::Mat img_disparity, img_rgb;
		img_disparity = disparity;
		img_rgb = left_image_rectified_crop;
		img_disparity.convertTo(img_disparity, CV_8U, 1 / 255.0, 1 / 255.0); //A REMETRE EN SPS
		//Create matrix that will contain 3D corrdinates of each pixel
		cv::Mat recons3D(img_disparity.size(), CV_32FC3);

		//Reproject image to 3D
		std::cout << "Reprojecting image to 3D..." << std::endl;
		cv::reprojectImageTo3D(img_disparity, recons3D, Q, false, CV_32F);

		std::cout << "Creating Point Cloud..." << std::endl;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_all_the_image_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

		double px, py, pz;
		uchar pr, pg, pb;
		clock4 = clock();

		for (int i = 0; i < img_rgb.rows; i++) {
			uchar *rgb_ptr = img_rgb.ptr<uchar>(i);

			uchar *disp_ptr = img_disparity.ptr<uchar>(i);

			//double* recons_ptr = recons3D.ptr<double>(i);
			for (int j = 0; j < img_rgb.cols; j++) {
				//Get 3D coordinates

				uchar d = disp_ptr[j];
				if (d == 0) continue; //Discard bad pixels
				double pw = -1.0 * static_cast<double>(d) * Q32 + Q33;
				px = static_cast<double>(j) + Q03;
				py = static_cast<double>(i) + Q13;
				pz = Q23;

				px = px / pw;
				py = py / pw;
				pz = pz / pw;

				//Get RGB info
				pb = rgb_ptr[3 * j];
				pg = rgb_ptr[3 * j + 1];
				pr = rgb_ptr[3 * j + 2];

				//Insert info into point cloud structure
				pcl::PointXYZRGB point;
				point.x = px;
				point.y = py;

				point.z =-1* pz;
				uint32_t rgb = (static_cast<uint32_t>(pr) << 16 |
						static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
				point.rgb = *reinterpret_cast<float *>(&rgb);
				//if(-1*pz < 90){
					point_cloud_all_the_image_rgb->points.push_back(point);//}
				pcl::PointXYZ pointx;
				pointx.x = px;
				pointx.y = py;
				pointx.z = -1*pz;
				if(-1*pz <90)
					cloud->points.push_back(pointx);
			}
		}

		point_cloud_all_the_image_rgb->width = (int) point_cloud_all_the_image_rgb->points.size();
		point_cloud_all_the_image_rgb->height = 1;
		cloud->width = (int) cloud->points.size();
		cloud->height = 1;
		clock4 = clock() - clock4;
		time4 = ((float) (clock4)) / CLOCKS_PER_SEC;
		printf("temps a faire les pointcloud : %f \n", time4);

		clock4 = clock();
		// Read in the cloud data
		pcl::PCDReader reader;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);


		//cloud = cloud_ptr;
		std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl; //*
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgbseg(new pcl::PointCloud<pcl::PointXYZRGB>);

		ExtractTheGround extractTheGround;
		extractTheGround.compute(cloud, cloud_filtered, cloud_ground);
		clock4 = clock() - clock4;
		time4 = ((float) (clock4)) / CLOCKS_PER_SEC;
		printf("temps a sortir le sol : %f \n", time4);
		clock4 = clock();

		std::cout << "ytyt" << std::endl;

		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;

		EuclidianClusters euclidianClusters;
		euclidianClusters.compute(cloud_filtered, clusters);

		clock4 = clock() - clock4;
		time4 = ((float) (clock4)) / CLOCKS_PER_SEC;
		printf("temps a faire la seg euclid : %f \n", time4);
		clock4 = clock();

		/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgbseg_less(new pcl::PointCloud<pcl::PointXYZRGB>);
		for(int i=0;i < cloud_filtered->points.size();i++){
			for(int j=0;j < point_cloud_ptr->points.size();j++){
				if(cloud_filtered->points[i].x == point_cloud_ptr->points[j].x && cloud_filtered->points[i].y == point_cloud_ptr->points[j].y &&
						cloud_filtered->points[i].z == point_cloud_ptr->points[j].z){
					cloud_rgbseg_less->points.push_back(point_cloud_ptr->points[j]);
					break;
				}
			}
		}
		RGBSegmentation rgbseg;
		rgbseg.compute(cloud_rgbseg_less, cloud_rgbseg);*/

		//viewer.removeVisualizationCallable("jhjh");
		viewer.removeVisualizationCallable("jhjha");
		viewer.showCloud(point_cloud_all_the_image_rgb,"jhjha");
		clock4 = clock() - clock4;
		time4 = ((float) (clock4)) / CLOCKS_PER_SEC;
		printf("temps afficher image: %f \n", time4);
		clock4 = clock();
		//viewer.showCloud(cloud_rgbseg,"jhjh");

		/*ClustersFilter clustersFilter;
		clustersFilter.compute(clusters);*/

		pcl::PointCloud<pcl::PointXYZ>::Ptr cube(new pcl::PointCloud<pcl::PointXYZ>);
		for(int i=0; i< clusters.size();i++) {
			float minX, minY, minZ,maxZ,maxX,maxY,memY,memZ;
			int nbrY,nbrZ;
			bool dontgo = false;
			bool zminbool = false;

			for(int j=0; j < clusters.at(i)->points.size();j++) {
				//pour chaque points du cluster en question, eh ouais !
				float xpoint = clusters.at(i)->points[j].x;
				float ypoint = clusters.at(i)->points[j].y;
				float zpoint = clusters.at(i)->points[j].z;
				if(j==0){
					minX = xpoint; minY = ypoint; minZ = zpoint;
					maxX = xpoint; maxY = ypoint; maxZ= zpoint;
					memY = ypoint; memZ = zpoint;
				}

				if(memZ == zpoint ){
					nbrZ++;
				}else {

					if (nbrZ >= 50) {
						maxZ = zpoint;
						if (!zminbool) {
							zminbool = true;
							minZ = zpoint;
						}
					}
					nbrZ = 1;
				}

				if(memY != ypoint)
					memY = ypoint;
				if(memZ != zpoint) {
					memZ = zpoint;
				}

				/*if(zpoint > maxZ)
					maxZ = zpoint;*/
				/*if(zpoint < minZ)
					minZ = zpoint;*/
				if(xpoint < minX)
					minX= xpoint;
				if(xpoint > maxX)
					maxX = xpoint;
				if( ypoint < minY)
					minY = ypoint;
				if( ypoint > maxY)
					maxY = ypoint;
				if(maxZ > (minZ +15))
					break;

			}
			if (maxX - minX > 3 * (maxY - minY))
				dontgo = true;
			if(maxZ - minZ < 3)
				dontgo = true;
			if(dontgo)
				continue;

			float xdebut = minX, ydebut = minY, zdebut = minZ, xfin = maxX, yfin = maxY, zfin = maxZ;
			//float xdebut = -10.20, ydebut = 0, zdebut = 12.20, xfin = -50.40, yfin = 15.24, zfin = -13.56;
			if (ydebut > yfin) {
				float tmp = ydebut;
				ydebut = yfin;
				yfin = tmp;
			}
			if (xdebut > xfin) {
				float tmp = xdebut;
				xdebut = xfin;
				xfin = tmp;
			}
			if (zdebut > zfin) {
				float tmp = zdebut;
				zdebut = zfin;
				zfin = tmp;
			}

			for (float i = xdebut; i <= xfin; i+=0.3) {
				for (float j = ydebut; j <= yfin; j+=0.3) {

					pcl::PointXYZ p;
					pcl::PointXYZ p2;
					p.x = i;
					if (i == xdebut || (i >= xfin - 0.3 && i <= xfin + 0.3)) {
						p.y = j;

					}
					else {
						p.y = ydebut;

						p2.y = yfin;
					}
					p.z = zfin;

					p2.x = p.x;
					p2.z = p.z;
					cube->points.push_back(p);
					cube->points.push_back(p2);
					p.z = zdebut;
					p2.z = zdebut;
					cube->points.push_back(p);
					cube->points.push_back(p2);
				}
			}
			for(float j=ydebut; j <=yfin;j+=0.3){
			for(float z =zdebut;z<=zfin;z+=0.3){
				pcl::PointXYZ p;
				pcl::PointXYZ p2;
				p.x=xdebut;
				p2.x=xfin;
				p.y=j;
				p.z=z;
				p2.y=p.y;
				p2.z=p.z;
				if (j == ydebut || (j>= yfin - 0.3 && j <= yfin + 0.3)) {

					cube->points.push_back(p);
					cube->points.push_back(p2);
				}
			}
		}
		}

		viewer.showCloud (cube,"oh");

		/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb_memory2 (new pcl::PointCloud<pcl::PointXYZRGB>);
		int r2=15,b2=255,g3r=125;
		for(int i = 0; i < clusters.size();i++){
			uint32_t rgb = (static_cast<uint32_t>(r2) << 16 |
					static_cast<uint32_t>(g3r) << 8 | static_cast<uint32_t>(b2));
			for(int j=0;j< clusters.at(i)->points.size();j++){
				clusters.at(i)->points[j].rgb = *reinterpret_cast<float*>(&rgb);
			}
			r2+=20;
			b2+=100;
			g3r+=5;
			if(r2 >252)
				r2=0;
			if(g3r>252)
				g3r=0;
			if(b2>253)
				b2=0;
			*cloud_rgb_memory2 += *clusters.at(i);
		}
		clock4 = clock() - clock4;
		time4 = ((float) (clock4)) / CLOCKS_PER_SEC;
		printf("temps a faire les clusters avant : %f \n", time4);
		clock4 = clock();
		std::cout<<"clusters size :"<<clusters.size() <<std::endl;
		//viewer.showCloud (cloud_rgb_memory2,"eh eh eh");
		clock4 = clock() - clock4;
		time4 = ((float) (clock4)) / CLOCKS_PER_SEC;
		printf("temps a faire les clusters aprés : %f \n", time4);
		clock4 = clock();*/
	}

	//time
	start = clock() - start;
	time = ((float) ( start))/ CLOCKS_PER_SEC;
	printf("time for all : %f and time without loss :%f \n",time,time - timtotal);
}

void writeDisparityPlaneFile(const std::vector< std::vector<double> >& disparityPlaneParameters, const std::string outputDisparityPlaneFilename) {
	std::ofstream outputFileStream(outputDisparityPlaneFilename.c_str(), std::ios_base::out);
	if (outputFileStream.fail()) {
		std::cerr << "error: can't open file (" << outputDisparityPlaneFilename << ")" << std::endl;
		exit(0);
	}

	int segmentTotal = static_cast<int>(disparityPlaneParameters.size());
	for (int segmentIndex = 0; segmentIndex < segmentTotal; ++segmentIndex) {
		outputFileStream << disparityPlaneParameters[segmentIndex][0] << " ";
		outputFileStream << disparityPlaneParameters[segmentIndex][1] << " ";
		outputFileStream << disparityPlaneParameters[segmentIndex][2] << std::endl;
	}

	outputFileStream.close();
}

void writeBoundaryLabelFile(const std::vector< std::vector<int> >& boundaryLabels, const std::string outputBoundaryLabelFilename) {
	std::ofstream outputFileStream(outputBoundaryLabelFilename.c_str(), std::ios_base::out);
	if (outputFileStream.fail()) {
		std::cerr << "error: can't open output file (" << outputBoundaryLabelFilename << ")" << std::endl;
		exit(1);
	}

	int boundaryTotal = static_cast<int>(boundaryLabels.size());
	for (int boundaryIndex = 0; boundaryIndex < boundaryTotal; ++boundaryIndex) {
		outputFileStream << boundaryLabels[boundaryIndex][0] << " ";
		outputFileStream << boundaryLabels[boundaryIndex][1] << " ";
		outputFileStream << boundaryLabels[boundaryIndex][2] << std::endl;
	}
	outputFileStream.close();
}



//=================================================================================================
//
//  Copyright(c)  2013  Jean Inderchit
//
//  Vitals is free software: you can redistribute it and/or modify it under the terms of the GNU
//	General Public License as published by the Free Software Foundation, either version 3 of the
//	License, or (at your option) any later version.
//
//  Vitals is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
//	even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along with Vitals. If not,
//	see <http://www.gnu.org/licenses/>.
//
//=================================================================================================

//=================================================================================================
// I N C L U D E   F I L E S

#include "StereoRectifier.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/contrib/contrib.hpp>

//=================================================================================================
// C O N S T A N T S   &   L O C A L   V A R I A B L E S

//=================================================================================================
// G L O B A L S

//=================================================================================================
// C O N S T R U C T O R (S) / D E S T R U C T O R   C O D E   S E C T I O N

vision::StereoRectifier::StereoRectifier(const cv::Size& src_size) :
		src_size_(src_size) {
}

//-------------------------------------------------------------------------------------------------
//
vision::StereoRectifier::~StereoRectifier() {
}

//=================================================================================================
// M E T H O D S   C O D E   S E C T I O N

//-------------------------------------------------------------------------------------------------
//
void vision::StereoRectifier::load_intrinsic_params(const std::string& path) {

	// Reading intrinsic parameters
	cv::FileStorage fs(path, CV_STORAGE_READ);

	if (!fs.isOpened()) {
		throw std::runtime_error("hh");
	}

	fs["M1"] >> M1_;
	fs["D1"] >> D1_;
	fs["M2"] >> M2_;
	fs["D2"] >> D2_;
}

//-------------------------------------------------------------------------------------------------
//
void vision::StereoRectifier::load_extrinsic_params(const std::string& path) {
	// Reading intrinsic parameters
	cv::FileStorage fs(path, CV_STORAGE_READ);

	if (!fs.isOpened()) {
		throw std::runtime_error("jhfjfh");
	}

	fs["R"] >> R_;
	fs["T"] >> T_;
	fs["R1"] >> R1_;
	fs["R2"] >> R2_;
	fs["P1"] >> P1_;
	fs["P2"] >> P2_;
	fs["Q"] >> Q_;
}

//-------------------------------------------------------------------------------------------------
//
void vision::StereoRectifier::stereo_rectify() {
	cv::stereoRectify(M1_, D1_, M2_, D2_, src_size_, R_, T_, R1_, R2_, P1_, P2_,
			Q_, cv::CALIB_ZERO_DISPARITY, 1, cv::Size(), &roi1_, &roi2_);

	validRoi_ = roi1_ & roi2_;
	newSize_ = cv::Size(validRoi_.width, validRoi_.height);

	initUndistortRectifyMap(M1_, D1_, R1_, P1_, src_size_, CV_16SC2,
			output_map1_[0], output_map1_[1]);

	initUndistortRectifyMap(M2_, D2_, R2_, P2_, src_size_, CV_16SC2,
			output_map2_[0], output_map2_[1]);
}

//-------------------------------------------------------------------------------------------------
//
void vision::StereoRectifier::generate_rectified_mat(const cv::Mat3b& src_mat1,
		const cv::Mat3b& src_mat2, cv::Mat3b& rectified_mat1,
		cv::Mat3b& rectified_mat2) {
	remap(src_mat1, rectified_mat1, output_map1_[0], output_map1_[1],
			cv::INTER_LINEAR);
	remap(src_mat2, rectified_mat2, output_map2_[0], output_map2_[1],
			cv::INTER_LINEAR);

	rectified_mat1 = cv::Mat3b(rectified_mat1, validRoi_);
	rectified_mat2 = cv::Mat3b(rectified_mat2, validRoi_);
}

//-------------------------------------------------------------------------------------------------
//
void vision::StereoRectifier::dump_rectification_info() const {

	printf("\nRectified Parameters:");
		printf("\t- Resolution: %d by %d", newSize_.width, newSize_.height);
		printf("\t- Principal Point Camera 1: %f , %f ", P1_.at<double>(0, 2),
				P1_.at<double>(1, 2));
		printf("\t- Principal Point Camera 2: %f , %f ", P2_.at<double>(0, 2),
				P2_.at<double>(1, 2));
		printf("");

	printf("\nOriginal Parameters:");
	printf("\t- Resolution: %d by %d", src_size_.width, src_size_.height);
	printf("\t- Principal Point Camera 1: %f , %f", M1_.at<double>(0, 2),
			M1_.at<double>(1, 2));
	printf("\t- Principal Point Camera 2: %f , %f ", M2_.at<double>(0, 2),
			M2_.at<double>(1, 2));
	printf("\t- Focal length: %f ", M1_.at<double>(0, 0));

}

//-------------------------------------------------------------------------------------------------
//
const cv::Size&
vision::StereoRectifier::get_recified_size() const {
	return newSize_;
}

//-------------------------------------------------------------------------------------------------
//
double vision::StereoRectifier::get_focal_length() const {
	return static_cast<uint32_t>(P2_.at<double>(0, 0));
}

//-------------------------------------------------------------------------------------------------
//
cv::Point2i vision::StereoRectifier::get_principal_point() const {
	cv::Point2i tmp { };

	tmp.x = (P2_.at<double>(0, 2)) - validRoi_.x;
	tmp.y = (P2_.at<double>(1, 2)) - validRoi_.y;

	return tmp;
}

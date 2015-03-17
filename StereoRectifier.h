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

#ifndef STEREORECTIFIER_H
#define STEREORECTIFIER_H

//=================================================================================================
// I N C L U D E   F I L E S

#include <opencv2/core/core.hpp>


//=================================================================================================
// F O R W A R D   D E C L A R A T I O N S

//=================================================================================================
// C O N S T A N T S

//=================================================================================================
// C L A S S E S

namespace vision
{

/// Class used for calibrating bench of stereo cameras. Information about the camera calibration
/// (like the principal point, the focal length...) can be retrieved after loading the calibration
/// file.
///
class StereoRectifier
{
//--Methods----------------------------------------------------------------------------------------
private:

public:
	StereoRectifier( const cv::Size& src_size );
	~StereoRectifier();

	void load_intrinsic_params( const std::string& path );
	void load_extrinsic_params( const std::string& path );
	void stereo_rectify();

	void generate_rectified_mat( const cv::Mat3b& src_mat1, const cv::Mat3b& src_mat2,
	                             cv::Mat3b& rectified_mat1, cv::Mat3b& rectified_mat2 );

	void dump_rectification_info() const;

	const cv::Size& get_recified_size() const;
	double get_focal_length() const;
	cv::Point2i get_principal_point() const;

//--Data members-----------------------------------------------------------------------------------
private:
	const cv::Size& src_size_;

	// Intrinsic parameters
	cv::Mat M1_;    // First camera matrix
	cv::Mat M2_;    // Second camera matrix

	cv::Mat D1_;    // First camera distortion coefficients (8)
	cv::Mat D2_;    // Second camera distortion coefficients (8)

	// Extrinsic parameters
	cv::Mat R_;     // Rotation matrix between the coordinate systems of the first and the second cameras.
	cv::Mat T_;     // Translation vector between coordinate systems of the cameras.

	cv::Mat R1_;    // Output 3x3 rectification transform (rotation matrix) for the first camera.
	cv::Mat R2_;    // Output 3x3 rectification transform (rotation matrix) for the second camera.

	cv::Mat P1_;    // Output 3x4 projection matrix in the new (rectified) coordinate systems for the first camera.
	cv::Mat P2_;    // Output 3x4 projection matrix in the new (rectified) coordinate systems for the second camera.

	cv::Mat Q_;     // Output 4*4 disparity-to-depth mapping matrix (see reprojectImageTo3D() ).

	cv::Rect roi1_;     // Rectangles inside the rectified images where all the pixels are valid.
	cv::Rect roi2_;     // Rectangles inside the rectified images where all the pixels are valid.

	cv::Rect validRoi_;

	cv::Size newSize_; // New image resolution after rectification.

	std::array<cv::Mat, 2> output_map1_;   // Output maps of the first camera
	std::array<cv::Mat, 2> output_map2_;   // Output map of the second camera
};

}

//=================================================================================================
// T E M P L A T E S   C O D E   S E C T I O N

//=================================================================================================
// I N L I N E   F U N C T I O N S   C O D E   S E C T I O N

#endif  // STEREORECTIFIER_H

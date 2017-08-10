
#include <iostream>
#include <sstream>

#include "EyeTracking_2D3D.h"
#include "Mesh.h"

#include <opencv2/calib3d/calib3d.hpp>
#include "Utils.h"

/* Functions headers */
cv::Point3f CROSS(cv::Point3f v1, cv::Point3f v2);
double DOT(cv::Point3f v1, cv::Point3f v2);
cv::Point3f SUB(cv::Point3f v1, cv::Point3f v2);
cv::Point3f get_nearest_3D_point(std::vector<cv::Point3f> &points_list, cv::Point3f origin);

/* Functions for Möller–Trumbore intersection algorithm */

cv::Point3f CROSS(cv::Point3f v1, cv::Point3f v2)
{
	cv::Point3f tmp_p;
	tmp_p.x = v1.y*v2.z - v1.z*v2.y;
	tmp_p.y = v1.z*v2.x - v1.x*v2.z;
	tmp_p.z = v1.x*v2.y - v1.y*v2.x;
	return tmp_p;
}

double DOT(cv::Point3f v1, cv::Point3f v2)
{
	return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

cv::Point3f SUB(cv::Point3f v1, cv::Point3f v2)
{
	cv::Point3f tmp_p;
	tmp_p.x = v1.x - v2.x;
	tmp_p.y = v1.y - v2.y;
	tmp_p.z = v1.z - v2.z;
	return tmp_p;
}

/* End functions for Möller–Trumbore intersection algorithm
*  */

// Function to get the nearest 3D point to the Ray origin
cv::Point3f get_nearest_3D_point(std::vector<cv::Point3f> &points_list, cv::Point3f origin)
{
	cv::Point3f p1 = points_list[0];
	cv::Point3f p2 = points_list[1];

	double d1 = std::sqrt(std::pow(p1.x - origin.x, 2) + std::pow(p1.y - origin.y, 2) + std::pow(p1.z - origin.z, 2));
	double d2 = std::sqrt(std::pow(p2.x - origin.x, 2) + std::pow(p2.y - origin.y, 2) + std::pow(p2.z - origin.z, 2));

	if (d1 < d2)
	{
		return p1;
	}
	else
	{
		return p2;
	}
}

	EyeTracking3D::EyeTracking3D(const cv::Mat intrinsics, const cv::Mat distortion)
	{
		intrinsics.copyTo(_A_matrix);
		_A_matrix.convertTo(_A_matrix, CV_32F);
		distortion.copyTo(distCoeffs);
		distCoeffs.convertTo(distCoeffs, CV_32F);

		//_A_matrix.at<float>(0, 0) = intrinsics[0];       //      [ fx   0  cx ]
		//_A_matrix.at<float>(1, 1) = intrinsics[1];       //      [  0  fy  cy ]
		//_A_matrix.at<float>(0, 2) = intrinsics[2];       //      [  0   0   1 ]
		//_A_matrix.at<float>(1, 2) = intrinsics[3];
		//_A_matrix.at<float>(2, 2) = 1;
		_R_matrix = cv::Mat::zeros(3, 3, CV_32F);   // rotation matrix
		_t_matrix = cv::Mat::zeros(3, 1, CV_32F);   // translation matrix
		_P_matrix = cv::Mat::zeros(3, 4, CV_32F);   // rotation-translation matrix
		pose = cv::Mat::zeros(4, 4, CV_32F);   // rotation-translation matrix
	}

	EyeTracking3D::~EyeTracking3D()
	{
		// TODO Auto-generated destructor stub
	}

	void EyeTracking3D::set_P_matrix(const cv::Mat &R_matrix, const cv::Mat &t_matrix)
	{
		// Rotation-Translation Matrix Definition
		R_matrix.copyTo(_P_matrix.rowRange(0, 3).colRange(0, 3));
		t_matrix.copyTo(_P_matrix.rowRange(0, 3).col(3));
        pose = cv::Mat::eye(4, 4, CV_32F);
        _P_matrix.copyTo(pose.rowRange(0, 3).colRange(0, 4));

		//_P_matrix.at<float>(0, 0) = R_matrix.at<float>(0, 0);
		//_P_matrix.at<float>(0, 1) = R_matrix.at<float>(0, 1);
		//_P_matrix.at<float>(0, 2) = R_matrix.at<float>(0, 2);
		//_P_matrix.at<float>(1, 0) = R_matrix.at<float>(1, 0);
		//_P_matrix.at<float>(1, 1) = R_matrix.at<float>(1, 1);
		//_P_matrix.at<float>(1, 2) = R_matrix.at<float>(1, 2);
		//_P_matrix.at<float>(2, 0) = R_matrix.at<float>(2, 0);
		//_P_matrix.at<float>(2, 1) = R_matrix.at<float>(2, 1);
		//_P_matrix.at<float>(2, 2) = R_matrix.at<float>(2, 2);
		//_P_matrix.at<float>(0, 3) = t_matrix.at<float>(0);
		//_P_matrix.at<float>(1, 3) = t_matrix.at<float>(1);
		//_P_matrix.at<float>(2, 3) = t_matrix.at<float>(2);

		//std::cout << "Current P: " << std::endl;
		//std::cout << _P_matrix << std::endl;
		set_RT_matrices_fromP();

	}

	void EyeTracking3D::set_RT_matrices_fromP()
	{
		//std::cout << "test" << std::endl;
		_R_matrix = _P_matrix.rowRange(0, 3).colRange(0, 3);
		_t_matrix =  _P_matrix.rowRange(0, 3).col(3);

		//_R_matrix.at<float>(0, 0) = _P_matrix.at<float>(0, 0);
		//_R_matrix.at<float>(0, 1) = _P_matrix.at<float>(0, 1);
		//_R_matrix.at<float>(0, 2) = _P_matrix.at<float>(0, 2);
		//_R_matrix.at<float>(1, 0) = _P_matrix.at<float>(1, 0);
		//_R_matrix.at<float>(1, 1) = _P_matrix.at<float>(1, 1);
		//_R_matrix.at<float>(1, 2) = _P_matrix.at<float>(1, 2);
		//_R_matrix.at<float>(2, 0) = _P_matrix.at<float>(2, 0);
		//_R_matrix.at<float>(2, 1) = _P_matrix.at<float>(2, 1);
		//_R_matrix.at<float>(2, 2) = _P_matrix.at<float>(2, 2);
		//_t_matrix.at<float>(0) = _P_matrix.at<float>(0, 3);
		//_t_matrix.at<float>(1) = _P_matrix.at<float>(1, 3);
		//_t_matrix.at<float>(2) = _P_matrix.at<float>(2, 3);
	}
	// *******************************************************************************************************************************
	// Backproject a 3D point to 2D using the estimated pose parameters

	cv::Point2f EyeTracking3D::backproject3DPoint(const cv::Point3f &point3d)
	{

		// 3D point vector [x y z 1]'
		cv::Mat point3d_vec = cv::Mat(4, 1, CV_32F);
		point3d_vec.at<float>(0) = point3d.x;
		point3d_vec.at<float>(1) = point3d.y;
		point3d_vec.at<float>(2) = point3d.z;
		point3d_vec.at<float>(3) = 1;

		// 2D point vector [u v 1]'
		cv::Mat point2d_vec = cv::Mat(4, 1, CV_32F);

		point2d_vec = _A_matrix * _P_matrix * point3d_vec;

		// Normalization of [u v]'
		cv::Point2f point2d;
		point2d.x = (float)(point2d_vec.at<float>(0) / point2d_vec.at<float>(2));
		point2d.y = (float)(point2d_vec.at<float>(1) / point2d_vec.at<float>(2));

		//std::cout << point3d << std::endl;

		return point2d;
	}



	// Back project a 2D point to 3D and returns if it's on the mesh surface
	bool EyeTracking3D::backproject2DPoint(const std::vector<std::vector<cv::Point3f> > trianglesVertices, const cv::Point2f &point2d, cv::Point3f &point3d)
	{
		// Triangles list of the object mesh
		//std::vector<std::vector<int> > triangles_list = mesh->getTrianglesList();
		//std::cout << _P_matrix << std::endl;
		//std::cout << _R_matrix << std::endl;
		//std::cout << _t_matrix << std::endl;

		double lambda = 8;
		double u = point2d.x;
		double v = point2d.y;

		// Point in vector form
		cv::Mat point2d_vec;
        point2d_vec = cv::Mat::ones(3, 1, CV_32F); // 3x1

		point2d_vec.at<float>(0) = u * lambda;
		point2d_vec.at<float>(1) = v * lambda;
		point2d_vec.at<float>(2) = lambda;

		// Point in camera coordinates
		cv::Mat X_c;
//        _A_matrix = _A_matrix.inv();
        X_c = _A_matrix.inv() * point2d_vec; // 3x1

		// Point in world coordinates
		cv::Mat X_w = _R_matrix.inv() * (X_c - _t_matrix); // 3x1

		/*std::cout << X_w << std::endl;*/

		// Center of projection
		cv::Mat C_op = cv::Mat(_R_matrix.inv()).mul(-1) * _t_matrix; // 3x1

		// Ray direction vector
		cv::Mat ray = X_w - C_op; // 3x1
		ray = ray / cv::norm(ray); // 3x1

		// Set up Ray
		Ray R((cv::Point3f)C_op, (cv::Point3f)ray);

		// A vector to store the intersections found
		std::vector<cv::Point3f> intersections_list;

		//std::cout << _A_matrix << std::endl << std::endl;
		//std::cout << _R_matrix << std::endl << std::endl;
		//std::cout << _t_matrix << std::endl << std::endl;
		// Loop for all the triangles and check the intersection
		for (unsigned int i = 0; i < trianglesVertices.size(); i++)
		{
			cv::Point3f V0 = trianglesVertices[i][0];
			cv::Point3f V1 = trianglesVertices[i][1];
			cv::Point3f V2 = trianglesVertices[i][2];

			Triangle T(i, V0, V1, V2);

			double out;
			if (this->intersect_MollerTrumbore(R, T, &out))
			{
				cv::Point3f tmp_pt = R.getP0() + out*R.getP1(); // P = O + t*D
				intersections_list.push_back(tmp_pt);
			}
		}

		//std::cout << intersections_list << std::endl;
		// If there are intersection, find the nearest one
		if (!intersections_list.empty())
		{
			point3d = get_nearest_3D_point(intersections_list, R.getP0());
			return true;
		}
		else
		{
			return false;
		}


	}

	// Möller–Trumbore intersection algorithm
	bool EyeTracking3D::intersect_MollerTrumbore(Ray &Ray, Triangle &Triangle, double *out)
	{
		const double EPSILON = 0.000001;

		cv::Point3f e1, e2;
		cv::Point3f P, Q, T;
		double det, inv_det, u, v;
		double t;

		cv::Point3f V1 = Triangle.getV0();  // Triangle vertices
		cv::Point3f V2 = Triangle.getV1();
		cv::Point3f V3 = Triangle.getV2();

		cv::Point3f O = Ray.getP0(); // Ray origin
		cv::Point3f D = Ray.getP1(); // Ray direction

		//Find vectors for two edges sharing V1
		e1 = SUB(V2, V1);
		e2 = SUB(V3, V1);

		// Begin calculation determinant - also used to calculate U parameter
		P = CROSS(D, e2);

		// If determinant is near zero, ray lie in plane of triangle
		det = DOT(e1, P);

		//NOT CULLING
		if (det > -EPSILON && det < EPSILON) return false;
		inv_det = 1.f / det;

		//calculate distance from V1 to ray origin
		T = SUB(O, V1);

		//Calculate u parameter and test bound
		u = DOT(T, P) * inv_det;

		//The intersection lies outside of the triangle
		if (u < 0.f || u > 1.f) return false;

		//Prepare to test v parameter
		Q = CROSS(T, e1);

		//Calculate V parameter and test bound
		v = DOT(D, Q) * inv_det;

		//The intersection lies outside of the triangle
		if (v < 0.f || u + v  > 1.f) return false;

		t = DOT(e2, Q) * inv_det;

		if (t > EPSILON) { //ray intersection
			*out = t;
			return true;
		}

		// No hit, no win
		return false;
	}

// Estimate the pose given a list of 2D/3D correspondences with RANSAC and the method to use
void EyeTracking3D::estimatePoseRANSAC( const std::vector<cv::Point3f> &list_points3d,        // list with model 3D coordinates
									 const std::vector<cv::Point2f> &list_points2d,        // list with scene 2D coordinates
									 int flags, cv::Mat &inliers, int iterationsCount,     // PnP method; inliers container
									 float reprojectionError, float confidence )           // Ransac parameters
{

	cv::Mat rvec; rvec = cv::Mat::zeros(3, 1, CV_32F);          // output rotation vector
	cv::Mat tvec; tvec = cv::Mat::zeros(3, 1, CV_32F);          // output translation vector
	bool useExtrinsicGuess = false;   // if true the function uses the provided rvec and tvec values as
	// initial approximations of the rotation and translation vectors

	cv::solvePnPRansac( list_points3d, list_points2d, _A_matrix, distCoeffs, rvec, tvec,
						useExtrinsicGuess, iterationsCount, reprojectionError, confidence,
						inliers, flags );
	Rodrigues(rvec,_R_matrix);                   // converts Rotation Vector to Matrix
	_t_matrix = tvec;                            // set translation matrix

//    std::cout << "list_points3d: " << list_points3d << std::endl;
//    std::cout << "list_points2d: " << list_points2d << std::endl;
//    std::cout << "_A_matrix: " << _A_matrix << std::endl;
//    std::cout << "rvec: " << rvec << std::endl;
//    std::cout << "tvec: " << tvec << std::endl;
//    std::cout << "_R_matrix: " << _R_matrix << std::endl;

	this->set_P_matrix(_R_matrix, _t_matrix);    // set rotation-translation matrix
}

//void EyeTracking3D::Estimate3Dfixation(cv::Point2f point2D)
//{
//    bool intersection_found = backproject2DPoint(trianglesVertices, point2D, current_3Dfixation);
//
//}

void EyeTracking3D::initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt)
{

    KF.init(nStates, nMeasurements, nInputs, CV_32F);                 // init Kalman Filter

    setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));       // set process noise
    setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-2));   // set measurement noise
    setIdentity(KF.errorCovPost, cv::Scalar::all(1));             // error covariance


    /** DYNAMIC MODEL **/

    //  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]

    // position
    KF.transitionMatrix.at<double>(0,3) = dt;
    KF.transitionMatrix.at<double>(1,4) = dt;
    KF.transitionMatrix.at<double>(2,5) = dt;
    KF.transitionMatrix.at<double>(3,6) = dt;
    KF.transitionMatrix.at<double>(4,7) = dt;
    KF.transitionMatrix.at<double>(5,8) = dt;
    KF.transitionMatrix.at<double>(0,6) = 0.5*pow(dt,2);
    KF.transitionMatrix.at<double>(1,7) = 0.5*pow(dt,2);
    KF.transitionMatrix.at<double>(2,8) = 0.5*pow(dt,2);

    // orientation
    KF.transitionMatrix.at<double>(9,12) = dt;
    KF.transitionMatrix.at<double>(10,13) = dt;
    KF.transitionMatrix.at<double>(11,14) = dt;
    KF.transitionMatrix.at<double>(12,15) = dt;
    KF.transitionMatrix.at<double>(13,16) = dt;
    KF.transitionMatrix.at<double>(14,17) = dt;
    KF.transitionMatrix.at<double>(9,15) = 0.5*pow(dt,2);
    KF.transitionMatrix.at<double>(10,16) = 0.5*pow(dt,2);
    KF.transitionMatrix.at<double>(11,17) = 0.5*pow(dt,2);


    /** MEASUREMENT MODEL **/

    //  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]

    KF.measurementMatrix.at<double>(0,0) = 1;  // x
    KF.measurementMatrix.at<double>(1,1) = 1;  // y
    KF.measurementMatrix.at<double>(2,2) = 1;  // z
    KF.measurementMatrix.at<double>(3,9) = 1;  // roll
    KF.measurementMatrix.at<double>(4,10) = 1; // pitch
    KF.measurementMatrix.at<double>(5,11) = 1; // yaw

}

/**********************************************************************************************************/
void EyeTracking3D::updateKalmanFilter( cv::KalmanFilter &KF, cv::Mat &measurement, cv::Mat &translation_estimated, cv::Mat &rotation_estimated )
{

    // First predict, to update the internal statePre variable
    cv::Mat prediction = KF.predict();

    // The "correct" phase that is going to use the predicted value and our measurement
    cv::Mat estimated = KF.correct(measurement);

    // Estimated translation
    translation_estimated.at<double>(0) = estimated.at<double>(0);
    translation_estimated.at<double>(1) = estimated.at<double>(1);
    translation_estimated.at<double>(2) = estimated.at<double>(2);

    // Estimated euler angles
    cv::Mat eulers_estimated(3, 1, CV_32F);
    eulers_estimated.at<double>(0) = estimated.at<double>(9);
    eulers_estimated.at<double>(1) = estimated.at<double>(10);
    eulers_estimated.at<double>(2) = estimated.at<double>(11);

    // Convert estimated quaternion to rotation matrix
    rotation_estimated = euler2rot(eulers_estimated);

}

/**********************************************************************************************************/
void EyeTracking3D::fillMeasurements( cv::Mat &measurements, const cv::Mat &translation_measured, const cv::Mat &rotation_measured)
{
    // Convert rotation matrix to euler angles
    cv::Mat measured_eulers(3, 1, CV_32F);
    measured_eulers = rot2euler(rotation_measured);

    // Set measurement to predict
    measurements.at<double>(0) = translation_measured.at<double>(0); // x
    measurements.at<double>(1) = translation_measured.at<double>(1); // y
    measurements.at<double>(2) = translation_measured.at<double>(2); // z
    measurements.at<double>(3) = measured_eulers.at<double>(0);      // roll
    measurements.at<double>(4) = measured_eulers.at<double>(1);      // pitch
    measurements.at<double>(5) = measured_eulers.at<double>(2);      // yaw
}


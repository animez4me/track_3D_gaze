
#include <fstream>  // NOLINT
#include <iomanip>
#include <iostream>  //NOLINT
#include <list>
#include <vector>
#include <map>
#include <stdexcept>
#include <chrono>


#include <brisk/brisk.h>
#include <brisk/brute-force-matcher.h>
#include <brisk/command-line-parser.h>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Kinect
//#include "libfreenect2/k2g.h"
//#include <pcl/visualization/cloud_viewer.h>

#include "RGB.h"
#include "RGBD.h"
#include "matcher.h"

//#include "System.h"
//#include "ETG.h"
//#include "OptiTrack.h"
//#include "Utils.h"
#include "EyeTracking_2D3D.h"

//#include "track.h"

//save compiler switches
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//restore compiler switches
#pragma GCC diagnostic pop

bool show_kinect = true;
bool show_mesh = false;

//cv::Point3f fixed_pt;
std::string RGBD_CAMERA_intrinsics = "/home/mike/catkin_ws/src/track_3D_gaze/calib_RGBD.yaml";
std::string RGB_CAMERA_intrinsics = "/home/mike/catkin_ws/src/track_3D_gaze/calib_RGB.xml";
int counter = 0;
cv::Point2f testpoint = cv::Point2f(0,0);
bool new_click = false;

struct EyeTracker_PoR {
    unsigned int frame;
    cv::Point2f PoR;
    bool is_fixation;
    bool is_saccade;
    float speed;
};

void PointsSelectionCallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        testpoint = cv::Point2f(x, y);
        new_click=true;
    }
}
//cv::Mat frame;

//class ImageConverter
//{
//  ros::NodeHandle nh_;
//  image_transport::ImageTransport it_;
//  image_transport::Subscriber image_color_sub_;
//  image_transport::Subscriber image_depth_sub_;
//  image_transport::Publisher image_pub_;


//public:
//  ImageConverter()
//    : it_(nh_)
//  {
//    // Subscrive to input video feed and publish output video feed
////    image_color_sub_ = it_.subscribe("/kinect2/qhd/image_color_rect", 1,
////      &ImageConverter::imageColorCb, this);
////    image_depth_sub_ = it_.subscribe("/kinect2/qhd/image_depth_rect", 1,
////      &ImageConverter::imageDepthCb, this);
//    //image_pub_ = it_.advertise("/image_converter/output_video", 1);

//  }

//  ~ImageConverter()
//  {

//  }

//  void imageColorCb(const sensor_msgs::ImageConstPtr& msg)
//  {
//    cv_bridge::CvImagePtr cv_color_ptr;
//    try
//    {
//      ROS_INFO_STREAM("hee");
//      cv_color_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//      //frame = cv_color_ptr->image;
//      //return cv_color_ptr->image;
//    }
//    catch (cv_bridge::Exception& e)
//    {
//      ROS_ERROR("cv_bridge exception: %s", e.what());
//      return;
//    }
//  }
////  void imageDepthCb(const sensor_msgs::ImageConstPtr& msg)
////  {
////    cv_bridge::CvImagePtr cv_depth_ptr;
////    try
////    {
////      cv_depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
////    }
////    catch (cv_bridge::Exception& e)
////    {
////      ROS_ERROR("cv_bridge exception: %s", e.what());
////      return;
////    }
////  }
//};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "track_3D_gaze_node");
  ros::NodeHandle n;
  ros::Publisher fixation_pub = n.advertise<geometry_msgs::Pose>("fixation_3d", 1000);

//  ros::NodeHandle nh_;
//  image_transport::ImageTransport it_;
//  image_transport::Subscriber image_color_sub_;
//  image_transport::Subscriber image_depth_sub_;
//  image_transport::Publisher image_pub_;

//  image_color_sub_ = it_.subscribe("/kinect2/qhd/image_color_rect",1,imageColorCb);



    RGBDcamera kinect(RGBD_CAMERA_intrinsics);
    RGBcamera camera(RGB_CAMERA_intrinsics);
    matcher briskf;
    EyeTracking3D et3d(camera.camMatrix, camera.distCoeffs);

// Kalman Filter parameters
    cv::KalmanFilter KF;         // instantiate Kalman Filter
    int nStates = 18;            // the number of states
    int nMeasurements = 6;       // the number of measured states
    int nInputs = 0;             // the number of control actions
    double dt = 0.125;           // time between measurements (1/FPS)

    et3d.initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);    // init function
    cv::Mat measurements(nMeasurements, 1, CV_32F); measurements.setTo(cv::Scalar(0));
    bool good_measurement = false;

    int minInliersKalman = 5; //30;    // Kalman threshold updating


// Fill in the cloud data with camera centres
    pcl::PointCloud<pcl::PointXYZ>::Ptr cameracloud(new pcl::PointCloud<pcl::PointXYZ>);
    cameracloud->width = 1;
    cameracloud->height = 1;
    cameracloud->points.resize(1);
    cameracloud->points[0].x = 0;
    cameracloud->points[0].y = 0;
    cameracloud->points[0].z = 0;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h2(cameracloud, 255, 0, 255);

    pcl::PointCloud<pcl::PointXYZ>::Ptr kinectcloud(new pcl::PointCloud<pcl::PointXYZ>);
    kinectcloud->width = 1;
    kinectcloud->height = 1;
    kinectcloud->points.resize(1);
    kinectcloud->points[0].x = 0;
    kinectcloud->points[0].y = 0;
    kinectcloud->points[0].z = 0;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h3(kinectcloud, 0, 200, 0);
    kinect.viewer->addPointCloud(kinectcloud, cloud_in_color_h3, "cloud_kinect");
    kinect.viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20, "cloud_kinect");

    pcl::PointCloud<pcl::PointXYZ>::Ptr fixation3Dc(new pcl::PointCloud<pcl::PointXYZ>);
    fixation3Dc->width = 1;
    fixation3Dc->height = 1;
    fixation3Dc->points.resize(1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_fix3d_color_h(fixation3Dc, 255, 0, 0);


// Create mesh out of point cloud
    kinect.triangulate();
    if (show_mesh)
        kinect.visualize_triangulation();

    std::cout << "On \"Viewer\": \t Green is for Kinect camera \t Purple is for EyeTracker's camera \t Red is for 3D fixation" << std::endl;
    std::cout << "Click on \"EyeTracker\" window to activate 3D fixation localisation" << std::endl << std::endl;
//    sleep(2);
for(;;) {


    //ImageConverter ic;

//    ros::spinOnce();
//    ros::Duration(3).sleep();

    //cv::Mat kinect_RGBFrame = frame;
    //kinect_RGBFrame = ic.cv_color_ptr->image;

    kinect.k2g.get(kinect.RGBFrame, kinect.DepthFrame, kinect.cloud);
    camera.captureFrame();
    cv::cvtColor(kinect.RGBFrame, briskf.imgGray1, CV_BGR2GRAY);

    //cv::cvtColor(kinect_RGBFrame, briskf.imgGray1, CV_BGR2GRAY);

    cv::cvtColor(camera.RGBFrame, briskf.imgGray2, CV_BGR2GRAY);

    if (show_kinect) {
        kinect.viewer->spinOnce();
        cv::imshow("color", kinect.RGBFrame);
        cv::waitKey(1);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(kinect.cloud);
        kinect.viewer->updatePointCloud<pcl::PointXYZRGB>(kinect.cloud, rgb, "sample cloud");
    }

//    std::cout << "Kinect frame size: " << kinect.RGBFrame.rows << "," << kinect.RGBFrame.cols << std::endl;
//    std::cout << "Camera frame size: " << camera.RGBFrame.rows << "," << camera.RGBFrame.cols << std::endl;

    briskf.matchFrames();

    /// Drawing
    cv::Mat outimg;

    drawMatches(camera.RGBFrame, briskf.keypoints2, kinect.RGBFrame, briskf.keypoints1, briskf.matches, outimg, cv::Scalar(0, 255, 0),
                cv::Scalar(0, 0, 255), std::vector<std::vector<char> >(),
                cv::DrawMatchesFlags::DEFAULT);
    cv::namedWindow("Matches");
    cv::imshow("Matches", outimg);
//    cv::namedWindow("Depth");
//    cv::imshow("Depth", kinect.DepthFrame);
    char key = cvWaitKey(1);
    if (key == 27) // ESC
        break;

    cv::namedWindow("EyeTracker");
    cv::setMouseCallback("EyeTracker", PointsSelectionCallBackFunc, NULL);
    cv::imshow("EyeTracker", camera.RGBFrame);


    //

/////Pose estimation

    std::vector<cv::Point3f> list_points3d_rgbd; // container for the model 3D coordinates found in the scene
    std::vector<cv::Point2f> list_points2d_rgb; // container for the model 2D coordinates found in the scene

    // extract 2D matches
    for( size_t i = 0; i < briskf.matches.size(); i++ )
    {
        for( size_t j = 0; j < briskf.matches[i].size(); j++ )
        {
            int i1 = briskf.matches[i][j].queryIdx;
            int i2 = briskf.matches[i][j].trainIdx;
            cv::Point2f point2d_rgb = briskf.keypoints2[i1].pt;
            cv::Point2f point2d_rgbd = briskf.keypoints1[i2].pt;
            cv::Point3f point3d_rgbd;
            // @todo: Fix depth by color acquisition
            if (kinect.getDepthByColor(point2d_rgbd, point3d_rgbd))
            {
                list_points2d_rgb.push_back(point2d_rgb);
                list_points3d_rgbd.push_back(point3d_rgbd);
//                printf("[X,Y] %d,%d --> [X,Y,Z] %f,%f,%f\n", int(point2d_rgbd.x), int(point2d_rgbd.y), point3d_rgbd.x, point3d_rgbd.y, point3d_rgbd.z);
            }
        }
    }

    // RANSAC parameters
    int iterationsCount = 500;      // number of Ransac iterations.
    float reprojectionError = 2.0;  // maximum allowed distance to consider it an inlier.
    double confidence = 0.95;        // ransac successful confidence.
    int pnpMethod = cv::ITERATIVE;
    cv::Mat inliers_idx;
    std::vector<cv::Point2f> list_points2d_inliers;

    if(list_points3d_rgbd.size() > 0) // None matches, then RANSAC crashes
    {
        // Estimate the pose using RANSAC approach
        et3d.estimatePoseRANSAC(list_points3d_rgbd, list_points2d_rgb,
                                pnpMethod, inliers_idx, iterationsCount, reprojectionError, confidence);


        // Inliers keypoints to draw
        for (int inliers_index = 0; inliers_index < inliers_idx.rows; ++inliers_index) {
            int n = inliers_idx.at<int>(inliers_index);         // i-inlier
            cv::Point2f point2d = list_points2d_rgb[n]; // i-inlier point 2D
            list_points2d_inliers.push_back(point2d);           // add i-inlier to list
        }
//        std::cout << "#inliers: " << inliers_idx.rows << std::endl;
//        std::cout << "Pose: " << et3d.pose << std::endl;

        // Kalman Filter

        good_measurement = false;

        // GOOD MEASUREMENT
        if( inliers_idx.rows >= minInliersKalman )
        {

            // Get the measured translation
            cv::Mat translation_measured(3, 1, CV_32F);
            translation_measured = et3d.get_t_matrix();

            // Get the measured rotation
            cv::Mat rotation_measured(3, 3, CV_32F);
            rotation_measured = et3d.get_R_matrix();

            // fill the measurements vector
            et3d.fillMeasurements(measurements, translation_measured, rotation_measured);

            good_measurement = true;

        }

        // Instantiate estimated translation and rotation
        cv::Mat translation_estimated(3, 1, CV_32F);
        cv::Mat rotation_estimated(3, 3, CV_32F);

        // update the Kalman filter with good measurements
        et3d.updateKalmanFilter( KF, measurements,
                            translation_estimated, rotation_estimated);

        // Set estimated projection matrix
//        et3d.set_P_matrix(rotation_estimated, translation_estimated);
//        std::cout << "PoseKalman: " << et3d.pose << std::endl << std::endl;

        // Estimate and visualise cameras' centre

        cv::Point3f camera_centre = cv::Point3f(0,0,0);
        cv::Mat camera_centre_vec; camera_centre_vec = cv::Mat::zeros(3, 1, CV_32F);
        cv::Mat tmp; tmp = cv::Mat::zeros(3, 3, CV_32F);
        tmp = et3d._R_matrix.inv();
        tmp = tmp*(-1);
        camera_centre_vec = tmp*et3d._t_matrix;
        camera_centre = cv::Point3f(camera_centre_vec.at<float>(0), camera_centre_vec.at<float>(1), camera_centre_vec.at<float>(2));
        //std::cout << "camera centre: " << camera_centre << std::endl;
        //cameracloud->points.resize(pose + 1);
        cameracloud->points.resize(1);
        cameracloud->points[0].x = camera_centre.x;
        cameracloud->points[0].y = camera_centre.y;
        cameracloud->points[0].z = camera_centre.z;

        if (cameracloud) {
            if (!kinect.viewer->updatePointCloud(cameracloud, cloud_in_color_h2, "cloud_camera")){
//                kinect.viewer->addPointCloud(cameracloud, "cloud_camera");
//                viewer->addCube(eigtvec, eigrvec, 1.280, 0.960, 0.1, "cube");
                kinect.viewer->addPointCloud(cameracloud, cloud_in_color_h2, "cloud_camera");
                kinect.viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20, "cloud_camera");
                //viewer->addLine<pcl::PointXYZ>(cameracloud->points[0], cameracloud->points[1], 255, 0, 0, "linex");
                //viewer->addLine<pcl::PointXYZ>(cameracloud->points[0], cameracloud->points[2], 0, 255, 0, "liney");
                //viewer->addLine<pcl::PointXYZ>(cameracloud->points[0], cameracloud->points[3], 0, 0, 255, "linez");
            }
        }

    }

    // Estimate 3D fixation
    EyeTracker_PoR currentPOR;
//    gLastFrameNumber = gCurrentFrameNumber;
//    currentPOR.frame = gCurrentFrameNumber;
    currentPOR.PoR = cv::Point2f(0, 0) * 1;
    currentPOR.speed = 0;
    currentPOR.is_saccade = true;
    currentPOR.is_fixation = false;


    if (new_click)
        //if (bTrackingValid && try_fixation)
    {
        new_click = false;
        currentPOR.is_fixation = true;

//        std::cout << std::endl << "Fixation more than x seconds" << std::endl;
//        std::cout << std::endl << "Current pose: " << et3d.pose << std::endl;


        std::cout << "Calculating 3D fixation ..." << std::endl;
//        high_resolution_clock::time_point t1 = high_resolution_clock::now();
        currentPOR.PoR = testpoint;
//        bool intersection_found = et3d.backproject2DPoint(kinect.trianglesVertices, currentPOR.PoR, et3d.current_3Dfixation);

//        high_resolution_clock::time_point t2 = high_resolution_clock::now();
//        auto duration = (duration_cast<microseconds>(t2 - t1).count()) / 1000000;
        std::cout << "... calculated 3D fixation: " << et3d.current_3Dfixation << std::endl;


        //cv::Point2f p1 = SLAM.mpEyeT3D->backproject3DPoint(SLAM.mpEyeT3D->current_3Dfixation);

        // Move robot to highlight fixations
        //robot_highlight_3Dfixation(Point_to_Mat(SLAM.mpEyeT3D->current_3Dfixation), 0.4, SLAM.mpRegSLAMToKinect->transformation_matrix_KinectToRobot);
        //robot_highlight_3Dfixation(Point_to_Mat(fixations[fixationindex % 4]), 0.4, SLAM.mpRegSLAMToKinect->transformation_matrix_KinectToRobot);

        fixation3Dc->points[0].x = et3d.current_3Dfixation.x;
        fixation3Dc->points[0].y = et3d.current_3Dfixation.y;
        fixation3Dc->points[0].z = et3d.current_3Dfixation.z;

//        std::cout << "camera centre: " << camera_centre << std::endl;

        if (!kinect.viewer->updatePointCloud(fixation3Dc, cloud_fix3d_color_h, "cloud_fix3d")){
            kinect.viewer->addPointCloud(fixation3Dc, cloud_fix3d_color_h, "cloud_fix3d");
            kinect.viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "cloud_fix3d");
        }

        //ros::Subscriber clicked_point_subscriber = n.subscribe("/clicked_point",1,&click_callback);
        geometry_msgs::Pose fixation;
        fixation.position.x = et3d.current_3Dfixation.x;
        fixation.position.y = et3d.current_3Dfixation.y;
        fixation.position.z = et3d.current_3Dfixation.z;


        ROS_INFO_STREAM("3D fixation: " << fixation);

        fixation_pub.publish(fixation);
    }




    // @todo: Mike, here you should publish to ROS the following:
    // "currentPOR.is_fixation" : this is going to trigger ORK (fixation classified according to duel time)
    // "et3d.current_3Dfixation" : is the current 3D fixation you need, but you'll use it if currentPOR.is_fixation == true

} //end of loop

  return 0;
}

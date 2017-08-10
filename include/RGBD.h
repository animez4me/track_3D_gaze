//
// Created by alexandros on 21/07/17.
//

#ifndef BRISK_RGBD_H
#define BRISK_RGBD_H

#pragma GCC diagnostic ignored "-Wpedantic"

#include "libfreenect2/k2g.h"
#include <pcl/visualization/cloud_viewer.h>
#include <thread>
#include <iostream>  //NOLINT

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/spring.h>
#include <pcl/common/time.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/conversions.h>



class RGBDcamera
{
public:
    RGBDcamera(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr);
    virtual ~RGBDcamera();

    bool getDepthByColor(const cv::Point2f &point2d, cv::Point3f &point3d);

    void triangulate();
    void visualize_triangulation();

    //void captureRGBDdata(const sensor_msgs::ImageConstPtr& msg);

private:
    void captureRGBDdata();
    void readCameraParameters(std::string filename);

public:
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    cv::Mat RGBFrame, DepthFrame;
    cv::Mat camMatrix;
    cv::Mat distCoeffs;

    bool triangulated;
    pcl::PolygonMesh triangles;
    std::vector<std::vector<cv::Point3f> > trianglesVertices;

public: //make private within thread
    //K2G k2g;
//    std::thread RGBDcameraThread_;
//    std::mutex RGBDcameraData_mutex_;
};


#endif //BRISK_RGBD_H

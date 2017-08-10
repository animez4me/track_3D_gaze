//
// Created by alexandros on 21/07/17.
//

#include "RGBD.h"

void KeyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void * data);
void point_picking_callback(const pcl::visualization::PointPickingEvent& event, void*);

RGBDcamera::RGBDcamera(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud):
        viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"))
{
//  RGBDcamera::RGBDcamera(std::string filename,):
//    k2g(OPENGL, true, true),
//    viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"))
//  {


//    cloud = k2g.getCloud();

//    k2g.printParameters();


    cloud->sensor_orientation_.w() = 0.0;
    cloud->sensor_orientation_.x() = 1.0;
    cloud->sensor_orientation_.y() = 0.0;
    cloud->sensor_orientation_.z() = 0.0;

    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");


    viewer->registerKeyboardCallback(KeyboardEventOccurred);
    viewer->registerPointPickingCallback(&point_picking_callback);

    readCameraParameters(filename);
//    k2g.disableLog();

////    RGBDcameraThread_ = std::thread(&RGBDcamera::captureRGBDdata, this);

}

RGBDcamera::~RGBDcamera()
{
//    RGBDcameraThread_.join();
}

//void RGBDcamera::captureRGBDdata() {
//    for (;;)
//        k2g.get(RGBFrame, DepthFrame, cloud);

//}

void RGBDcamera::readCameraParameters(std::string filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        std::cout << "RGBD calibration file cannot be opened" << std::endl;
    fs["cameraMatrix"] >> camMatrix;
    fs["distortionCoefficients"] >> distCoeffs;
    std::cout << "RGBD cameraMatrix: " << camMatrix << std::endl;
    std::cout << "RGBD distortion: " << distCoeffs << std::endl;
}

bool RGBDcamera::getDepthByColor(const cv::Point2f &point2d, cv::Point3f &point3d)
{
    //return k2g.getDepthByColor_(point2d, point3d);
  return true;
}

void RGBDcamera::triangulate() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_xyz->points.resize(cloud->points.size());

    for (size_t i = 0; i < cloud->size(); i++) {
        cloud_xyz->points[i].x = cloud->points[i].x;
        cloud_xyz->points[i].y = cloud->points[i].y;
        cloud_xyz->points[i].z = cloud->points[i].z;
    }

    std::cout << "Kinect model processing... " << std::endl;

    // Remove outliers from Point cloud (statistical)
    std::cout << "Removing outliers... ";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZ>);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_outliers;
    sor_outliers.setInputCloud(cloud_xyz);
    sor_outliers.setMeanK(30);
    sor_outliers.setStddevMulThresh(1.0);
    sor_outliers.filter(*cloud_inliers);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_inliers << std::endl;

    std::cout << "...removed outliers." << std::endl;

    // Smoothing surface (LSM)
    std::cout << "Smoothing surface... ";
    // @Header

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_smooth(new pcl::search::KdTree<pcl::PointXYZ>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals(true);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers_(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_inliers_->height=1;
    for (int i=0, j=0; i<cloud_inliers->size(); i++)
    {
        if(pcl_isfinite (cloud_inliers->points[i].x)) {
            cloud_inliers_->width=j+1;
            cloud_inliers_->points.resize(j+1);
            cloud_inliers_->points[j].x = cloud_inliers->points[i].x;
            cloud_inliers_->points[j].y = cloud_inliers->points[i].y;
            cloud_inliers_->points[j].z = cloud_inliers->points[i].z;
            j++;
        }
    }
//    pcl::io::savePCDFileASCII("test_pcd.pcd", *cloud_inliers_);
    mls.setInputCloud(cloud_inliers_);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree_smooth);
    mls.setSearchRadius(0.07);

    // Reconstruct
    mls.process(mls_points);

    cloud_filtered->resize(mls_points.size());
    for (size_t i = 0; i < mls_points.points.size(); ++i)
    {
        cloud_filtered->points[i].x = mls_points.points[i].x;
        cloud_filtered->points[i].y = mls_points.points[i].y;
        cloud_filtered->points[i].z = mls_points.points[i].z;
    };

    std::cout << "...smoothed surface." << std::endl;

    std::cout << "Kinect point cloud triangulation... " << std::endl;
    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);


    tree->setInputCloud(cloud_filtered);
    n.setInputCloud(cloud_filtered);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);
    // normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud_filtered, *normals, *cloud_with_normals);
    // cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    //pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(0.5);
    //gp3.setSearchRadius(2);
    // Set typical values for the parameters
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18); // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    int countermissing = 0;
    //std::vector<std::vector<cv::Point3f> > trianglesVertices;
    for (size_t i = 0; i < triangles.polygons.size(); ++i)
    {
        std::vector<cv::Point3f> vertices(3);
        for (int triangle_vertex_idx = 0; triangle_vertex_idx < 3; triangle_vertex_idx++)
        {
            //cout << (float)(mpMap->vpPoints[mpMap->triangles.polygons[i].vertices[triangle_vertex_idx]]->v3WorldPos[0]) << endl;
            //cout << (float)(mpMap->vpPoints[mpMap->triangles.polygons[i].vertices[triangle_vertex_idx]]->v3WorldPos[1]) << endl;
            //cout << (float)(mpMap->cloud_filtered[mpMap->triangles.polygons[i].vertices[triangle_vertex_idx]]->v3WorldPos[2]) << endl;
            float x = (float)(cloud_filtered->points[triangles.polygons[i].vertices[triangle_vertex_idx]].x);
            float y = (float)(cloud_filtered->points[triangles.polygons[i].vertices[triangle_vertex_idx]].y);
            float z = (float)(cloud_filtered->points[triangles.polygons[i].vertices[triangle_vertex_idx]].z);
            cv::Point3f vertex = cv::Point3f(x, y, z);
            vertices[triangle_vertex_idx] = vertex;
        }
        trianglesVertices.push_back(vertices);
        //if (parts[i] == -1)
        // countermissing++;
    }
    //cout << countermissing << " unconnected points" << endl;

    //pcl::io::saveVTKFile("Kinect_triangulated.vtk", triangles);

    triangulated = true;
    std::cout << "...finished Kinect point cloud triangulation." << std::endl << std::endl;
    /*
*/
}

void RGBDcamera::visualize_triangulation()
{

    // Visualization
    pcl::visualization::PCLVisualizer viewer("TRIANGULATION");
    // Create two verticaly separated viewports
    int v1(0);
    viewer.createViewPort(0.0, 0.0, 1.0, 1.0, v1);

    // The color we will be using
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    viewer.setBackgroundColor(0.2, 0.2, 0.2);
    viewer.addPolygonMesh(triangles);

    viewer.setSize(1280, 960);  // Visualiser window size

    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        //std::cout << "getViewerPose: " << std::endl;
        //viewer_pose = viewer.getViewerPose().matrix();
        //print4x4Matrix(viewer_pose.cast<double>());
        //std::cout << std::endl;
    }
    viewer.close();
}

void KeyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void * data)
{
    std::string pressed = event.getKeySym();
    if(event.keyDown ())
    {
        if(pressed == "s")
        {
            std::cout << "pressed s" << std::endl;
        }
    }
}

void point_picking_callback(const pcl::visualization::PointPickingEvent& event, void*)
{
    if (event.getPointIndex() == -1)
        return;
    cv::Point3f current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);

    std::cout << "3D point: " << current_point.x << ", " << current_point.y << ", " << current_point.z << "" << std::endl;

}

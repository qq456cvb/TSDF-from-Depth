//
//  main.cpp
//  Scanner
//
//  Created by Neil on 10/02/2017.
//  Copyright © 2017 Neil. All rights reserved.
//
#include <iostream>
#include <vector>
#include "ColorShader.h"
#include "FileReader.hpp"
#include <iostream>
#include <fstream>
#include <assert.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

extern int main_gpu(int argc, char *argv[]);

using namespace std;

double x_min = -0.7, x_max = 0.8, y_min = -0.4, y_max = 1.1, z_min = -1.0, z_max = 0.5;
cv::Vec3d vol_start(x_min, y_min, z_min);
cv::Vec3d vol_end(x_max, y_max, z_max);
int dim = 64;
double voxel = (x_max - x_min) / (dim-1);
int tex_dim = sqrt(dim*dim*dim);

void drawScene() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    std::vector<cv::Vec4f> cube = { cv::Vec4f(-1, -1, 1.0, 1.0),
        cv::Vec4f(1.0, -1.0, 1.0, 1.0),
        cv::Vec4f(0, 1.0, 1.0, 1.0)};
    ColorShader color_shader;
    color_shader.init();
    
    color_shader.setMVPMatrix(cv::Matx44f::eye());
    
    
    color_shader.drawVertices(GL_TRIANGLES, cube.data(), 3, cv::Vec4f(1.0, 0, 0, 1.0));
    
    glutSwapBuffers();
}

std::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}

cv::Vec3b interpColor(const cv::Mat& color, const cv::Vec3d& pos) {
    cv::Vec3d ind = (pos - cv::Vec3d(x_min, y_min, z_min)) / voxel;
    int tex_ind = int(ind[0]) * dim * dim + int(ind[1]) * dim + int(ind[2]);
    
    return color.at<cv::Vec3b>(tex_ind / tex_dim, tex_ind % tex_dim);
}

double interpTSDF(const cv::Mat& tsdf, const cv::Vec3d& pos)
{
    assert(pos[0] > x_min && pos[1] > y_min && pos[2] > z_min);
    assert(pos[0] < x_max && pos[1] < y_max && pos[2] < z_max);
    
    cv::Vec3d ind = (pos - cv::Vec3d(x_min, y_min, z_min)) / voxel;
    
    int tex_ind = int(ind[0]) * dim * dim + int(ind[1]) * dim + int(ind[2]);
    double lll = tsdf.at<double_t>(tex_ind / tex_dim, tex_ind % tex_dim);
    
    tex_ind = int(ind[0]) * dim * dim + int(ind[1]) * dim + int(ind[2]+1);
    double llh = tsdf.at<double_t>(tex_ind / tex_dim, tex_ind % tex_dim);
    
    tex_ind = int(ind[0]) * dim * dim + int(ind[1]+1) * dim + int(ind[2]);
    double lhl = tsdf.at<double_t>(tex_ind / tex_dim, tex_ind % tex_dim);
    
    tex_ind = int(ind[0]) * dim * dim + int(ind[1]+1) * dim + int(ind[2]+1);
    double lhh = tsdf.at<double_t>(tex_ind / tex_dim, tex_ind % tex_dim);
    
    tex_ind = int(ind[0]+1) * dim * dim + int(ind[1]) * dim + int(ind[2]);
    double hll = tsdf.at<double_t>(tex_ind / tex_dim, tex_ind % tex_dim);
    
    tex_ind = int(ind[0]+1) * dim * dim + int(ind[1]) * dim + int(ind[2]+1);
    double hlh = tsdf.at<double_t>(tex_ind / tex_dim, tex_ind % tex_dim);
    
    tex_ind = int(ind[0]+1) * dim * dim + int(ind[1]+1) * dim + int(ind[2]);
    double hhl = tsdf.at<double_t>(tex_ind / tex_dim, tex_ind % tex_dim);
    
    tex_ind = int(ind[0]+1) * dim * dim + int(ind[1]+1) * dim + int(ind[2]+1);
    double hhh = tsdf.at<double_t>(tex_ind / tex_dim, tex_ind % tex_dim);
    
    double frac_x = ind[0] - int(ind[0]);
    double frac_y = ind[1] - int(ind[1]);
    double frac_z = ind[2] - int(ind[2]);
    
    double x1 = (1-frac_x) * lll + frac_x * hll;
    double x2 = (1-frac_x) * lhl + frac_x * hhl;
    double x3 = (1-frac_x) * llh + frac_x * hlh;
    double x4 = (1-frac_x) * lhh + frac_x * hhh;
    
    double plane1 = (1-frac_y) * x1 + frac_y * x2;
    double plane2 = (1-frac_y) * x3 + frac_y * x4;
    
    return (1-frac_z) * plane1 + frac_z * plane2;
}


int main_cpu(int argc, char * argv[]) {
//    glutInit(&argc, argv);
//    
//    // We're going to animate it, so double buffer
//    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH );
//    
//    // Initial parameters for window position and size
//    glutInitWindowPosition( 60, 60 );
//    glutInitWindowSize( 800, 600 );
//    glutCreateWindow("Test");
//
//    // Call this whenever window needs redrawing
//    glutDisplayFunc( drawScene );
//    
//    // Start the main loop.  glutMainLoop never returns.
//    glutMainLoop( );
    
    // 0.8965 1.3578 1.2498 0.9449 0.0716 -0.1917 0.2554
    FileReader reader;
//    0.8802 1.3900 1.2212 0.9443 0.0924 -0.2025 0.2422
//    auto depth = reader.readDepth("./data/E2_pre_registereddata/depth/1433088554.25673.png");
//    cv::Vec3f camera_center(1.6522, 1.4959, 0.1105);
//    auto theta = acos(0.1826);
//    auto axis = cv::Vec3d(0.7774, 0.2000, -0.5678);
//    auto depth = reader.readDepth("./data/E2_pre_registereddata/depth/1433088549.58964.png");
//    cv::Vec3f camera_center(0.8802, 1.3900, 1.2212);
//    auto theta = acos(0.2422);
//    auto axis = cv::Vec3d(0.9443, 0.0924, -0.2025);
    
    auto depth = reader.readDepth("./data/E2_pre_registereddata/depth/1433088554.989649.png");
    auto color = cv::imread("./data/E2_pre_registereddata/rgb/1433088554.995899.png");
    cv::Vec3f camera_center(1.7318, 1.4286, -0.1555);
    auto theta = acos(0.1938);
    auto axis = cv::Vec3d(0.7529, 0.1969, -0.5974);
    axis /= sin(theta);
    
    auto rod = 2 * theta * axis;
    cv::Mat rotation;
    cv::Rodrigues(rod, rotation);
    
    double fx = 468.60, fy = 468.60, cx = 318.27, cy = 243.99;
    cv::Mat intrinsic = cv::Mat::eye(3, 3, CV_64F);
    intrinsic.at<double_t>(0, 0) = fx;
    intrinsic.at<double_t>(0, 2) = cx;
    intrinsic.at<double_t>(1, 1) = fy;
    intrinsic.at<double_t>(1, 2) = cy;
    intrinsic.at<double_t>(2, 2) = 1;
    
//    std::cout << intrisic << std::endl;
    
    cv::Mat extrinsic = cv::Mat::eye(4, 4, CV_64F);
    rotation.copyTo(extrinsic(cv::Rect(0, 0, 3, 3)));
    
//    cv::Mat C(3, 1, CV_64F);
//    C.at<double_t>(0, 0) = 0.8965;
//    C.at<double_t>(1, 0) = 1.3578;
//    C.at<double_t>(2, 0) = 1.2498;
//    cv::Mat negRc = - rotation * C;
    extrinsic.at<double_t>(0, 3) = camera_center[0];
    extrinsic.at<double_t>(1, 3) = camera_center[1];
    extrinsic.at<double_t>(2, 3) = camera_center[2];
//    negRc.copyTo(extrinsic(cv::Rect(3, 0, 1, 3)));
    
    extrinsic = extrinsic.inv();

    rotation = rotation.inv();
    std::cout << extrinsic << std::endl;
    std::cout << rotation << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
//    
    cv::Mat intrinsic_inv = intrinsic.inv();
    cv::Mat extrinsic_inv = extrinsic.inv();
    cv::Mat rotation_inv = rotation.inv();
    
    cv::Mat w2s = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat intrinsic44 = cv::Mat::eye(4, 4, CV_64F);
    intrinsic.copyTo(intrinsic44(cv::Rect(0, 0, 3, 3)));
    w2s = intrinsic44 * extrinsic;
    auto w2s_inv = w2s.inv();
//    for (int i = 0; i < depth.rows; i++) {
//        for (int j = 0; j < depth.cols; j++) {
//            pcl::PointXYZRGB point;
////            point.x = basic_point.x;
////            point.y = basic_point.y;
////            point.z = basic_point.z;
//            cv::Mat scr = cv::Mat(3, 1, CV_64F);
//            double z = ((double)depth.at<uint16_t>(i, j)) / 5000;
//            scr.at<double_t>(0, 0) = j * z;
//            scr.at<double_t>(1, 0) = i * z;
//            scr.at<double_t>(2, 0) = z;
//
//            cv::Mat cam = intrinsic_inv * scr;
//            cv::Mat world(4, 1, CV_64F);
//            world.at<double_t>(0, 0) = cam.at<double_t>(0, 0);
//            world.at<double_t>(1, 0) = cam.at<double_t>(1, 0);
//            world.at<double_t>(2, 0) = cam.at<double_t>(2, 0);
//            world.at<double_t>(3, 0) = 1;
//
//            world = extrinsic_inv * world;
//
//            point.x = world.at<double_t>(0, 0);
//            point.y = world.at<double_t>(1, 0);
//            point.z = world.at<double_t>(2, 0);
//
//            point.r = 255;
//            point.g = 128;
//            point.b = 128;
//            point_cloud_ptr->points.push_back (point);
//        }
//    }
//
//    point_cloud_ptr->width = (int) depth.rows * depth.cols;
//    point_cloud_ptr->height = 1;
//
    
//


    double mu = (x_max - x_min) / dim * 2;
    cv::Mat tsdf(tex_dim, tex_dim, CV_64F, cv::Scalar(mu));
    cv::Mat tsdf_color(tex_dim, tex_dim, CV_8UC3, cv::Scalar(0, 0, 0));
    
    double eps = 1e-5;
    for (int i = 0; i < tex_dim; i++) {
        for (int j = 0; j< tex_dim; j++) {
            auto flatten = i*tex_dim + j;
            auto x_ind = flatten / (dim*dim);
            auto y_ind = flatten / dim - x_ind * dim;
            auto z_ind = flatten % dim;
            
            cv::Mat pos(4, 1, CV_64F);
            pos.at<double_t>(0, 0) = x_min + x_ind * (x_max-x_min) / dim;
            pos.at<double_t>(1, 0) = y_min + y_ind * (y_max-y_min) / dim;
            pos.at<double_t>(2, 0) = z_min + z_ind * (z_max-z_min) / dim;
            pos.at<double_t>(3, 0) = 1.0;
            
            
            cv::Mat proj = extrinsic * pos;
            cv::Mat cam(3, 1, CV_64F);
            cam.at<double_t>(0, 0) = proj.at<double_t>(0, 0);
            cam.at<double_t>(1, 0) = proj.at<double_t>(1, 0);
            cam.at<double_t>(2, 0) = proj.at<double_t>(2, 0);
            
            
            cv::Mat pixel = intrinsic * cam;
            
            pixel /= pixel.at<double_t>(2, 0);
            
            double x = pixel.at<double_t>(0, 0);
            double y = pixel.at<double_t>(1, 0);
            if (x < 0 || x > 639 || y < 0 || y > 479) {
                continue;
            }
            // TODO: add interpolation
            double diff =  ((double)depth.at<uint16_t>(480-y, x)) / 5000 - cam.at<double_t>(2, 0);
            if (depth.at<uint16_t>(480-y, x) == 0) diff = mu;
            diff = std::max(std::min(diff, mu), -mu);
            tsdf.at<double_t>(i, j) = diff;
            tsdf_color.at<cv::Vec3b>(i, j) = color.at<cv::Vec3b>(480-y, x);
        }
    }
    
    
    
    for (int i = 0; i < tex_dim; i++) {
        for (int j = 0; j< tex_dim; j++) {
            auto flatten = i*tex_dim + j;
            auto x_ind = flatten / (dim*dim);
            auto y_ind = flatten / dim - x_ind * dim;
            auto z_ind = flatten % dim;
            
            cv::Mat pos(4, 1, CV_64F);
            pos.at<double_t>(0, 0) = x_min + x_ind * (x_max-x_min) / dim;
            pos.at<double_t>(1, 0) = y_min + y_ind * (y_max-y_min) / dim;
            pos.at<double_t>(2, 0) = z_min + z_ind * (z_max-z_min) / dim;
            
            pcl::PointXYZRGB point;
            
            point.x = pos.at<double_t>(0, 0);
            point.y = pos.at<double_t>(1, 0);
            point.z = pos.at<double_t>(2, 0);
            
            auto ratio = (tsdf.at<double_t>(i, j) + mu) / mu / 2;
            if (ratio > 0.5) {
                continue;
            }
            point.r = ratio * 255;
            point.g = 0;
            point.b = (1-ratio) * 255;
//            point_cloud_ptr->points.push_back (point);

        }
    }
    
    
    pcl::PointXYZ p2;
    cv::Mat img(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    // ray tracing
    for (int i = 0; i < 640; i++) {
        for (int j = 0; j < 480; j++) {
            cv::Vec3d origin = camera_center;
            
            cv::Mat scr = cv::Mat(4, 1, CV_64F);
            double z = ((double)depth.at<uint16_t>(j, i)) / 5000;
            scr.at<double_t>(0, 0) = i;
            scr.at<double_t>(1, 0) = j;
            scr.at<double_t>(2, 0) = 1;
            scr.at<double_t>(3, 0) = 1;

            cv::Mat world = w2s_inv * scr;
            cv::Vec3d target(world.at<double_t>(0, 0),
                             world.at<double_t>(1, 0),
                             world.at<double_t>(2, 0));
//            cv::Mat cam = intrinsic_inv * scr;
//            cam = rotation_inv * cam;
//
//
//            cam.at<double_t>(0, 0) += origin[0];
//            cam.at<double_t>(1, 0) += origin[1];
//            cam.at<double_t>(2, 0) += origin[2];
//
////            cv::Mat world(4, 1, CV_64F);
////            world.at<double_t>(0, 0) = cam.at<double_t>(0, 0);
////            world.at<double_t>(1, 0) = cam.at<double_t>(1, 0);
////            world.at<double_t>(2, 0) = cam.at<double_t>(2, 0);
////            world.at<double_t>(3, 0) = 1;
//
////            world = extrinsic_inv * world;
//
//            cv::Vec3d target(cam.at<double_t>(0, 0),
//                          cam.at<double_t>(1, 0),
//                          cam.at<double_t>(2, 0));
            
            if (i == 320 && j == 240) {
                p2.x = target[0];
                p2.y = target[1];
                p2.z = target[2];
//                cout << cam << endl;
            }
            pcl::PointXYZRGB point;
            
            point.x = target[0];
            point.y = target[1];
            point.z = target[2];
            
            point.r = 0;
            point.g = 128;
            point.b = 255;
//            point_cloud_ptr->points.push_back (point);
            
            
            cv::Vec3d dist = target - origin;
            cv::Vec3d dir;
            cv::normalize(dist, dir);
            
            cv::Vec3d inv_dir = cv::Vec3d(1.0/dir[0], 1.0/dir[1], 1.0/dir[2]);
            cv::Vec3d tbot, ttop;
            cv::multiply(inv_dir, vol_start - origin, tbot);
            cv::multiply(inv_dir, vol_end - origin, ttop);
            
            cv::Vec3d tmin(std::min(ttop[0], tbot[0]), std::min(ttop[1], tbot[1]), std::min(ttop[2], tbot[2]));
            cv::Vec3d tmax(std::max(ttop[0], tbot[0]), std::max(ttop[1], tbot[1]), std::max(ttop[2], tbot[2]));
            double largest_tmin = std::max(std::max(tmin[0], tmin[1]), tmin[2]);
            double smallest_tmax = std::min(std::min(tmax[0], tmax[1]), tmax[2]);
            
            double tnear = std::max(largest_tmin, 0.1);
            double tfar = std::min(smallest_tmax, 100.0);
            
            double eps = 0.001;
            if (tnear < tfar - eps) {
                
                double t = tnear + eps;
                double stepsize = voxel;
                double f_t = interpTSDF(tsdf, origin + dir * t);
                
                img.at<cv::Vec3b>(j, i) = interpColor(tsdf_color, origin + dir * t);
//                auto pt = origin + dir * t;
                
//                pcl::PointXYZRGB point;
//                
//                point.x = pt[0];
//                point.y = pt[1];
//                point.z = pt[2];
//                
//                point.r = 0;
//                point.g = 255;
//                point.b = 0;
//                point_cloud_ptr->points.push_back (point);
                
                double f_tt = 0.0;
                if( f_t > 0.0){     // ups, if we were already in it, then don't render anything here
                    for(; t < tfar; t += stepsize){
                        f_tt = interpTSDF(tsdf, origin + dir * t);
                        if(f_tt < 0.0)                               // got it, jump out of inner loop
                            break;
                        if(f_tt < voxel / 2)                            // coming closer, reduce stepsize
                            stepsize = voxel / 4;
                        f_t = f_tt;
                    }
                    if(f_tt < 0.0){                               // got it, calculate accurate intersection
                        t = t + stepsize * f_tt / (f_t - f_tt);
                        cv::Vec3d pt = origin + dir * t;
                        pcl::PointXYZRGB point;
                        
                        point.x = pt[0];
                        point.y = pt[1];
                        point.z = pt[2];
                        
                        point.r = 0;
                        point.g = 255;
                        point.b = 0;
                        point_cloud_ptr->points.push_back (point);
                    }
                }
            }
        }
    }
    cv::imshow("img", img);
    cv::waitKey(30);
    auto viewer = rgbVis(point_cloud_ptr);
    
    pcl::PointXYZ p1;
    //0.8965, 1.3578, 1.2498
    p1.x = camera_center[0];
    p1.y =camera_center[1];
    p1.z =camera_center[2];
    viewer->addLine(p1, p2);
    
//    viewer->addCube(x_min, x_max, y_min, y_max, z_min, z_max);
//    std::fstream fs;
//    fs.open("");
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        // std::this_thread::sleep (std::posix_time::microseconds (100000));
    }
    return 0;
}



using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int, char **argv)
{
    print_error ("Syntax is: %s input.ply output.pcd\n", argv[0]);
}

bool
loadCloud (const std::string &filename, PCLPointCloud2 &cloud)
{
    TicToc tt;
    print_highlight ("Loading "); print_value ("%s ", filename.c_str ());
    
    pcl::PLYReader reader;
    tt.tic ();
    if (reader.read (filename, cloud) < 0)
        return (false);
    print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
    print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());
    
    return (true);
}

void
saveCloud (const std::string &filename, const pcl::PCLPointCloud2 &cloud, bool format)
{
    TicToc tt;
    tt.tic ();
    
    print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
    
    pcl::PCDWriter writer;
    writer.write (filename, cloud, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), format);
    
    print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
}

/* ---[ */
int
main (int argc, char** argv)
{
    print_info ("Convert a PLY file to PCD format. For more information, use: %s -h\n", argv[0]);

    if (argc < 3)
    {
        printHelp (argc, argv);
        return (-1);
    }

    // Parse the command line arguments for .pcd and .ply files
    std::vector<int> pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
    std::vector<int> ply_file_indices = parse_file_extension_argument (argc, argv, ".ply");
    if (pcd_file_indices.size () != 1 || ply_file_indices.size () != 1)
    {
        print_error ("Need one input PLY file and one output PCD file.\n");
        return (-1);
    }

    // Command line parsing
    bool format = 0;
    parse_argument (argc, argv, "-format", format);
    print_info ("PCD output format: "); print_value ("%s\n", (format ? "binary" : "asci"));

    // Load the first file
    pcl::PCLPointCloud2 cloud;
    if (!loadCloud (argv[ply_file_indices[0]], cloud))
        return (-1);

    // Convert to PLY and save
    saveCloud (argv[pcd_file_indices[0]], cloud, format);
}



//int main(int argc, char * argv[]) {
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::PCDReader reader;
//    if (reader.read ("out.pcd", *cloud) < 0)
//        return (false);
//
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//    viewer->setBackgroundColor (0, 0, 0);
//    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
//    viewer->addCoordinateSystem (1.0);
//    viewer->initCameraParameters ();
//    while (!viewer->wasStopped ())  {
//        viewer->spinOnce (100);
//    }
//    return 0;
//    return main_gpu(argc, argv);
//    return main_cpu(argc, argv);
//}





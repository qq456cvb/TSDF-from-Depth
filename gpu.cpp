//
//  gpu.cpp
//  Scanner
//
//  Created by Neil on 30/10/2017.
//  Copyright © 2017 Neil. All rights reserved.
//

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include "ColorShader.h"
#include "TSDFShader.hpp"
#include "ViewShader.hpp"
#include "TextureShader.hpp"
#include "FileReader.hpp"
#include <iostream>
#include <fstream>
#include <assert.h>


int main_gpu(int argc, char * argv[]) {
    if (!glfwInit()) {
        fprintf(stderr, "ERROR: could not start GLFW3\n");
        return 1;
    }
    
    FileReader reader;
//    auto depth = reader.readDepth("./data/E2_pre_registereddata/depth/1433088549.58964.png");
//    auto color = cv::imread("./data/E2_pre_registereddata/rgb/1433088549.562969.png");
//    cv::cvtColor(color, color, CV_BGR2RGB);
//    cv::Vec3f camera_center(0.8802, 1.3900, 1.2212);
//    auto theta = acos(0.2422);
//    auto axis = cv::Vec3d(0.9443, 0.0924, -0.2025);
    
    auto depth = reader.readDepth("./data/E2_pre_registereddata/depth/1433088554.989649.png");
    auto color = cv::imread("./data/E2_pre_registereddata/rgb/1433088554.995899.png");
    cv::cvtColor(color, color, CV_BGR2RGB);
    cv::Vec3f camera_center(1.7318, 1.4286, -0.1555);
    auto theta = acos(0.1938);
    auto axis = cv::Vec3d(0.7529, 0.1969, -0.5974);
    
    assert(depth.isContinuous());
    assert(color.isContinuous());
    
    std::cout << color.channels() << std::endl;
    
    
    
    axis /= sin(theta);
    
    auto rod = 2 * theta * axis;
    cv::Mat rotation;
    cv::Rodrigues(rod, rotation);
    
    double fx = 468.60, fy = 468.60, cx = 318.27, cy = 243.99;
    cv::Mat intrinsic = cv::Mat::eye(4, 4, CV_32F);
    intrinsic.at<float>(0, 0) = fx;
    intrinsic.at<float>(0, 2) = cx;
    intrinsic.at<float>(1, 1) = fy;
    intrinsic.at<float>(1, 2) = cy;
    intrinsic.at<float>(2, 2) = 1;
    
    //    std::cout << intrisic << std::endl;
    
    cv::Mat extrinsic = cv::Mat::eye(4, 4, CV_32F);
    rotation.copyTo(extrinsic(cv::Rect(0, 0, 3, 3)));
    
    extrinsic.at<float>(0, 3) = camera_center[0];
    extrinsic.at<float>(1, 3) = camera_center[1];
    extrinsic.at<float>(2, 3) = camera_center[2];
    
    extrinsic = extrinsic.inv();
    
    rotation = rotation.inv();
    //    std::cout << extrinsic << std::endl;
    
    //    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    //
    cv::Mat intrinsic_inv = intrinsic.inv();
    cv::Mat extrinsic_inv = extrinsic.inv();
    cv::Mat rotation_inv = rotation.inv();
    
    cv::Mat w2c = intrinsic * extrinsic;
    cv::Mat identity = cv::Mat::eye(4, 4, CV_32F);
    cv::Matx44f w2c_x((float *) w2c.ptr());
    
    cv::Mat w2s = cv::Mat::eye(4, 4, CV_32F);
    w2s = intrinsic * extrinsic;
    cv::Mat w2s_inv = w2s.inv();
    cv::Matx44f w2s_x((float *) w2s.ptr());
    cv::Matx44f w2s_inv_x((float *) w2s_inv.ptr());
    
    
    
    // uncomment these lines if on Apple OS X
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(640, 480, "Hello Triangle", NULL, NULL);
    if (!window) {
        fprintf(stderr, "ERROR: could not open window with GLFW3\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    
    int height, width;
    glfwGetFramebufferSize(window, &width, &height);
    
    // start GLEW extension handler
    glewExperimental = GL_TRUE;
    glewInit();
    
    glClampColor(GL_CLAMP_FRAGMENT_COLOR, GL_FALSE);
    glClampColor(GL_CLAMP_VERTEX_COLOR, GL_FALSE);
    glClampColor(GL_CLAMP_READ_COLOR, GL_FALSE);
    
    // upload depth texture
    GLuint tex_depth, tex_color;
    glGenTextures(1, &tex_depth);
    glBindTexture(GL_TEXTURE_2D, tex_depth);
    
    unsigned char pix[]={255,255,255, 255, 0,0,0, 255, 255,255,255, 255, 0,0,0, 255};
    
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    cv::Mat depth_float(depth.rows, depth.cols, CV_32FC1);
    depth.convertTo(depth_float, CV_32F, 1.0 / 65535.);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, depth.cols, depth.rows, 0, GL_RED, GL_UNSIGNED_SHORT, depth.ptr());
//    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, depth_float.cols, depth_float.rows, 0, GL_RED, GL_FLOAT, depth_float.ptr());
    glBindTexture(GL_TEXTURE_2D, 0);
    
    // upload color texture
    glGenTextures(1, &tex_color);
    glBindTexture(GL_TEXTURE_2D, tex_color);
    
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, color.cols, color.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, color.ptr());
    //    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, pix);
    glBindTexture(GL_TEXTURE_2D, 0);
    
//    glViewport(0, 0, 640, 480);
    float c[3] = {camera_center[0], camera_center[1], camera_center[2]};
    auto shader = std::make_shared<TSDFShader>();
    shader->setMVPMatrix(w2s_x);
    auto tex_shader = std::make_shared<TextureShader>();
    auto view_shader = std::make_shared<ViewShader>();
    view_shader->setS2WMatrix(w2s_inv_x);
    view_shader->setCameraCenter(c);

    
    
    while(!glfwWindowShouldClose(window)) {
        // wipe the drawing surface clear
        
//
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glActiveTexture(GL_TEXTURE0);
        
        glBindTexture(GL_TEXTURE_2D, tex_color);
        
        glActiveTexture(GL_TEXTURE1);
        
        glBindTexture(GL_TEXTURE_2D, tex_depth);
        shader->drawFrameBuffer();
//        glViewport(0, 0, width, height);
//        glBindFramebuffer(GL_FRAMEBUFFER, fboId);
//        glUseProgram(shader->mProgram.getId());
//        glBindVertexArray(shader->vao);
//        glDrawArrays(GL_TRIANGLES, 0, 3);
//        glBindVertexArray(0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glActiveTexture(GL_TEXTURE0);
        
        
        glViewport(0, 0, width, height);
        glUseProgram(view_shader->mProgram.getId());

        glBindVertexArray(view_shader->vao);
        
        glBindTexture(GL_TEXTURE_2D, shader->mTexTSDF);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glBindTexture(GL_TEXTURE_2D, 0);
//
//        glBindVertexArray(0);
////
//        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        //        glUnmapBuffer(GL_ARRAY_BUFFER);
        // draw points 0-3 from the currently bound VAO with current in-use shader
        //        glDrawArrays(GL_TRIANGLES, 0, 3);
        // update other events like input handling
        glfwPollEvents();
        // put the stuff we've been drawing onto the display
        glfwSwapBuffers(window);
    }
    // close GL context and any other GLFW resources
    glfwTerminate();
    
    //    shader->setMVPMatrix()
    return 0;
}

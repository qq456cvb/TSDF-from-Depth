//
//  ColorShader.hpp
//  Tracking3D
//
//  Created by Neil on 17/10/2017.
//  Copyright © 2017 Neil. All rights reserved.
//

#ifndef ColorShader_hpp
#define ColorShader_hpp

#include <opencv2/core.hpp>
#include "ShaderProgram.h"

#include <stdio.h>

class ColorShader
{
public:
    ColorShader() {
        init();
    }
    ~ColorShader() {
        free();
    }
    
    bool init();
    void free() {mProgram.free();}
    
    //mvp is in normal opencv row-major order
    void setMVPMatrix(const cv::Matx44f &mvp);
    
    void drawVertices(GLenum mode, const cv::Point2f *vertices, int count, const cv::Vec4f &color);
    void drawVertices(GLenum mode, const cv::Point2f *vertices, const cv::Vec4f *color, int count);
    void drawVertices(GLenum mode, const cv::Vec4f *vertices, int count, const cv::Vec4f &color);
    void drawVertices(GLenum mode, const cv::Vec4f *vertices, const cv::Vec4f *color, int count);
    void drawVertices(GLenum mode, const unsigned int *indices, unsigned int indexCount, const cv::Vec4f *vertices, const cv::Vec4f *color);
    void drawRect(const cv::Point2f center[], int count, const cv::Vec4f &color, float size, float aspect);
    void colorTest(float points[]);
    GLuint vbos[2];
    GLuint vao;
    ShaderProgram mProgram;
protected:
    
    int mUniformMVPMatrix;
    int mAttribPosCoord;
    int mAttribColor;
    
};
#endif /* ColorShader_hpp */


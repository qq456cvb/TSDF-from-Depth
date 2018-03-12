//
//  TSDFShader.hpp
//  Scanner
//
//  Created by Neil on 11/02/2017.
//  Copyright © 2017 Neil. All rights reserved.
//

#ifndef TSDFShader_hpp
#define TSDFShader_hpp

#include <stdio.h>
#include <cmath>

#include <opencv2/core/core.hpp>
#include "ShaderProgram.h"

class TSDFShader
{
public:
    TSDFShader() {
        init();
    }
    ~TSDFShader() {
        glDeleteTextures(1, &mTexTSDF);
        glDeleteFramebuffers(1, &mFrameBuffer);
    }
    
    bool init();
    void free() {
        mProgram.free();
    }
//    void drawCube();
    void update(const cv::Point2f center[], int count, const cv::Vec4f &color, float size, float aspect);
    void drawFrameBuffer();
    void updateRGBA(const cv::Mat &img, GLuint targetId);
    void setMVPMatrix(const cv::Matx44f &mvp);
    
public:
    float vol_dim = 64;
    float tex_dim = sqrtf(vol_dim * vol_dim * vol_dim);
    bool mBufferSwaped;
    ShaderProgram mProgram, mProgram2;
    //        int mUniformMVPMatrix;
    GLuint mTexTSDF, mTexDepth, mTexTSDFBuf, mTexTSDFColor;
    GLuint mFrameBuffer, mFrameBufferBuf;
    GLint mDefaultFBO;
    int mUniformTexColor, mUniformTexDepth, mUniformW2S;
    int mAttribPosCoord;
    int mAttribTexCoord;
    GLuint vbos[2];
    GLuint vao;
};

#endif /* TSDFShader_hpp */

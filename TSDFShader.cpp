//
//  TSDFShader.cpp
//  Scanner
//
//  Created by Neil on 11/02/2017.
//  Copyright © 2017 Neil. All rights reserved.
//

#include "TSDFShader.hpp"

bool TSDFShader::init()
{
    glGenBuffers(2, vbos);
    glGenVertexArrays(1, &vao);
    
    bool res = true;
    
    const char *uniforms[] = { "rgb",  "depth", "w2s"};
//    const char *uniforms2[] = { "tsdf",  "color", "origin"};
    int *uniformIds[] = { &mUniformTexColor, &mUniformTexDepth, &mUniformW2S};
//    int *uniformIds2[] = { &mUniformTexTSDF2, &mUniformTexDepth2, &mOrigin};
    const int uniformsCount = sizeof(uniforms) / sizeof(uniforms[0]);
//    const int uniformsCount2 = sizeof(uniforms2) / sizeof(uniforms2[0]);
    
    const char *attribs[] =	{ "aPosCoord", "aTexCoord" };
    int *attribIds[] = {&mAttribPosCoord, &mAttribTexCoord};
//    int *attribIds2[] = {&mAttribPosCoord2, &mAttribTexCoord2};
    const int attribsCount = sizeof(attribs) / sizeof(attribs[0]);
    
    
    glGenFramebuffers(1, &mFrameBuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, mFrameBuffer);

    glGenTextures(1, &mTexTSDF);
    //        glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, mTexTSDF);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, tex_dim, tex_dim, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);

    glBindTexture(GL_TEXTURE_2D, 0);
    //        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mTexTSDF, 0);

    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if(status != GL_FRAMEBUFFER_COMPLETE)
        printf("Frame buffer ERROR\n");

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    
    res &= mProgram.load("./shaders/tsdf_update.vert", "./shaders/tsdf_update.frag", uniforms, uniformIds, uniformsCount,
                         attribs, attribIds, attribsCount);
    
    float vertices[] =
    { -1., -1., -1., 1., 1.0, -1., 1., 1.};
    float textureCoords[] =
    { 0, 0.f, 0, 1, 1.f, 0.f, 1.f, 1 };
    
    // set up test data
    glBindVertexArray(vao);
    
    glEnableVertexAttribArray(mAttribPosCoord);
    glBindBuffer(GL_ARRAY_BUFFER, vbos[0]);
    glBufferData(GL_ARRAY_BUFFER, 8 * sizeof(float), vertices, GL_STATIC_DRAW);
    glVertexAttribPointer(mAttribPosCoord, 2, GL_FLOAT, GL_FALSE, 0, NULL);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    
    glEnableVertexAttribArray(mAttribTexCoord);
    glBindBuffer(GL_ARRAY_BUFFER, vbos[1]);
    glBufferData(GL_ARRAY_BUFFER, 8 * sizeof(float), textureCoords, GL_STATIC_DRAW);
    glVertexAttribPointer(mAttribTexCoord, 2, GL_FLOAT, GL_FALSE, 0, NULL);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    
    glBindVertexArray(0);
    
    glUseProgram(mProgram.getId());
    glUniform1i(mUniformTexColor, 0);
    glUniform1i(mUniformTexDepth, 1);
    
//    res &= mProgram2.load([[[NSBundle mainBundle] pathForResource:@"tsdf2" ofType:@"vert"] UTF8String], [[[NSBundle mainBundle] pathForResource:@"tsdf2" ofType:@"frag"] UTF8String], uniforms2, uniformIds2, uniformsCount2,
//                          attribs, attribIds2, 2);
    
//    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &mDefaultFBO);
    
    return res;
    
}

void TSDFShader::setMVPMatrix(const cv::Matx44f &mvp)
{
    //Transpose to opengl column-major format
    cv::Matx44f mvpt = mvp.t();
    glUseProgram(mProgram.getId());
    glUniformMatrix4fv(mUniformW2S, 1, false, mvpt.val);
}

void TSDFShader::update(const cv::Point2f center[], int count, const cv::Vec4f &color, float size, float aspect)
{
    //        glEnable(GL_TEXTURE_2D);
    //        glActiveTexture(GL_TEXTURE0);
    //        glBindTexture(GL_TEXTURE_2D, 0);
    
    glViewport(0, 0, tex_dim, tex_dim);
    glEnable(GL_TEXTURE_2D);
    
    if (mBufferSwaped) {
        glBindFramebuffer(GL_FRAMEBUFFER, mFrameBuffer);
    } else {
        glBindFramebuffer(GL_FRAMEBUFFER, mFrameBufferBuf);
    }
    
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glUseProgram(mProgram.getId());
    
    
    glActiveTexture(GL_TEXTURE0);
    if (mBufferSwaped) {
        glBindTexture(GL_TEXTURE_2D, mTexTSDFBuf);
    } else {
        glBindTexture(GL_TEXTURE_2D, mTexTSDF);
    }
    
//    glUniform1i(mUniformTexTSDF, 0);
    
    cv::Vec2f const vertices[] =
    { cv::Vec2f(-1., -1.), cv::Vec2f(-1., 1.), cv::Vec2f(1., -1.), cv::Vec2f(1., 1.)};
    cv::Vec2f const textureCoords[] =
    { cv::Vec2f(0, 1), cv::Vec2f(0, 0), cv::Vec2f(1, 1), cv::Vec2f(1, 0) };
    
    // drawing quad
    glVertexAttribPointer(mAttribPosCoord, 2, GL_FLOAT, GL_FALSE, 0, vertices);
    glVertexAttribPointer(mAttribTexCoord, 2, GL_FLOAT, GL_FALSE, 0, textureCoords);
    glEnableVertexAttribArray(mAttribPosCoord);
    glEnableVertexAttribArray(mAttribTexCoord);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glDisableVertexAttribArray(mAttribPosCoord);
    glDisableVertexAttribArray(mAttribTexCoord);
    
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    
}

void TSDFShader::drawFrameBuffer()
{
    glViewport(0, 0, tex_dim, tex_dim);
    glUseProgram(mProgram.getId());
    glBindFramebuffer(GL_FRAMEBUFFER, mFrameBuffer);
    glBindVertexArray(vao);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glBindVertexArray(0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void TSDFShader::updateRGBA(const cv::Mat &img, GLuint targetId)
{
    assert(img.channels() == 4);
    
    glBindTexture(GL_TEXTURE_2D, targetId);
    
    if(!img.isContinuous())
    {
        glPixelStorei(GL_UNPACK_ROW_LENGTH, (int)img.step[0] / 4);
    }
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, img.cols, img.rows, GL_RGBA, GL_UNSIGNED_BYTE, img.data);
    if(!img.isContinuous())
    {
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    }
    
}

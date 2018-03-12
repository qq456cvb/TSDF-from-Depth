//
//  ColorShader.cpp
//  Tracking3D
//
//  Created by Neil on 17/10/2017.
//  Copyright © 2017 Neil. All rights reserved.
//

#include "ColorShader.h"


bool ColorShader::init()
{
    bool res = true;
    
    const char *uniforms[] = { "uMVPMatrix" };
    int *uniformIds[] = {&mUniformMVPMatrix};
    const int uniformsCount = sizeof(uniforms) / sizeof(uniforms[0]);
    
    const char *attribs[] =    { "aPosCoord", "aColor" };
    int *attribIds[] = {&mAttribPosCoord, &mAttribColor };
    const int attribsCount = sizeof(attribs) / sizeof(attribs[0]);
    
    glGenBuffers(2, vbos);
    res &= mProgram.load("shaders/color_render.vert", "shaders/color_render.frag", uniforms, uniformIds, uniformsCount,
                         attribs, attribIds, attribsCount);
    glGenVertexArrays(1, &vao);
    
    return res;
}

void ColorShader::setMVPMatrix(const cv::Matx44f &mvp)
{
    //Transpose to opengl column-major format
    cv::Matx44f mvpt = mvp.t();
    glUseProgram(mProgram.getId());
    glUniformMatrix4fv(mUniformMVPMatrix, 1, false, mvpt.val);
}

void ColorShader::drawVertices(GLenum mode, const cv::Point2f *vertices, int count, const cv::Vec4f &color)
{
    std::vector<cv::Vec4f> colors;
    colors.resize(count, color);
    
    drawVertices(mode, vertices, colors.data(), count);
}

void ColorShader::drawVertices(GLenum mode, const cv::Point2f *vertices, const cv::Vec4f *color, int count)
{
    glUseProgram(mProgram.getId());
    
    glVertexAttribPointer(mAttribPosCoord, 2, GL_FLOAT, GL_FALSE, 0, vertices);
    glEnableVertexAttribArray(mAttribPosCoord);
    
    glVertexAttribPointer(mAttribColor, 4, GL_FLOAT, GL_FALSE, 0, color);
    glEnableVertexAttribArray(mAttribColor);
    
    glDrawArrays(mode, 0, count);
    
    glDisableVertexAttribArray(mAttribPosCoord);
    glDisableVertexAttribArray(mAttribColor);
}

void ColorShader::drawVertices(GLenum mode, const cv::Vec4f *vertices, int count, const cv::Vec4f &color)
{
    std::vector<cv::Vec4f> colors;
    colors.resize(count, color);
    
    drawVertices(mode, vertices, colors.data(), count);
}

void ColorShader::drawVertices(GLenum mode, const cv::Vec4f *vertices, const cv::Vec4f *color, int count)
{
    glUseProgram(mProgram.getId());
    glBindVertexArrayAPPLE(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbos[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(cv::Vec4f) * count, vertices, GL_STATIC_DRAW);
    glVertexAttribPointer(mAttribPosCoord, 4, GL_FLOAT, GL_FALSE, 0, NULL);
    glEnableVertexAttribArray(mAttribPosCoord);
    
    glBindBuffer(GL_ARRAY_BUFFER, vbos[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(cv::Vec4f) * count, color, GL_STATIC_DRAW);
    glVertexAttribPointer(mAttribColor, 4, GL_FLOAT, GL_FALSE, 0, NULL);
    glEnableVertexAttribArray(mAttribColor);
    
    glDrawArrays(mode, 0, count);
    
    glDisableVertexAttribArray(mAttribPosCoord);
    glDisableVertexAttribArray(mAttribColor);
    glBindVertexArrayAPPLE(0);
}

void ColorShader::drawVertices(GLenum mode, const unsigned int *indices, unsigned int indexCount, const cv::Vec4f *vertices, const cv::Vec4f *color)
{
    glUseProgram(mProgram.getId());
    
    glVertexAttribPointer(mAttribPosCoord, 4, GL_FLOAT, GL_FALSE, 0, vertices);
    glEnableVertexAttribArray(mAttribPosCoord);
    
    glVertexAttribPointer(mAttribColor, 4, GL_FLOAT, GL_FALSE, 0, color);
    glEnableVertexAttribArray(mAttribColor);
    
    glDrawElements(mode, indexCount, GL_UNSIGNED_INT, indices);
    
    glDisableVertexAttribArray(mAttribPosCoord);
    glDisableVertexAttribArray(mAttribColor);
}

void ColorShader::drawRect(const cv::Point2f center[], int count, const cv::Vec4f &color, float size, float aspect)
{
    float sizeh = aspect * size / 2;
    float sizev = size / 2;
    float vertex[8];
    cv::Vec4f colors[4] = {color,color,color,color};
    
    glUseProgram(mProgram.getId());
    glVertexAttribPointer(mAttribColor, 4, GL_FLOAT, GL_FALSE, 0, colors);
    glEnableVertexAttribArray(mAttribColor);
    
    glVertexAttribPointer(mAttribPosCoord, 2, GL_FLOAT, GL_FALSE, 0, vertex);
    glEnableVertexAttribArray(mAttribPosCoord);
    
    for (int i = 0; i < count; i++)
    {
        const cv::Point2f &c = center[i];
        vertex[0] = c.x - sizeh;
        vertex[1] = c.y - sizev;
        vertex[2] = c.x + sizeh;
        vertex[3] = c.y - sizev;
        vertex[4] = c.x + sizeh;
        vertex[5] = c.y + sizev;
        vertex[6] = c.x - sizeh;
        vertex[7] = c.y + sizev;
        glDrawArrays(GL_LINE_LOOP, 0, 4);
    }
    
    glDisableVertexAttribArray(mAttribPosCoord);
    glDisableVertexAttribArray(mAttribColor);
}

void ColorShader::colorTest(float points[]) {
    
    
    
}


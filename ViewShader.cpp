//
//  ViewShader.cpp
//  Scanner
//
//  Created by Neil on 01/11/2017.
//  Copyright © 2017 Neil. All rights reserved.
//

#include "ViewShader.hpp"
bool ViewShader::init() {
    bool res = true;
    const char *attribs[] =    { "aPosCoord", "aTexCoord" };
    int *attribIds[] = {&mAttribPosCoord, &mAttribTexCoord};
    const int attribsCount = sizeof(attribs) / sizeof(attribs[0]);
    
    const char *uniforms[] = { "tsdf", "s2w", "c"};
    int *uniformIds[] = { &mUniformTex, &mUniformS2W, &mUniformC};
    const int uniformsCount = sizeof(uniforms) / sizeof(uniforms[0]);
    
    res &= mProgram.load("shaders/tsdf_render.vert", "shaders/tsdf_render.frag", uniforms, uniformIds, uniformsCount, attribs, attribIds, attribsCount);
    glGenBuffers(2, vbos);
    glGenVertexArrays(1, &vao);
    
    float vertices[] =
    { -1., -1., -1., 1., 1.0, -1., 1., 1.};
    float textureCoords[] =
    { 0, 1.f, 0, 0, 1.f, 1.f, 1.f, 0 };
    
    
    glBindVertexArray(vao);
    
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    
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
    glUniform1i(mUniformTex, 0);
    return res;
}

void ViewShader::setS2WMatrix(const cv::Matx44f &mat) {
    //Transpose to opengl column-major format
    cv::Matx44f mat_t = mat.t();
    glUseProgram(mProgram.getId());
    glUniformMatrix4fv(mUniformS2W, 1, false, mat_t.val);
}

void ViewShader::setCameraCenter(float *origin) {
    glUseProgram(mProgram.getId());
    glUniform3fv(mUniformC, 1, origin);
}

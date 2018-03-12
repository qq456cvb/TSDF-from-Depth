//
//  TextureShader.cpp
//  Scanner
//
//  Created by Neil on 30/10/2017.
//  Copyright © 2017 Neil. All rights reserved.
//

#include "TextureShader.hpp"

bool TextureShader::init() {
    bool res = true;
    const char *attribs[] =    { "aPosCoord", "aTexCoord" };
    int *attribIds[] = {&mAttribPosCoord, &mAttribTexCoord};
    const int attribsCount = sizeof(attribs) / sizeof(attribs[0]);
    
    const char *uniforms[] = { "tex"};
    int *uniformIds[] = { &mUniformTex};
    const int uniformsCount = sizeof(uniforms) / sizeof(uniforms[0]);
    
    res &= mProgram.load("shaders/tex.vert", "shaders/tex.frag", uniforms, uniformIds, uniformsCount, attribs, attribIds, attribsCount);
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

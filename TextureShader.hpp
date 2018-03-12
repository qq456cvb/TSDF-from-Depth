//
//  TextureShader.hpp
//  Scanner
//
//  Created by Neil on 30/10/2017.
//  Copyright © 2017 Neil. All rights reserved.
//

#ifndef TextureShader_hpp
#define TextureShader_hpp

#include <stdio.h>
#include <opencv2/core.hpp>
#include "ShaderProgram.h"
class TextureShader
{
public:
    TextureShader() {
        init();
    }
    ~TextureShader() {
        free();
    }
    
    bool init();
    void free() {mProgram.free();}
    
    //mvp is in normal opencv row-major order
    
    GLuint vbos[2];
    GLuint vao;
    ShaderProgram mProgram;
    int mUniformTex;
    int mAttribPosCoord;
    int mAttribTexCoord;
protected:
};
#endif /* TextureShader_hpp */

//
//  ViewShader.hpp
//  Scanner
//
//  Created by Neil on 01/11/2017.
//  Copyright © 2017 Neil. All rights reserved.
//

#ifndef ViewShader_hpp
#define ViewShader_hpp

#include <stdio.h>
#include <opencv2/core.hpp>
#include "ShaderProgram.h"
class ViewShader
{
public:
    ViewShader() {
        init();
    }
    ~ViewShader() {
        free();
    }
    
    bool init();
    void free() {mProgram.free();}
    
    //mvp is in normal opencv row-major order
    
    GLuint vbos[2];
    GLuint vao;
    ShaderProgram mProgram;
    int mUniformTex, mUniformS2W, mUniformC;
    int mAttribPosCoord;
    int mAttribTexCoord;
    
    void setS2WMatrix(const cv::Matx44f &mat);
    void setCameraCenter(float *origin);
protected:
};
#endif /* ViewShader_hpp */

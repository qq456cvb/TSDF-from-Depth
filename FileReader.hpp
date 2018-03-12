//
//  FileReader.hpp
//  Scanner
//
//  Created by Neil on 11/02/2017.
//  Copyright © 2017 Neil. All rights reserved.
//

#ifndef FileReader_hpp
#define FileReader_hpp

#include <stdio.h>
#include "opencv2/opencv.hpp"

class FileReader {
    void prefilterDepth(cv::Mat& depth);
    
public:
    cv::Mat readDepth(const char* file);
};

#endif /* FileReader_hpp */

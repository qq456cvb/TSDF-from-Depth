//
//  FileReader.cpp
//  Scanner
//
//  Created by Neil on 11/02/2017.
//  Copyright © 2017 Neil. All rights reserved.
//

#include "FileReader.hpp"

cv::Mat FileReader::readDepth(const char *file) {
    auto img = cv::imread(file, CV_16U);
    prefilterDepth(img);
    
    return img;
}

void FileReader::prefilterDepth(cv::Mat &depth) {
    static int filter_width = 3;
    cv::Mat out = cv::Mat(depth.rows, depth.cols, CV_16U);
    
//    cv::imshow("depth", depth);
    unsigned long long mean = 0;
    int mean_cnt = 0;
    for (auto i = filter_width/2; i < depth.rows-filter_width/2; i++) {
        for (auto j = filter_width/2; j < depth.cols-filter_width/2; j++) {
            int val = 0, cnt = 0;
            for (int m = -filter_width/2; m <= filter_width/2 ; m++) {
                for (int n = -filter_width/2; n <= filter_width/2 ; n++) {
                    auto pixel = depth.at<uint16_t>(i+m, j+n);
                    if (pixel != 0) {
                        val += pixel;
                        cnt++;
                    }
                }
            }
            if (cnt > 0) {
                out.at<uint16_t>(i, j) = val / cnt;
            } else {
                out.at<uint16_t>(i, j) = depth.at<uint16_t>(i, j);
            }
            if (depth.at<uint16_t>(i, j) != 0) {
                mean += depth.at<uint16_t>(i, j);
                mean_cnt++;
            }
        }
    }
    
    printf("mean value: %llu\n", mean / mean_cnt);
    
    depth = out;
    cv::imshow("test", depth);
    cv::waitKey(30);
}

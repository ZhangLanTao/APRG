#ifndef HIRACHICALPLANESEGMENTATION_ITODD_DATA_TOOLS_HPP
#define HIRACHICALPLANESEGMENTATION_ITODD_DATA_TOOLS_HPP

#include "Params.h"
#include "document.h"

#include <fstream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <vector>


cv::Mat readDepthFile_ICL(const std::string &filename, Params params) {
    cv::Mat depth_img = cv::Mat::zeros(params.img_height, params.img_width, CV_32FC1);

    std::ifstream depthfile;
    depthfile.open(filename, std::ios::in);

    for (int i = 0; i < params.img_height; i++) {
        for (int j = 0; j < params.img_width; j++) {
            float val = 0;
            depthfile >> val;
            float z = 1000 * (params.fx + params.fy) / 2 *
                      sqrt((val * val) / (params.fx * params.fy + (i - params.cy) * (i - params.cy) +
                                          (j - params.cx) * (j - params.cx)));

            depth_img.at<float>(i, j) = z;
        }
    }
    depthfile.close();
    return depth_img;
}



#endif // HIRACHICALPLANESEGMENTATION_ITODD_DATA_TOOLS_HPP

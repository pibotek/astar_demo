#ifndef NAVIGATION_DISPLAY_H_
#define NAVIGATION_DISPLAY_H_

#include <vector>
#include <opencv2/opencv.hpp>

void Display(const cv::Mat& map_data,
             cv::Point begin,
             cv::Point end,
             const std::vector<cv::Point>& path,
             const std::vector<cv::Point>& close_list = {});
#endif

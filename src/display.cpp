#include "display.h"
#include <opencv2/imgproc/imgproc_c.h>

void Display(const cv::Mat& map_data,
             cv::Point begin,
             cv::Point end,
             const std::vector<cv::Point>& path,
             const std::vector<cv::Point>& close_list) {
  int ROWS = map_data.rows + 2;
  int COLS = map_data.cols + 2;

  int Size = 30;
  if (ROWS < 15 && COLS < 15) {
    Size = 30;
  } else {
    Size = 8;
  }
  int Menu = Size / 2;

  cv::Mat img(ROWS * Size, COLS * Size, CV_8UC3, cv::Scalar(0x96, 0x96, 0x96));
  cv::Point left_up, right_bottom;
  cv::Point point_first, point_second;

  for (int i = 1; i < ROWS - 1; i++) {
    for (int j = 1; j < COLS - 1; j++) {
      left_up.x = j * Size;
      left_up.y = i * Size;
      right_bottom.x = left_up.x + Size;
      right_bottom.y = left_up.y + Size;
      if (map_data.at<char>(i - 1, j - 1)) {
        cv::rectangle(img, left_up, right_bottom, cv::Scalar(0, 0, 0), CV_FILLED, 8, 0);
      } else {
        if (j - 1 == begin.x && i - 1 == begin.y)
          cv::rectangle(img, left_up, right_bottom, cv::Scalar(0, 255, 0), CV_FILLED, 8, 0);
        else if (j - 1 == end.x && i - 1 == end.y)
          cv::rectangle(img, left_up, right_bottom, cv::Scalar(0, 0, 255), CV_FILLED, 8, 0);
      }
    }
  }

  // 中间线
  for (int i = 1; i < COLS; i++) {
    point_first.x = i * Size;
    point_first.y = 1 * Size;
    point_second.x = i * Size;
    point_second.y = (ROWS - 1) * Size;
    cv::line(img, point_first, point_second, cv::Scalar(141, 168, 168), 2, 2);
  }
  for (int i = 1; i < ROWS; i++) {
    point_first.x = 1 * Size;
    point_first.y = i * Size;
    point_second.x = (COLS - 1) * Size;
    point_second.y = i * Size;
    cv::line(img, point_first, point_second, cv::Scalar(141, 168, 168), 2, 2);
  }

  // close list
  if (!close_list.empty()) {
    for (auto& it : close_list) {
      left_up.x = (it.x + 1) * Size + Menu;
      left_up.y = (it.y + 1) * Size + Menu;
      cv::circle(img, left_up, Size / 5, cv::Scalar(255, 0, 0), -1);
    }
  }

  // 路径线
  if (!path.empty()) {
    auto temp_path = path;
    point_first.x = (begin.x + 1) * Size + Menu;
    point_first.y = (begin.y + 1) * Size + Menu;

    std::reverse(temp_path.begin(), temp_path.end());
    temp_path.emplace_back(end.x, end.y);
    for (auto& it : temp_path) {
      left_up.x = (it.x + 1) * Size;
      left_up.y = (it.y + 1) * Size;
      point_second.x = left_up.x + Menu;
      point_second.y = left_up.y + Menu;
      cv::line(img, point_first, point_second, cv::Scalar(0, 255, 255), 2, 4);

      cv::circle(img, point_first, Size / 10, cv::Scalar(50, 50, 250), -1);
      point_first = point_second;
    }
  }

  cv::imshow("planner", img);
  cv::moveWindow("planner", 200, 200);

  cv::waitKey(1);
}

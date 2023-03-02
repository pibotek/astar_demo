#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include "display.h"
#include "astar_planner.h"
#include "theta_star.hpp"
#include "stopwatch.h"
#include <thread>

void PlannerActionCB(const cv::Mat& map_data, const cv::Point begin, const cv::Point end, const std::vector<cv::Point>& close_list, const std::vector<cv::Point>& path) {
  Display(map_data, begin, end, path, close_list);
}

int main(int argc, char* argv[]) {

#if USE_THETA_STAR
  std::shared_ptr<navigation::Planner> planner = std::make_shared<navigation::ThetaStar>();
#else
  std::shared_ptr<navigation::Planner> planner = std::make_shared<navigation::AStarPlanner>();
#endif

#if USE_MINI_SAMPLE
  const int col = 8;
  const int row = 6;
  unsigned char map_data[row][col] = {0};

  //  map_data[0][2] = navigation::kLethalObstacle;
  //  map_data[1][2] = navigation::kLethalObstacle;
  map_data[2][2] = navigation::kLethalObstacle;
  map_data[3][2] = navigation::kLethalObstacle;
  map_data[4][2] = navigation::kLethalObstacle;
  map_data[5][2] = navigation::kLethalObstacle;

  map_data[0][5] = navigation::kLethalObstacle;
  map_data[1][5] = navigation::kLethalObstacle;
  map_data[2][5] = navigation::kLethalObstacle;
  map_data[3][5] = navigation::kLethalObstacle;
  map_data[4][5] = navigation::kLethalObstacle;

  Point begin = {1, 4};
  Point end = {6, 3};

  cv::Mat mat(row, col, CV_8UC1, (void*)map_data);
#else
  cv::Mat mat = cv::imread("../map/map_demo.png", cv::IMREAD_GRAYSCALE);

  cv::threshold(mat, mat, 128, 255, CV_THRESH_BINARY_INV);

  Point begin = {1, 4};
  Point end = {76, 75};

  for (int i = 0; i < mat.rows; i++) {
    for (int j = 0; j < mat.cols; j++) {
      if (j == begin.x && i == begin.y) {
        printf("b ");
      } else if (j == end.x && i == end.y) {
        printf("e ");
      } else {
        if (mat.at<unsigned char>(i, j) >= 1) {
          printf("* ");
        } else {
          printf("  ");
        }
      }
    }
    printf("\n");
  }

#endif

  std::vector<Point> path;
  Stopwatch stopwatch;
  bool got = planner->Plan(mat, begin, end, path, PlannerActionCB);
  std::cout << std::boolalpha << "plan result:" << got << ", elapsed:" << stopwatch.elapsed().count() << "ms" << std::endl;
  if (got) {
#if !DEBUG_PLANNER_DETAIL
    Display(mat, begin, end, path);
#endif
  }

  cv::waitKey(0);
  cv::destroyAllWindows();

  return 0;
}

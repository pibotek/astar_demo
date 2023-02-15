#ifndef NAVIGATION_ASTAR_PLANNER_H_
#define NAVIGATION_ASTAR_PLANNER_H_

#include <queue>
#include <functional>
#include <unordered_map>
#include <opencv2/opencv.hpp>
using namespace cv;

namespace navigation {

using PlannerAction = std::function<void(const Mat&, const Point, const Point, const std::vector<Point>&, const std::vector<Point>&)>;

// map grid val
static const unsigned char kNoInformation = 255;
static const unsigned char kLethalObstacle = 254;
static const unsigned char kInscribedInflatedObstacle = 253;
static const unsigned char kFreeSpace = 0;

const static int kLineCost = 10;
const static int kCornerCost = 14;

class Grid {
 public:
  Grid(Point p = Point(), float g = FLT_MAX, float h = 0) {
    point_ = p;
    parent_point_ = p;

    g_ = g;
    h_ = h;

    is_close_ = false;
  }

  inline void Update(Point point, bool close) {
    point_ = point;
    is_close_ = close;
  }

  inline void Update(Point point, Point parent_point, float g, float h) {
    parent_point_ = parent_point;
    point_ = point;
    g_ = g;
    h_ = h;
  }

  Point parent_point_;
  Point point_;
  float g_;
  float h_;  // f = g + h

  bool is_close_ = false;
};

class AStarPlanner {
 public:
  AStarPlanner();
  virtual ~AStarPlanner();

  bool Plan(const Mat& cost_map, const Point& start_point, const Point& end_point, std::vector<Point>& path, PlannerAction planner_action);

 private:
  float CalcG(const Point& start_point, const Point& current_point, float parent_g = 0.0f);
  float CalcH(const Point& current_point, const Point& end_point);

  bool PointValid(const Point& point);

  bool Point2Index(const Point& point, int& index);
  bool Index2Point(int index, Point& point);


  bool GetPathFromGrid(const Point& start_point, const Point& end_point, std::vector<Point>& path);

private:
  inline bool InRange(const Mat& image, int mx, int my) {
    return (mx >= 0 && mx < image.cols && my >= 0 && my < image.rows);
  }

  inline bool InRange(const Mat& image, const Point& point) {
    return InRange(image, point.x, point.y);
  }

  inline bool InRange(const Mat& image, int index) {
    return InRange(image, index % image.cols, index / image.cols);
  }

  inline bool InRange(const Rect& rect, const Point& point) {
    return (point.x >= rect.tl().x && point.x <= rect.br().x && point.y >= rect.tl().y && point.y <= rect.br().y);
  }

 private:
  bool InCloseList(const Point& point);

  bool CalcPotentials(const Mat& cost_map, const Point& start_point, const Point& end_point, int max_num, PlannerAction planner_action);
  std::vector<Point> GetNeighbors(const Mat& cost_map, const Point& current_point);

  void InitNeighborInfo();

  // neighbors
  std::vector<Point> neighbors_;

  // map
  Mat cost_map_;

  struct greater {
    bool operator()(const Grid& g1, const Grid& g2) const {
      float f1 = g1.h_ + g1.g_;
      float f2 = g2.h_ + g2.g_;

      return f1 > f2 || (f1 == f2 && g1.g_ < g2.g_);
    }
  };

  std::priority_queue<Grid, std::vector<Grid>, greater> open_list_;

  std::unordered_map<int, Grid> all_grid_;

#if DEBUG_PLANNER_DETAIL
  std::vector<Point> close_list_;
#endif
};

}  // namespace navigation
#endif
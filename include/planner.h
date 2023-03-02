#ifndef NAVIGATION_PLANNER_H_
#define NAVIGATION_PLANNER_H_

#include <functional>
#include <opencv2/opencv.hpp>
using namespace cv;

namespace navigation {

using PlannerAction = std::function<void(const Mat&, const Point, const Point, const std::vector<Point>&, const std::vector<Point>&)>;

// map grid val
static const unsigned char kNoInformation = 255;
static const unsigned char kLethalObstacle = 254;
static const unsigned char kInscribedInflatedObstacle = 253;
static const unsigned char kFreeSpace = 0;

class Planner {
 public:
  Planner() = default;
  virtual ~Planner() = default;

  virtual bool Plan(const Mat& cost_map, const Point& start_point, const Point& end_point, std::vector<Point>& path, PlannerAction planner_action) = 0;

 protected:
  Mat cost_map_;
};

}  // namespace navigation
#endif
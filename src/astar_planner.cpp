#include "astar_planner.h"

namespace navigation {

AStarPlanner::AStarPlanner() {
  InitNeighborInfo();
}

AStarPlanner::~AStarPlanner() {}

float AStarPlanner::CalcG(const Point& start_point, const Point& current_point, float parent_g) {
  int diff_x = abs(start_point.x - current_point.x);
  int diff_y = abs(start_point.y - current_point.y);

  int min_diff = std::min(diff_x, diff_y);
  int max_diff = std::max(diff_x, diff_y);

  return min_diff * kCornerCost + (max_diff - min_diff) * kLineCost + parent_g;
}

float AStarPlanner::CalcH(const Point& current_point, const Point& end_point) {
  int diff_x = abs(end_point.x - current_point.x);
  int diff_y = abs(end_point.y - current_point.y);

  int min_diff = std::min(diff_x, diff_y);
  int max_diff = std::max(diff_x, diff_y);

  return min_diff * kCornerCost + (max_diff - min_diff) * kLineCost;
}

bool AStarPlanner::Point2Index(const Point& point, int& index) {
  if (!InRange(cost_map_, point)) {
    return false;
  }

  index = point.x + point.y * cost_map_.cols;
  return true;
}

bool AStarPlanner::Index2Point(int index, Point& point) {
  if (!InRange(cost_map_, index)) {
    return false;
  }

  point.x = index % cost_map_.cols;
  point.y = index / cost_map_.cols;
  return true;
}

bool AStarPlanner::InCloseList(const Point& point) {
  int index;
  if (!Point2Index(point, index)) {
    return false;
  }

  return all_grid_[index].is_close_;
}

bool AStarPlanner::PointValid(const Point& point) {
  if (InCloseList(point)) {
    return false;
  }

  if (cost_map_.at<unsigned char>(point.y, point.x) >= kLethalObstacle) {  // >= 254 为障碍物
    return false;
  }

  return true;
}

bool AStarPlanner::Plan(const Mat& cost_map, const Point& start_point, const Point& end_point, std::vector<Point>& path, PlannerAction planner_action) {
  cost_map_ = cost_map.clone();

  all_grid_.clear();

  bool ret = CalcPotentials(cost_map_, start_point, end_point, cost_map.cols * cost_map_.rows, planner_action);
  if (!ret) {
    return false;
  }

  ret = GetPathFromGrid(start_point, end_point, path);

  if (!ret) {
    return false;
  }

#if DEBUG_PLANNER_DETAIL
  if (planner_action) {
    planner_action(cost_map, start_point, end_point, close_list_, path);
  }
#endif

  return true;
}

bool AStarPlanner::CalcPotentials(const Mat& cost_map, const Point& start_point, const Point& end_point, int max_num, PlannerAction planner_action) {
  // 清除open list 放置起点
  decltype(open_list_) tmp;
  open_list_.swap(tmp);
  open_list_.emplace(start_point, 0, CalcH(start_point, end_point));

  int cycle_num = 0;

#if DEBUG_PLANNER_DETAIL
  close_list_.clear();
#endif

  while (!open_list_.empty()) {
    Grid top = open_list_.top();
    open_list_.pop();

    Point point = top.point_;
    int index;
    if (!Point2Index(point, index)) {
      return false;
    }

    if (point == end_point) {
      return true;
    }

    all_grid_[index].Update(point, true);

#if DEBUG_PLANNER_DETAIL
    close_list_.emplace_back(all_grid_[index].point_.x, all_grid_[index].point_.y);
    if (planner_action) {
      planner_action(cost_map, start_point, end_point, close_list_, {});
    }
#endif

    std::vector<Point> neighbors = GetNeighbors(cost_map, point);

    // 循环当前点的邻域
    for (auto& neighbor : neighbors) {
      if (!Point2Index(neighbor, index)) {
        continue;
      }

      float child_origin_g = all_grid_[index].g_;
      float G = CalcG(point, neighbor, top.g_);

      // 计算g 小于之前的，则更新
      if (G < child_origin_g) {
        all_grid_[index].Update(neighbor, point, G, CalcH(neighbor, end_point));

        open_list_.push(all_grid_[index]);
      }
    }

    if (cycle_num++ >= max_num) {
      break;
    }
  }

  return false;
}

bool AStarPlanner::GetPathFromGrid(const Point& start_point, const Point& end_point, std::vector<Point>& path) {
  path.clear();

  path.push_back(end_point);

  int start_index;
  bool ret = Point2Index(start_point, start_index);
  if (!ret) {
    return false;
  }

  int index;
  Point point = end_point;
  ret = Point2Index(point, index);
  if (!ret) {
    return false;
  }

  while (index != start_index) {
    point = all_grid_[index].parent_point_;
    path.push_back(point);

    Point2Index(point, index);
  }

  return true;
}

std::vector<Point> AStarPlanner::GetNeighbors(const Mat& cost_map, const Point& current_point) {
  std::vector<Point> neighbor_points;

  for (auto& neighbor : neighbors_) {
    Point new_point(current_point.x + neighbor.x, current_point.y + neighbor.y);

    if (!PointValid(new_point)) {
      continue;
    }

    neighbor_points.push_back(new_point);
  }

  return neighbor_points;
}

void AStarPlanner::InitNeighborInfo() {
  // 取8邻域
  for (int dx = -1; dx <= 1; dx++) {
    for (int dy = -1; dy <= 1; dy++) {
      if (0 == dx && 0 == dy) {
        continue;
      }

      neighbors_.emplace_back(dx, dy);
    }
  }
}

}  // namespace navigation
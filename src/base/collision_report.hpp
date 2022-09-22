/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include <map>
#include <set>

#include "base/geometry_2D.hpp"

class Buckets2D : public Geometry2D {
  std::map<std::pair<int, int>, std::vector<int>> buckets;
  std::vector<Point> points;
  double radius;

 public:
  Buckets2D(const double& radius, const std::vector<Point>& points) {
    this->points = points;
    this->radius = radius;
    for (int i = 0; i < points.size(); i++) {
      buckets[std::make_pair(std::floor(points[i].x / radius),
                             std::floor(points[i].y / radius))]
          .push_back(i);
    }
  }
  const int dx[5] = {0, 0, 1, 1, 1}, dy[5] = {0, 1, -1, 0, 1};

  std::set<std::pair<int, int>> calculateAllPairs() {
    std::set<std::pair<int, int>> pair_list;
    for (auto& element : buckets) {
      auto& bucket = element.first;
      for (int v = 0; v < 5; v++) {
        auto neighbor_bucket =
            std::make_pair(bucket.first + dx[v], bucket.second + dy[v]);
        auto neighbor_it = buckets.find(neighbor_bucket);
        if (neighbor_it == buckets.end()) {
          continue;
        }
        for (auto& point_id_0 : element.second) {
          for (auto& point_id_1 : neighbor_it->second) {
            if (distance(points[point_id_0], points[point_id_1]) < radius) {
              pair_list.insert(
                  std::make_pair(std::min(point_id_0, point_id_1),
                                 std::max(point_id_0, point_id_1)));
            }
          }
        }
      }
    }
    return pair_list;
  }
  std::vector<int> calculateCollisionVertices(const Point& s, const Point& t) {
    Point p0, p1;
    if (s.x <= t.x) {
      p0 = s, p1 = t;
    } else {
      p0 = t, p1 = s;
    }
    Point vec = p1 - p0,
          u = radius / std::sqrt(inner(vec, vec)) * Point(-vec.y, vec.x);
    Point l, m0, m1, r;
    if (p0.y <= p1.y) {
      l = p0 + u;
      m0 = p0 - u;
      m1 = p1 + u;
      r = p1 - u;
    } else {
      l = p0 - u;
      m1 = p0 + u;
      m0 = p1 - u;
      r = p1 + u;
    }
    int L = std::floor(l.x / radius), M0 = std::floor(m0.x / radius),
        M1 = std::floor(m1.x / radius), R = std::floor(r.x / radius);
    std::vector<int> lower(R - L + 1), upper(R - L + 1);
    for (int x = L; x < M0; x++) {
      lower[x - L] = std::floor(
          (l.y + ((x + 1) * radius - l.x) / (m0.x - l.x) * (m0.y - l.y)) /
          radius);
    }
    lower[M0 - L] = std::floor(m0.y / radius);
    for (int x = M0 + 1; x <= R; x++) {
      lower[x - L] = std::floor(
          (m0.y + (x * radius - m0.x) / (r.x - m0.x) * (r.y - m0.y)) / radius);
    }
    for (int x = L; x < M1; x++) {
      upper[x - L] = std::floor(
          (l.y + ((x + 1) * radius - l.x) / (m1.x - l.x) * (m1.y - l.y)) /
          radius);
    }
    upper[M1 - L] = std::floor(m1.y / radius);
    for (int x = M1 + 1; x <= R; x++) {
      upper[x - L] = std::floor(
          (m1.y + (x * radius - m1.x) / (r.x - m1.x) * (r.y - m1.y)) / radius);
    }
    std::vector<int> vertices_list;
    for (int x = L; x <= R; x++) {
      for (int y = lower[x - L]; y <= upper[x - L]; y++) {
        auto list_it = buckets.find(std::make_pair(x, y));
        if (list_it == buckets.end()) continue;
        for (auto& point_id : list_it->second) {
          auto& point = points[point_id];
          double A = inner(point - p0, vec), D = inner(point - p0, u);
          if (0 < A && A < inner(vec, vec) && -radius * radius < D &&
              D < radius * radius) {
            vertices_list.push_back(point_id);
          }
        }
      }
    }
    return vertices_list;
  }
};

#include <cassert>
#include <queue>
#include <random>

const double pi = 2.0 * asin(1.0);

class BentleyOttoman {
  static constexpr double EPS = 1e-11;

  std::vector<std::pair<Geometry2D::Point, Geometry2D::Point>> segments;
  std::set<std::pair<int, int>> cross_pairs;

  enum event_type { emerge, vanish, cross };
  struct Event {
    double time;
    event_type type;
    int segment, segment_1;
    double slope_0, slope_1;
    bool operator<(const Event& event) const {
      if (std::abs(time - event.time) >= EPS) {
        return time > event.time;
      } else if (type == emerge) {
        return event.type != emerge;
      } else if (type == vanish) {
        return event.type == cross;
      } else if (event.type == cross) {
        if (std::abs(slope_0 - event.slope_0) >= EPS) {
          return slope_0 > event.slope_0;
        } else if (std::abs(slope_1 - event.slope_1) >= EPS) {
          return slope_1 > event.slope_1;
        }
        return segment - segment_1 < event.segment - event.segment_1;
      }
      return false;
    }
  };
  std::vector<std::pair<double, double>> lines;
  std::priority_queue<Event> events;

 public:
  BentleyOttoman(
      const std::vector<std::pair<Geometry2D::Point, Geometry2D::Point>>
          segments) {
    std::random_device rd;
    std::default_random_engine eng(rd());
    std::uniform_real_distribution<double> distr(-pi, pi);
    double random_angle = distr(eng), random_sin = sin(random_angle),
           random_cos = cos(random_angle);
    this->segments.resize(segments.size());
    for (int i = 0; i < segments.size(); i++) {
      Geometry2D::Point point_0(
          random_cos * segments[i].first.x - random_sin * segments[i].first.y,
          random_sin * segments[i].first.x + random_cos * segments[i].first.y);
      Geometry2D::Point point_1(
          random_cos * segments[i].second.x - random_sin * segments[i].second.y,
          random_sin * segments[i].second.x +
              random_cos * segments[i].second.y);
      if (point_0.x > point_1.x) {
        std::swap(point_0, point_1);
      }
      this->segments[i] = (std::make_pair(point_0, point_1));
    }
  }

  void checkPair(const int segment_0, const int segment_1) {
    if (std::abs(lines[segment_0].first - lines[segment_1].first) < EPS &&
        std::abs(lines[segment_0].second - lines[segment_1].second) < EPS) {
      cross_pairs.insert(std::make_pair(segment_0, segment_1));
      return;
    }
    if (lines[segment_0].first <= lines[segment_1].first + EPS) return;
    if (cross_pairs.find(std::make_pair(segment_0, segment_1)) !=
        cross_pairs.end())
      return;
    double cross_time = (lines[segment_1].second - lines[segment_0].second) /
                        (lines[segment_0].first - lines[segment_1].first);
    if (cross_time >= segments[segment_0].second.x - EPS ||
        cross_time >= segments[segment_1].second.x - EPS)
      return;
    cross_pairs.insert(std::make_pair(segment_0, segment_1));
    Event crossing;
    crossing.time = cross_time;
    crossing.type = cross;
    crossing.segment = segment_0;
    crossing.slope_0 = lines[segment_0].first;
    crossing.segment_1 = segment_1;
    crossing.slope_1 = lines[segment_1].first;
    events.push(crossing);
  }

  std::set<std::pair<int, int>> calculateAllCrosses() {
    for (int i = 0; i < segments.size(); i++) {
      if (segments[i].first.x == segments[i].second.x) {
        continue;
      }
      Event emerging, vanishing;
      emerging.time = segments[i].first.x;
      emerging.type = emerge;
      emerging.segment = i;
      events.push(emerging);
      vanishing.time = segments[i].second.x;
      vanishing.type = vanish;
      vanishing.segment = i;
      events.push(vanishing);
    }
    lines.resize(segments.size());
    for (int i = 0; i < segments.size(); i++) {
      lines[i].first = (segments[i].second.y - segments[i].first.y) /
                       (segments[i].second.x - segments[i].first.x);
      lines[i].second =
          segments[i].first.y - lines[i].first * segments[i].first.x;
    }
    auto lines_ptr = std::make_shared<decltype(lines)>(lines);
    std::shared_ptr<double> time_ptr(new double);
    std::shared_ptr<std::vector<int>> id_to_segments_ptr(
        new std::vector<int>(segments.size()));
    auto& id_to_segments = *id_to_segments_ptr;
    auto compare_lines = [lines_ptr, time_ptr, id_to_segments_ptr](
                             const int id0, const int id1) -> bool {
      int s0 = (*id_to_segments_ptr)[id0], s1 = (*id_to_segments_ptr)[id1];
      double y0 = (*lines_ptr)[s0].first * *time_ptr + (*lines_ptr)[s0].second;
      double y1 = (*lines_ptr)[s1].first * *time_ptr + (*lines_ptr)[s1].second;
      if (std::abs(y0 - y1) >= EPS)
        return y0 < y1;
      else if (std::abs((*lines_ptr)[s0].first - (*lines_ptr)[s1].first) >= EPS)
        return (*lines_ptr)[s0].first < (*lines_ptr)[s1].first;
      return s0 < s1;
    };
    std::set<int, decltype(compare_lines)> on_line(compare_lines);
    std::vector<decltype(on_line)::iterator> iterators(segments.size());
    int current_id = 0;
    while (!events.empty()) {
      Event event = events.top();
      events.pop();
      *time_ptr = event.time;
      if (event.type == emerge) {
        id_to_segments[current_id] = event.segment;
        auto ret = on_line.insert(current_id);
        assert(ret.second);
        auto& it = ret.first;
        iterators[event.segment] = it;
        if (it != on_line.begin()) {
          checkPair(id_to_segments[*std::prev(it)], event.segment);
        }
        if (std::next(it) != on_line.end()) {
          checkPair(event.segment, id_to_segments[*std::next(it)]);
        }
        current_id++;
      } else if (event.type == vanish) {
        auto& it = iterators[event.segment];
        assert(id_to_segments[*it] == event.segment);
        if (it != on_line.begin() && std::next(it) != on_line.end()) {
          checkPair(id_to_segments[*std::prev(it)],
                    id_to_segments[*std::next(it)]);
        }
        on_line.erase(it);
      } else {
        auto it_0 = iterators[event.segment], it_1 = iterators[event.segment_1];
        assert(id_to_segments[*it_0] == event.segment);
        assert(id_to_segments[*it_1] == event.segment_1);
        if (std::next(it_0) != it_1) {
          event.time += EPS;
          events.push(event);
          continue;
        }
        std::swap(id_to_segments[*it_0], id_to_segments[*it_1]);
        iterators[event.segment_1] = it_0;
        iterators[event.segment] = it_1;
        if (it_0 != on_line.begin()) {
          checkPair(id_to_segments[*std::prev(it_0)], id_to_segments[*it_0]);
        }
        if (std::next(it_1) != on_line.end()) {
          checkPair(id_to_segments[*it_1], id_to_segments[*std::next(it_1)]);
        }
      }
    }
    return cross_pairs;
  }
};

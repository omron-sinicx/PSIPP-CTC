/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#ifndef MAPF_BENCHMARK_GEOMETRY
#define MAPF_BENCHMARK_GEOMETRY

#include <algorithm>
#include <cmath>
#include <random>

class Geometry2D {
 public:
  struct Point {
    double x, y;
    Point() {}
    Point(const double x, const double y) : x(x), y(y) {}
    Point(const double *values) : x(values[0]), y(values[1]) {}
  };
  using AgentGeometry = double;

  static constexpr double EPS = 0.0,
                          INF = std::numeric_limits<double>::infinity();

 private:
  static void solve_linear_inequation(const double &A, const double &B,
                                      double &lower, double &upper) {
    // solve A*x+B<=0 and add its restraints
    if (A < -EPS) {
      lower = std::max(lower, -B / A);
    } else if (A > EPS) {
      upper = std::min(upper, -B / A);
    } else if (B > 0) {
      lower = INF;
      upper = -INF;
    }
  }
  static void solve_quadratic_inequation(const double &A, const double &B,
                                         const double &C, double &lower,
                                         double &upper) {
    // Assumed A>=0
    // solve A * X^2 + 2 * B * X + C <=0
    if (A < EPS) {
      if (B < -EPS) {
        lower = C / B / 2.;
        upper = INF;
      } else if (B > EPS) {
        lower = -INF;
        upper = C / B / 2.;
      } else if (C <= 0) {
        lower = -INF;
        upper = INF;
      } else {
        lower = INF;
        upper = -INF;
      }
    } else {
      double D = B * B - A * C;
      if (D < 0.) {
        lower = INF;
        upper = -INF;
      } else {
        lower = (-B - sqrt(D)) / A;
        upper = (-B + sqrt(D)) / A;
      }
    }
  }
  static void solve_vector_inequation(const double &ax, const double &ay,
                                      const double &bx, const double &by,
                                      const double &r, double &lower,
                                      double &upper) {
    // solve abs(a*t + b) <= r
    solve_quadratic_inequation(ax * ax + ay * ay, ax * bx + ay * by,
                               bx * bx + by * by - r * r, lower, upper);
  }

 public:
  static double distance(const Point &p0, const Point &p1) {
    return sqrt(pow(p0.x - p1.x, 2) + pow(p0.y - p1.y, 2));
  }

  static bool CollisionCheck(const Point &p0, const Point &p1,
                             const AgentGeometry &agent_geometry,
                             const double &EPS) {
    return distance(p0, p1) < 2 * agent_geometry - EPS;
  }

  static bool CollisionCheck(const Point &sp0, const double &st0,
                             const Point &tp0, const double &tt0,
                             const Point &sp1, const double &st1,
                             const Point &tp1, const double &tt1,
                             const AgentGeometry &r, const double &EPS) {
    // One agent leaves sp0 at time st0 and reaches sp1 at time tt0
    // Another agent leaves sp0 at time st0 and reaches sp1 at time tt0
    // If distance between two agents is lower than r0+r1 at some time, return
    // true

    if (tt0 - EPS < st1 || tt1 - EPS < st0) {
      return false;
    }
    double dt0 = tt0 - st0, dt1 = tt1 - st1;

    // p0[t] = (tp0 - sp0) * (t - st0) / (tt0 - st0) + sp0
    //       = ((tp0 - sp0) * t + (-tp0 * st0 + sp0 * tt0)) / (tt0 - st0)
    // (p0[t]-p1[t])*(tt0 - st0)*(tp0 - sp0)
    // = ((tp0-sp0) * dt1 - (tp1-sp1) * dt0) * t + ...
    // = a * t + b
    double ax = (tp0.x - sp0.x) / dt0 - (tp1.x - sp1.x) / dt1;
    double bx =
        (sp0.x * tt0 - tp0.x * st0) / dt0 - (sp1.x * tt1 - tp1.x * st1) / dt1;
    double ay = (tp0.y - sp0.y) / dt0 - (tp1.y - sp1.y) / dt1;
    double by =
        (sp0.y * tt0 - tp0.y * st0) / dt0 - (sp1.y * tt1 - tp1.y * st1) / dt1;
    // d * dt0^2 * dt1^2 = a * a * t * t + 2 * a * b * t + b * b
    double A = ax * ax + ay * ay, B = ax * bx + ay * by,
           C = bx * bx + by * by - 4 * r * r;
    // solve A * t^2 +  2 * B * t + C = A * (t + B/A)^2 - (B*B - A*C)/A <=0
    double st = std::max(st0, st1), tt = std::min(tt0, tt1);
    bool collide;
    if (A * st * st + 2 * B * st + C < -EPS) {
      collide = true;
    } else if (A * tt * tt + 2 * B * tt + C < -EPS) {
      collide = true;
    } else if (A > EPS && A * st + EPS < -B && -B < A * tt - EPS &&
               B * B - A * C > EPS) {
      collide = true;
    } else {
      collide = false;
    }
    return collide;
  }

  static bool getCollisionInterval(const Point &p0, const Point &sp1,
                                   const Point &tp1, const double &dt1,
                                   const AgentGeometry &r,
                                   double &collision_start,
                                   double &collision_end) {
    double ax = -(tp1.x - sp1.x) / dt1;
    double ay = -(tp1.y - sp1.y) / dt1;
    double bx = p0.x - sp1.x;
    double by = p0.y - sp1.y;
    double lower, upper;
    solve_vector_inequation(ax, ay, bx, by, 2 * r, lower, upper);
    collision_start = std::max(0., lower);
    collision_end = std::min(dt1, upper);
    return collision_start < collision_end;
  }

  static bool CollisionCheck(const Point &p0, const double &st0,
                             const double &tt0, const Point &sp1,
                             const double &st1, const Point &tp1,
                             const double &tt1,
                             const AgentGeometry &agent_geometry,
                             const double &EPS) {
    double lower, upper;
    if (!getCollisionInterval(p0, sp1, tp1, tt1 - st1, agent_geometry, lower,
                              upper)) {
      return false;
    }
    return std::max(st0, st1 + lower) + EPS < std::min(tt0, st1 + upper);
  }

  static bool getCollisionInterval(const Point &sp0, const Point &tp0,
                                   const double &dt0, const Point &sp1,
                                   const Point &tp1, const double &dt1,
                                   const AgentGeometry &r,
                                   double &collision_start,
                                   double &collision_end) {
    // p0[t] = (tp0 - sp0) * (t - st0) / (tt0 - st0) + sp0
    //       = ((tp0 - sp0) * t + (-tp0 * st0 + sp0 * tt0)) / (tt0 - st0)
    // (p0[t]-p1[t])*(tt0 - st0)*(tp0 - sp0)
    // = ((tp0-sp0) * dt1 - (tp1-sp1) * dt0) * t + ...
    // = a * t + b
    double cx = -(tp0.x - sp0.x) / dt0;
    double cy = -(tp0.y - sp0.y) / dt0;
    double ax = -cx - (tp1.x - sp1.x) / dt1;
    double ay = -cy - (tp1.y - sp1.y) / dt1;
    double bx = sp0.x - sp1.x;
    double by = sp0.y - sp1.y;

    // find the range of s such that
    // |a*t + b + c*s|<=2r have solution in 0<=t<=dt0 and s<=t<=s+dt1

    double lower, upper;
    collision_start = INF;
    collision_end = -INF;

    // solution t=0 case
    solve_vector_inequation(cx, cy, bx, by, 2 * r, lower, upper);
    // s <= 0 <= s + dt0
    lower = std::max(lower, -dt0);
    upper = std::min(upper, 0.0);
    if (lower < upper) {
      collision_start = std::min(collision_start, lower);
      collision_end = std::max(collision_end, upper);
    }

    // solution t=s case
    solve_vector_inequation(ax + cx, ay + cy, bx, by, 2 * r, lower, upper);
    // 0.0 <= s <= dt1
    lower = std::max(lower, 0.0);
    upper = std::min(upper, dt1);
    if (lower < upper) {
      collision_start = std::min(collision_start, lower);
      collision_end = std::max(collision_end, upper);
    }

    // solution t=dt1 case
    solve_vector_inequation(cx, cy, ax * dt1 + bx, ay * dt1 + by, 2 * r, lower,
                            upper);
    // s <= dt1 <= s + dt0
    lower = std::max(lower, dt1 - dt0);
    upper = std::min(upper, dt1);
    if (lower < upper) {
      collision_start = std::min(collision_start, lower);
      collision_end = std::max(collision_end, upper);
    }

    // solution t=s+dt0 case
    solve_vector_inequation(ax + cx, ay + cy, ax * dt0 + bx, ay * dt0 + by,
                            2 * r, lower, upper);
    // 0 <= s+dt0 <= dt1
    lower = std::max(lower, -dt0);
    upper = std::min(upper, dt1 - dt0);
    if (lower < upper) {
      collision_start = std::min(collision_start, lower);
      collision_end = std::max(collision_end, upper);
    }

    if (ax * ax + ay * ay > EPS) {
      // solution t=a*(b+cs)/|a|^2 case
      double aa = ax * ax + ay * ay;
      double ab = ax * bx + ay * by;
      double ac = ax * cx + ay * cy;
      double bb = bx * bx + by * by;
      double cc = cx * cx + cy * cy;
      double cb = cx * bx + cy * by;
      solve_quadratic_inequation(aa * cc - ac * ac, aa * cb - ac * ab,
                                 aa * (bb - 4 * r * r) - ab * ab, lower, upper);
      // 0 <= -(ac * s + ab) <= aa *dt1
      solve_linear_inequation(ac, ab, lower, upper);
      solve_linear_inequation(-ac, -ab - aa * dt1, lower, upper);
      // aa* s <= -(ac * s + ab) <= aa * s + aa * dt0
      solve_linear_inequation(aa + ac, ab, lower, upper);
      solve_linear_inequation(-ac - aa, -ab - aa * dt0, lower, upper);
      if (lower < upper) {
        collision_start = std::min(collision_start, lower);
        collision_end = std::max(collision_end, upper);
      }
    }
    return collision_start < collision_end;
  }

  static Point scanPoint(FILE *input_file) {
    double x, y;
    fscanf(input_file, "%lf%lf", &x, &y);
    return Point(x, y);
  }
  static void printPoint(FILE *output_file, const Point point) {
    fprintf(output_file, "%lf %lf\n", point.x, point.y);
  }

  static double inner(const Geometry2D::Point &point_0,
                      const Geometry2D::Point &point_1) {
    return point_0.x * point_1.x + point_0.y * point_1.y;
  }
  static double length(const Geometry2D::Point &point) {
    return std::sqrt(inner(point, point));
  }
};

static Geometry2D::Point operator+(const Geometry2D::Point &point_0,
                                   const Geometry2D::Point &point_1) {
  return Geometry2D::Point(point_0.x + point_1.x, point_0.y + point_1.y);
}
static Geometry2D::Point operator-(const Geometry2D::Point &point_0,
                                   const Geometry2D::Point &point_1) {
  return Geometry2D::Point(point_0.x - point_1.x, point_0.y - point_1.y);
}
static Geometry2D::Point operator*(const double scalar,
                                   const Geometry2D::Point &point) {
  return Geometry2D::Point(scalar * point.x, scalar * point.y);
}
#endif

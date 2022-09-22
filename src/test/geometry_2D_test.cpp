/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#include "base/geometry_2D.hpp"

void test_from_stdin() {
  using Point = Geometry2D::Point;
  Point sp0 = Geometry2D::scanPoint(stdin);
  Point tp0 = Geometry2D::scanPoint(stdin);
  double st0, tt0;
  scanf("%lf%lf", &st0, &tt0);
  Point sp1 = Geometry2D::scanPoint(stdin);
  Point tp1 = Geometry2D::scanPoint(stdin);
  double st1, tt1, r;
  scanf("%lf%lf%lf", &st1, &tt1, &r);
  Geometry2D::CollisionCheck(sp0, st0, tp0, tt0, sp1, st1, tp1, tt1, r, 0.0);
}

void CI_test_from_stdin() {
  using Point = Geometry2D::Point;
  Point sp0 = Geometry2D::scanPoint(stdin);
  Point tp0 = Geometry2D::scanPoint(stdin);
  Point sp1 = Geometry2D::scanPoint(stdin);
  Point tp1 = Geometry2D::scanPoint(stdin);
  double dt0, dt1, r;
  scanf("%lf%lf%lf", &dt0, &dt1, &r);
  double start, end;
  bool collision = Geometry2D::getCollisionInterval(sp0, tp0, dt0, sp1, tp1,
                                                    dt1, r, start, end);
  printf("%d %lf %lf\n", collision, start, end);
}

int main() {
  CI_test_from_stdin();
  std::random_device random_device;
  std::default_random_engine engine(random_device());
  std::uniform_real_distribution<double> distribution(0.0, 1.0);

  using Point = Geometry2D::Point;

  for (int t = 0; t < 10000; t++) {
    Point p0 = Point(distribution(engine), distribution(engine));
    Point sp1 = Point(distribution(engine), distribution(engine));
    Point tp1 = Point(distribution(engine), distribution(engine));
    double dt1 = distribution(engine);
    double r = 0.3 * distribution(engine);
    double lower, upper;
    bool collide =
        Geometry2D::getCollisionInterval(p0, sp1, tp1, dt1, r, lower, upper);
    int N = 10000;
    for (int g = 1; g < N; g++) {
      double s = dt1 / N * g;
      bool expected;
      if (collide) {
        expected = (lower < s && s < upper);
      } else {
        expected = false;
      }
      Point p1 = Point(sp1.x + (tp1.x - sp1.x) / dt1 * s,
                       sp1.y + (tp1.y - sp1.y) / dt1 * s);
      if (expected != (Geometry2D::distance(p0, p1) < 2 * r)) {
        printf("%d\n", (int)expected);
        printf("%lf %lf\n", p0.x, p0.y);
        printf("%lf %lf %lf %lf %lf %lf %lf\n", sp1.x, sp1.y, tp1.x, tp1.y, 0.0,
               dt1, r);
        printf("%lf %lf %lf\n", lower, upper, s);
        return 1;
      }
    }
  }

  for (int t = 0; t < 10000; t++) {
    Point sp0 = Point(distribution(engine), distribution(engine));
    Point tp0 = Point(distribution(engine), distribution(engine));
    double st0 = distribution(engine), tt0 = distribution(engine);
    if (st0 > tt0) std::swap(st0, tt0);
    Point sp1 = Point(distribution(engine), distribution(engine));
    Point tp1 = Point(distribution(engine), distribution(engine));
    double st1 = distribution(engine), tt1 = distribution(engine);
    if (st1 > tt1) std::swap(st1, tt1);
    double r = 0.3 * distribution(engine);
    bool result = Geometry2D::CollisionCheck(sp0, st0, tp0, tt0, sp1, st1, tp1,
                                             tt1, r, 0.0);
    bool experiment_result = false;
    double st = std::max(st0, st1), tt = std::min(tt0, tt1);
    double ct;
    Point cp0, cp1;
    if (st < tt) {
      int N = 10000;
      for (int g = 0; g <= N; g++) {
        double t = st + (tt - st) / N * g;
        Point p0 = Point(sp0.x + (tp0.x - sp0.x) / (tt0 - st0) * (t - st0),
                         sp0.y + (tp0.y - sp0.y) / (tt0 - st0) * (t - st0));
        Point p1 = Point(sp1.x + (tp1.x - sp1.x) / (tt1 - st1) * (t - st1),
                         sp1.y + (tp1.y - sp1.y) / (tt1 - st1) * (t - st1));
        if (Geometry2D::distance(p0, p1) < 2 * r) {
          experiment_result = true;
          ct = t;
          cp0 = p0, cp1 = p1;
        }
      }
    }
    if (result != experiment_result) {
      printf("%d %d\n", (int)result, (int)experiment_result);
      printf("%lf %lf %lf %lf %lf %lf\n", sp0.x, sp0.y, tp0.x, tp0.y, st0, tt0);
      printf("%lf %lf %lf %lf %lf %lf\n", sp1.x, sp1.y, tp1.x, tp1.y, st1, tt1);
      if (experiment_result) {
        printf("%lf %lf %lf %lf %lf\n", cp0.x, cp0.y, cp1.x, cp1.y, ct);
      }
      return 1;
    }
  }

  for (int t = 0; t < 10000; t++) {
    Point sp0 = Point(distribution(engine), distribution(engine));
    Point tp0 = Point(distribution(engine), distribution(engine));
    double dt0 = distribution(engine);
    Point sp1 = Point(distribution(engine), distribution(engine));
    Point tp1 = Point(distribution(engine), distribution(engine));
    double dt1 = distribution(engine);
    double r = 0.3 * distribution(engine);
    double lower, upper;
    bool collide = Geometry2D::getCollisionInterval(sp0, tp0, dt0, sp1, tp1,
                                                    dt1, r, lower, upper);
    int N = 10000;
    for (int g = -N; g <= 2 * N; g++) {
      double s;
      bool expected;
      if (collide) {
        s = lower + (upper - lower) / N * g;
        expected = (0 < g && g < N);
      } else {
        s = -dt0 + (dt0 + dt1) / N * g;
        expected = false;
      }
      bool result = Geometry2D::CollisionCheck(
          sp0, s, tp0, s + dt0, sp1, 0.0, tp1, dt1, r, expected ? 0.0 : 1e-9);
      if (result != expected) {
        printf("%d %d\n", (int)result, (int)expected);
        printf("%lf %lf %lf %lf %lf %lf\n", sp0.x, sp0.y, tp0.x, tp0.y, s,
               s + dt0);
        printf("%lf %lf %lf %lf %lf %lf %lf\n", sp1.x, sp1.y, tp1.x, tp1.y, 0.0,
               dt1, r);
        printf("%lf %lf %lf\n", lower, upper, s);
        return 1;
      }
    }
  }
  return 0;
}

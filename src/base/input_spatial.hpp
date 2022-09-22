/*
Copyright (c) 2022 OMRON SINIC X Corporation
Author: Kazumi Kasaura
*/

#ifndef MAPF_BENCHMARK_INPUT_SPATIAL
#define MAPF_BENCHMARK_INPUT_SPATIAL

#include <cassert>

#include "base/geometry_2D_for_generator.hpp"
#include "base/spatial.hpp"

template <>
inline void SpatialProblemInstance<Space2D>::loadTextFile(
    const std::string &file_path) {
  FILE *input_file =
      file_path == "stdin" ? stdin : fopen(file_path.c_str(), "r");
  if (input_file == NULL) {
    fprintf(stderr, "File cannot be opened\n");
    return;
  }
  int number_of_holes, number_of_agents;
  fscanf(input_file, "%d%d", &number_of_holes, &number_of_agents);
  int number_of_vertices;
  fscanf(input_file, "%d", &number_of_vertices);
  space.outer().resize(number_of_vertices + 1);
  for (int i = 0; i < number_of_vertices; i++) {
    space.outer()[i] = Geometry2D::scanPoint(input_file);
  }
  space.outer()[number_of_vertices] = space.outer()[0];
  space.inners().resize(number_of_holes);
  for (int j = 0; j < number_of_holes; j++) {
    fscanf(input_file, "%d", &number_of_vertices);
    space.inners()[j].resize(number_of_vertices + 1);
    for (int i = 0; i < number_of_vertices; i++) {
      space.inners()[j][i] = Geometry2D::scanPoint(input_file);
    }
    space.inners()[j][number_of_vertices] = space.inners()[j][0];
  }
  fscanf(input_file, "%lf", &agent_geometry);
  tasks.resize(number_of_agents);
  for (int k = 0; k < number_of_agents; k++) {
    tasks[k].initial_point = Geometry2D::scanPoint(input_file);
    tasks[k].goal_point = Geometry2D::scanPoint(input_file);
  }
  if (input_file != stdin) {
    fclose(input_file);
  }
};

std::vector<std::vector<bool>> loadMAPFBGrid(const std::string &file_path) {
  FILE *in = fopen(file_path.c_str(), "r");
  if (in == NULL) {
    throw std::runtime_error("Map file cannot be opened\n");
  }
  int height, width;
  fscanf(in, "type octile\nheight %d\nwidth %d\nmap\n", &height, &width);
  std::vector<std::vector<bool>> row_grid(
      height + 2,
      std::vector<bool>(width + 2,
                        false));  // fill boundaries by unpassable blocks
  for (int i = 0; i < height; i++) {
    char c;
    for (int j = 0; j < width; j++) {
      c = getc(in);
      row_grid[i + 1][j + 1] = c == '.' || c == 'G' || c == 'S';
    }
    while ((c = getc(in)) == '\n' || c == '\r')
      ;
    ungetc(c, in);
  }
  fclose(in);

  std::vector<std::vector<bool>> grid(
      height + 2,
      std::vector<bool>(width + 2,
                        false));  // consider only one connected component
  int start_x, start_y;
  for (start_y = 1; start_y <= height; start_y++) {
    for (start_x = 1; start_x <= width; start_x++) {
      if (row_grid[start_y][start_x]) break;
    }
    if (start_x <= width) break;
  }
  assert(start_y <= height);  // at least one open cel
  const int dx[4] = {1, 0, -1, 0},
            dy[4] = {0, 1, 0, -1};  // left:0 bottom:1 right:2 top:3
  std::queue<std::pair<int, int>> open_cells;
  open_cells.push(std::make_pair(start_y, start_x));
  grid[start_y][start_x] = true;
  while (!open_cells.empty()) {
    int y = open_cells.front().first, x = open_cells.front().second;
    open_cells.pop();
    for (int v = 0; v < 4; v++) {
      int new_y = y + dy[v], new_x = x + dx[v];
      if (row_grid[new_y][new_x] && !grid[new_y][new_x]) {
        grid[new_y][new_x] = true;
        open_cells.push(std::make_pair(new_y, new_x));
      }
    }
  }
  return grid;
}

template <>
inline void SpatialProblemInstance<Space2D>::loadGripMap(
    const std::string &file_path) {
  auto grid = std::move(loadMAPFBGrid(file_path));
  int height = grid.size() - 2, width = grid[0].size() - 2;

  std::vector<std::vector<bool>> visited(height + 2,
                                         std::vector<bool>(width + 2, false));
  bool found_outer = false;
  const int dx[4] = {1, 0, -1, 0},
            dy[4] = {0, 1, 0, -1};  // left:0 bottom:1 right:2 top:3
  const int sx[4] = {0, -1, -1, 0}, sy[4] = {0, 0, -1, -1};
  for (int start_y = 1; start_y <= height; start_y++) {
    for (int start_x = 1; start_x <= width; start_x++) {
      if (!(!grid[start_y][start_x - 1] && grid[start_y][start_x]) ||
          visited[start_y][start_x]) {
        continue;
      }
      // right hand method
      std::vector<int> xs, ys;
      int x = start_x, y = start_y;
      int direction = 1;
      do {
        if (direction == 1) visited[y][x] = true;
        x += dx[direction];
        y += dy[direction];
        int ldir = (direction + 3) % 4, rdir = (direction + 1) % 4;
        if (!grid[y + sy[ldir]][x + sx[ldir]]) {
          xs.push_back(x);
          ys.push_back(y);
          direction = ldir;
        } else if (grid[y + sy[direction]][x + sx[direction]]) {
          xs.push_back(x);
          ys.push_back(y);
          direction = rdir;
        }
      } while (x != start_x || y != start_y);
      if (!found_outer) {
        found_outer = true;
        space.outer().resize(xs.size() + 1);
        for (int i = 0; i < xs.size(); i++) {
          space.outer()[i] = Geometry2D::Point(xs[i] - 1, ys[i] - 1);
        }
        space.outer()[xs.size()] = space.outer()[0];
      } else {
        Space2D::Space::ring_type obstacle;
        obstacle.resize(xs.size() + 1);
        for (int i = 0; i < xs.size(); i++) {
          obstacle[i] = Geometry2D::Point(xs[i] - 1., ys[i] - 1.);
        }
        obstacle[xs.size()] = obstacle[0];
        space.inners().push_back(obstacle);
      }
    }
  }
}

std::vector<std::tuple<int, int, int, int>> loadMAPFBScenario(
    const std::string &file_path) {
  FILE *in = fopen(file_path.c_str(), "r");
  if (in == NULL) {
    throw std::runtime_error("Scenario file cannot be opened\n");
  }
  double version;
  fscanf(in, "version %lf", &version);
  int bucket, width, height, start_x, start_y, goal_x, goal_y;
  char map_name[1000];
  double length;
  std::vector<std::tuple<int, int, int, int>> tasks;
  while (fscanf(in, "%d%999s%d%d%d%d%d%d%lf", &bucket, map_name, &width,
                &height, &start_x, &start_y, &goal_x, &goal_y,
                &length) != EOF) {
    tasks.push_back(std::make_tuple(start_x, start_y, goal_x, goal_y));
  }
  fclose(in);
  return tasks;
}

template <>
inline void SpatialProblemInstance<Space2D>::loadScenario(
    const std::string &file_path) {
  auto task_tuples = std::move(loadMAPFBScenario(file_path));
  for (auto &task : task_tuples) {
    auto initial =
        Geometry2D::Point(std::get<0>(task) + 0.5, std::get<1>(task) + 0.5);
    auto goal =
        Geometry2D::Point(std::get<2>(task) + 0.5, std::get<3>(task) + 0.5);
    if (isInside(initial) && isInside(goal)) {
      tasks.push_back(Task(initial, goal));
    }
  }
}
#endif

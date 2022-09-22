# Prioritized Safe Interval Path Planning with Continuous-Time Conflict Annotation

This is a repository for the following paper:
- Kazumi Kasaura, Mai Nishimura, and Ryo Yonetani. “Prioritized Safe Interval Path Planning for Multi-Agent Pathfinding With Continuous Time on 2D Roadmaps.” IEEE Robotics and Automation Letters 7.4 (2022): 10494–10501.

This repository contains [Continuous-CBS](https://github.com/thaynewalker/CCBS) as a submodule.

## Getting Started

You can use [Dockerfile](Dockerfile) to build an environment for this repository.

This project uses C++14 standard library. Make sure your compiler support it.
Some tools, such as visualizer, uses Python3 with numpy and matplotlib.

### Prerequisities
- [CMake](https://cmake.org/) &mdash; an open-source, cross-platform family of tools designed to build, test and package software.
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) &mdash; a YAML parser and emitter in C++ matching the YAML 1.2 spec.
- [CGAL](https://www.cgal.org/) &mdash; a software project that provides easy access to efficient and reliable geometric algorithms in the form of a C++ library.

### Installing

Download this repository:
```
git clone https://github.com/omron-sinicx/PSIPP-CTC.git
```
At the downloaded repository, build by make command:
```
mkdir build
cd build
cmake ..
make
```

## Usage

The samples of problems with continuous spaces and problems with roadmaps are contained in `problem_instances/spatial` or `problem_instances/roadmaps`.

### Generate Roadmap

`roadmap_generation` inputs a problem with continuous space and generate a problem with roadmap.

In default setting, it reads from the standard input and writes to the standard output.
For exmaple,
```
./build/roadmap_generation < spatial_problem.txt > roadmap_problem.txt
```

The configuration is stored in `config/roadmap_generation.yaml`.
You can also use your configuration file as follows:
```
./build/roadmap_generation your_generation_config.yaml
```

Roadmaps can be visualized by `tools/plot_roadmap.py` script:
```
./tools/plot_roadmap.txt spatial_problem.txt roadmap_problem.txt
```

### Planning

`planner_benchmark` inputs problems with roadmaps and generate plans for them by planning algorithms.
Runtime for planning is displayed in standard error.

In default setting, it reads one problem from the standard input and writes to the standard output.
For example,
```
./build/planner_benchmark < roadmap_problem.txt > plan.txt
```

The configuration is stored in `config/planner_benchmark.yaml`.
You can also use your configuration file as follows:
```
./build/planner_benchmark your_planning_config.yaml
```

By modifying configuration file, this command can solve several problems by several planners at once.

Plans can be visualized by `tools/visualize_plan.py` script:
<pre><code>./tools/visualize_plan.py spatial_problem.txt plan.txt <i>time_step</i>
</code></pre>
where *time_step* is a value which means the time scale corresponding one frame in the animation.

## Config Parameters

### Roadmap Generation
- `Generator`
  - `Type`: `kPRM` (Probabilistic Roadmap with Nearest neighborhood) or `CDT` (Constrained Delaunay Triangulation).
  - `number_of_vertices`: For kPRM, the number of sampled points.
  - `number_of_neighbors`: For kPRM, the number of points connected with each point.
  - `number_of_additionals`: For CDT, the number of points sampled additionally.
- `Input`
  - `Type`: `Text`
  - `file`: `stdin` for starndard input, or path to input file.
- `Output`: `stdout` for standard output, or path to output file.
### Planning
- `Problems`: Several configurations for problems can be listed here.
  - *ProblemName*: Any string which identify the problem.
    - `Type`: `Geometric` for collision check in planning time, or `Explicit` for annotating conflicts beforehand.
    - `file`: stdin` for starndard input, or path to input file.
- `Problems`: Several configurations for planners can be listed here.
  - *PlannerName*: Any string which identify the planner.
    - `Type`: `PSIPP` for Prioritized Safe Interval Path Planning, or `CCBS` for Continuous Conflict-Based Search.
    - `time_limit`: time limit for planning. The default value is 30 seconds.
- `Output`: `stdout` for standard output, or path to output file.
## File Formats
### Problem with Continuous Space
> *number_of_obstacles* *number_of_agents* \
*description_of_polygon_for_outline*\
*description_of_polygon_for_obstacle_1*\
*description_of_polygon_for_obstacle_2*\
&#xFE19;\
*radius_of_agents*\
*agent_1_start_x* *agent_1_start_y* *agent_1_goal_x* *agent_1_goal_y*\
*agent_2_start_x* *agent_2_start_y* *agent_2_goal_x* *agent_2_goal_y*\
&#xFE19;

Polygons are described as follows:
> *number_of_vertices*\
 *vertex_1_x* *vertex_1_y*\
 *vertex_2_x* *vertex_2_y*\
&#xFE19;
### Problem with Roadmap
> *number_of_vertices* *number_of_edges* *number_of_agents*\
*vertex_1_x* *vertex_1_y*\
*vertex_2_x* *vertex_2_y*\
&#xFE19;\
*edge_1_source_vertex_id* *edge_1_target_vertex_id*\
*edge_2_source_vertex_id* *edge_2_target_vertex_id*\
&#xFE19;\
*agent_1_start_vertex_id* *agent_1_goal_vertex_id*\
*agent_2_start_vertex_id* *agent_2_goal_vertex_id*\
&#xFE19;\
*radius_of_agents*
### Plan
> *number_of_agents* *radius_of_agents*\
*description_of_path_for_agent_1*\
*description_of_path_for_agent_2*\
&#xFE19;

Paths are described as follows:
> *number_of_points*\
*time_1*\
 *x_1* *y_1*\
*time_2*\
 *x_2* *y_2*\
&#xFE19;

## License
This software is released under the MIT License, see [LICENSE](LICENSE).
## Citation
```
@article{kasaura2022prioritized,
  title={Prioritized Safe Interval Path Planning for Multi-Agent Pathfinding With Continuous Time on 2D Roadmaps},
  author={Kasaura, Kazumi and Nishimura, Mai and Yonetani, Ryo},
  journal={IEEE Robotics and Automation Letters},
  volume={7},
  number={4},
  pages={10494--10501},
  year={2022},
  publisher={IEEE}
}
```

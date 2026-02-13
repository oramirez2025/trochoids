![workflow status badge](https://github.com/castacks/trochoids/actions/workflows/ci-tests.yml/badge.svg)
# Time-Optimal Path Planning in a Constant Wind for Uncrewed Aerial Vehicles using Dubins Set Classification 

This repository contains code for the paper
**<a href="https://arxiv.org/abs/2306.11845">"Time-Optimal Path Planning in a Constant Wind for Uncrewed Aerial Vehicles using Dubins Set Classification"</a>**  by *<a href="https://bradymoon.com">Brady Moon\*</a>, <a href="https://sagars2.com">Sagar Sachdev\*</a>, <a href="https://theairlab.org/team/junbiny/">Junbin Yuan</a>, and <a href="https://www.ri.cmu.edu/ri-faculty/sebastian-scherer/">Sebastian Scherer</a> (\* equal contribution)*.

This codebase includes both a solver for trochoidal paths when there is wind as well as also solving Dubins paths when there is no wind. The Dubins path solutions use the work <a href="http://dx.doi.org/10.1016/S0921-8890(00)00127-5">"Classification of the Dubins set"</a> as well as the correction proposed in the work  <a href="https://www.research-collection.ethz.ch/handle/20.500.11850/615185">"Circling Back: Dubins set Classification Revisited."</a>

<p align="center">   
    <img src="img/Fig1v7-2.png" alt="drawing" style="width:50%;"/>
</p>

## Brief Overview
Time-optimal path planning in high winds for a turning-rate constrained UAV is a challenging problem to solve and is important for deployment and field operations. Previous works have used trochoidal path segments comprising straight and maximum-rate turn segments, as optimal extremal paths in uniform wind conditions. Current methods iterate over all candidate trochoidal trajectory types and select the one that is time-optimal; however, this exhaustive search can be computationally slow. In this paper, we introduce a method to decrease the computation time. This is achieved by reducing the number of candidate trochoidal trajectory types by framing the problem in the air-relative frame and bounding the solution within a subset of candidate trajectories. Our method reduces overall computation by 37.4% compared to pre-existing methods in Bang-Straight-Bang trajectories, freeing up computation for other onboard processes and can lead to significant total computational reductions when solving many trochoidal paths. When used within the framework of a global path planner, faster state expansions help find solutions faster or compute higher-quality paths. We also release our open-source codebase as a C++ package.


## Package Layout

This repository now supports four install/use paths:

* Core C++ library (pure CMake): repo root.
* Python package (pip): built from the same core using `pybind11` and `scikit-build-core`.
* ROS1 package (catkin, local compatibility): `ros1/trochoids_ros1`.
* ROS2 package (ament, release target): `ros2/trochoids_ros2`.

## Prerequisites

* CMake 3.16+
* C++17 compiler
* Eigen3 (`sudo apt-get install libeigen3-dev`)
* Python 3.8+ (for pip package and visualization)
* Optional benchmark dep: `sudo apt-get install libbenchmark-dev`

## Core C++ Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Install the C++ library:

```bash
cmake --install build --prefix /usr/local
```

Run tests (if GTest is installed):

```bash
cd build
ctest --output-on-failure
```

## Python (pip) Build

Build/install from source:

```bash
python3 -m pip install .
```

Editable install for development:

```bash
python3 -m pip install -e .
```

If your environment has an older pip/PEP660 setup (for example some base Docker images), use:

```bash
python3 -m pip install .
```

Python API exposes:
* `XYZPsiState`
* `VerticalConstraints`
* `VerticalPlanInfo`
* `VerticalPlanningCase`
* `get_trochoid_path(...)`
* `get_trochoid_path_numerical(...)`
* `get_trochoid_path_3d(...)`

## ROS1 (catkin)

ROS1 package is in `ros1/trochoids_ros1`.

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone <repo-url>
cd ..
catkin build trochoids_ros1
```

## ROS2 (ament/colcon)

ROS2 package is in `ros2/trochoids_ros2`.

Primary recommendation: rely on CI for ROS2 build validation and packaging checks.  
Local ROS2 builds are optional and mainly useful for debugging.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <repo-url>
cd ..
colcon build --packages-select trochoids_ros2
```

## ROS apt Distribution Notes

Official ROS apt release target for this repo is ROS2 only:

* Release `ros2/trochoids_ros2` into ROS2 rosdistro tracks (for example `humble`).
* Keep `ros1/trochoids_ros1` for local compatibility/development builds only.

This repository is structured so ROS2 release workflows are independent from the core C++/pip workflows.
The `build-all-targets` GitHub Action already validates ROS1 and ROS2 package builds on every PR/push.
See `RELEASING_ROS.md` for the ROS2 release checklist.

### Visualizing 3D test paths

The dedicated 3D test target writes CSV path outputs (`x,y,z,psi`) that you can visualize:

```bash
# choose CSV output directory
# native/host:
# export TROCHOIDS_3D_CSV_DIR=csv_files/3d
#
# docker:
# export TROCHOIDS_3D_CSV_DIR=/ws/src/trochoids/csv_files/3d
export TROCHOIDS_3D_CSV_DIR=/ws/src/trochoids/csv_files/3d

# run only the 3D tests
./build/trochoids-3d-test

# render XY and 3D figures from all generated CSV files
python3 /ws/src/trochoids/test/visualize_trochoids_3d.py --csv-dir /ws/src/trochoids/csv_files/3d
```

Figures are saved to `figures/3d` by default.
If you use `docker compose run --rm`, only mounted paths persist after the container exits.

## Usage

The main function for the trochoid solver is `get_trochoid_path()`. This function takes in the following parameters:

* Start State: [x, y, z, psi]
* Goal State: [x, y, z, psi]
* Wind: [x, y, z]
* Desired Speed (m/s)
* Max Kappa: 1/turning_radius (1/m)
* (Optional) Waypoint Distance: The distance between waypoints in the trochoid path (m)


If there is no wind, it solves for the path using a Dubins path, and if there is wind it uses a trochoidal path. 

### Simple Example
```cpp
double wind[3] = {0.3, 0.5, 0};
double desired_speed = 15;
double max_kappa = .02;
double waypoint_distance = 10;

trochoids::XYZPsiState start_state = {0, 0, 110, 0};
trochoids::XYZPsiState goal_state = {500, 0, 110, 0};

std::vector<trochoids::XYZPsiState> trochoid_path;
bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa, waypoint_distance);
```

### Simple Python Example
```python
import trochoids

start = trochoids.XYZPsiState()
start.x, start.y, start.z, start.psi = 0.0, 0.0, 110.0, 0.0

goal = trochoids.XYZPsiState()
goal.x, goal.y, goal.z, goal.psi = 500.0, 0.0, 110.0, 0.0

wind = [0.3, 0.5, 0.0]
desired_speed = 15.0
max_kappa = 0.02
waypoint_distance = 10.0

valid, path = trochoids.get_trochoid_path(
    start, goal, wind, desired_speed, max_kappa, waypoint_distance
)

print("valid:", valid)
print("num waypoints:", len(path))
if valid and path:
    print("first waypoint:", path[0].x, path[0].y, path[0].z, path[0].psi)
    print("last waypoint:", path[-1].x, path[-1].y, path[-1].z, path[-1].psi)
```



## Citation
If you find this work useful, please cite our paper:
```BibTeX
@article{moon2023timeoptimal,
    title={Time-Optimal Path Planning in a Constant Wind for Uncrewed Aerial Vehicles using Dubins Set Classification}, 
    author={Brady Moon and Sagar Sachdev and Junbin Yuan and Sebastian Scherer},
    year={2023},
    journal = {IEEE Robotics and Automation Letters},
    publisher = {IEEE},
    doi = {10.1109/LRA.2023.3333167},
    url = {https://arxiv.org/pdf/2306.11845.pdf},
    video = {https://youtu.be/qOU5gI7JshI}
}
```

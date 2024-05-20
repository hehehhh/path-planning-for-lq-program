# path-planning-for-lq-program
ros package for path planning and map_fusion which were used in lq program.
## 运行环境

<p align="center">
    <img width="100px" height="20px" src="https://img.shields.io/badge/Ubuntu-20.04-orange?logo=Ubuntu&Ubuntu-20.04"
        alt="ubuntu" />
    <img width="100px" height="20px" src="https://img.shields.io/badge/ROS-noetic-blue?logo=ROS&ROS=noetic" alt="ROS" />
</p>

## Quike Start

1.
```shell
git clone https://github.com/hehehhh/path-planning-for-lq-program.git
```

2.
```shell
cd path-path-planning-for-lq-program
catkin_make
souce ./devel/setup.bash
```

```shell
roslaunch lq_yg_map_server test_map_fusion_server.launch
```

## Document
#### LQ rosnode
```shell
source ./devel/setup.bash
roslaunch lq_yg_map_server map_fusion_server.launch 
```
<p align="center">
  <img src="asserts/8-1.png">
</p>

<p align="center">
  <img src="asserts/8-2.png">
</p>

<p align="center">
  <img src="asserts/8-3.png">
</p>

<p align="center">
  <img src="asserts/8-4.png">
</p>

<p align="center">
  <img src="asserts/8-5.png">
</p>

<p align="center">
  <img src="asserts/8-6.png">
</p>

#### a local planner using b-spline curve search
```shell
source ./devel/setup.bash
roslaunch lq_yg_map_server test_map_fusion_server.launch
rosrun path_search bspline_local_planner_node 
```
<p align="center">
  <img src="asserts/b-1.png">
  <img src="asserts/b-2.png">
  <img src="asserts/b-3.png">
  <img src="asserts/b-4.png">
  <img src="asserts/b-5.png">
</p>

#### Standard path with Dubins curve
```shell
rosrun standard_path standard_path_test
```
##### line path with Dubins curve
<p align="center">
  <img src="asserts/line_path.png">
</p>
##### circle path with Dubins curve
<p align="center">
  <img src="asserts/circle_path.png">
</p>
##### line path with Dubins curve
<p align="center">
  <img src="asserts/Octagonal_path.png">
</p>




# Dynamic Obstacle Tracker

Standalone ROS 2 dynamic-obstacle clustering, EKF tracking, and short-horizon trajectory prediction.

## Interfaces

The node subscribes to `dynamic_cloud` by default and publishes:

- `predicted_obstacles` (`dynamic_obstacle_tracker/msg/DynamicObstacleTrajectory`)
- `cluster_bounding_boxes` (`visualization_msgs/MarkerArray`)
- `tracked_obstacles` (`visualization_msgs/MarkerArray`)

## Launch

```bash
ros2 launch dynamic_obstacle_tracker dynamic_obstacle_tracker.launch.py
```

Override the regular YAML path with:

```bash
ros2 launch dynamic_obstacle_tracker dynamic_obstacle_tracker.launch.py \
  config_file:=/path/to/params.yaml
```

Override the input cloud topic with:

```bash
ros2 launch dynamic_obstacle_tracker dynamic_obstacle_tracker.launch.py \
  input_cloud_topic:=/my/dynamic_cloud
```

## Attribution and disclaimer

The tracker structure and implementation in
`include/dynamic_obstacle_tracker/tracker/obstacle_tracker.hpp` and `src/tracker/obstacle_tracker.cpp` are adapted from the obstacle tracker in the MIT-licensed `global_mapper_ros` package from the [acl-mapping](https://gitlab.com/mit-acl/lab/acl-mapping/-/tree/mighty) repository.

The upstream package identifies the following MIT notice:

```text
MIT License

Copyright (c) 2017 John Ware

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

The upstream implementation also records that it was ported from [`dynus/obstacle_tracker_node.hpp`](https://github.com/mit-acl/dynus/blob/main/include/dynus/obstacle_tracker_node.hpp).

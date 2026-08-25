# Dynamic Obstacle Tracker

Standalone ROS 2 dynamic-point detection, obstacle clustering, EKF tracking, and short-horizon trajectory prediction.

The package provides two independent nodes:

- `dynamic_point_detector_node`: deskewed point cloud to dynamic/static point clouds.
- `dynamic_obstacle_tracker_node`: dynamic point cloud to tracked and predicted obstacles.

The repository vendors [cilantro](https://github.com/kzampog/cilantro) as a Git submodule. After cloning without
`--recursive`, initialize it with:

```bash
git submodule update --init --recursive
```

## Interfaces

The detector subscribes to `deskewed_cloud` by default and publishes:

- `dynamic_points` (`sensor_msgs/PointCloud2`)
- `static_points` (`sensor_msgs/PointCloud2`)

Static points are only materialized and published when `static_points` has at
least one subscriber.

The tracker subscribes to `dynamic_cloud` and publishes:

- `obstacle_predicted_traj` (`dynamic_obstacle_tracker/msg/DynamicObstacleTrajectory`)
- `obstacle_bbox_marker` (`visualization_msgs/MarkerArray`)
- `obstacle_prediction_marker` (`visualization_msgs/MarkerArray`)

## Launch

Launch the complete scan-driven pipeline as two standalone processes with:

```bash
ros2 launch dynamic_obstacle_tracker dynamic_obstacle_pipeline.launch.py
```

Launch both nodes as components in one multithreaded container with intra-process communication enabled with:

```bash
ros2 launch dynamic_obstacle_tracker dynamic_obstacle_pipeline_composed.launch.py
```

The composed pipeline avoids DDS serialization for the detector-to-tracker `dynamic_cloud`. The multithreaded
container can run the detector and tracker callbacks concurrently, while each node's default mutually-exclusive
callback group keeps consecutive updates to that node serialized.

Launch only the detector with:

```bash
ros2 launch dynamic_obstacle_tracker dynamic_point_detector.launch.py
```

Launch only the tracker with:

```bash
ros2 launch dynamic_obstacle_tracker dynamic_obstacle_tracker.launch.py
```

Launch RViz2 with the packaged configuration:

```bash
ros2 launch dynamic_obstacle_tracker dynamic_obstacle_tracker.launch.py rviz:=true
```

Override the regular YAML path with:

```bash
ros2 launch dynamic_obstacle_tracker dynamic_obstacle_tracker.launch.py \
  config_file:=/path/to/params.yaml
```

Override the deskewed input cloud topic for the detector or complete pipeline with:

```bash
ros2 launch dynamic_obstacle_tracker dynamic_obstacle_pipeline_composed.launch.py \
  input_cloud_topic:=/my/deskewed_cloud
```

## Frame contract

`frame.tracking_frame` is the single coordinate frame used by the detector output, tracker state, velocities, and
predicted trajectories. It must be a continuous fixed frame such as `odom` or a continuous `world` frame, not
`base_link` or a sensor frame.

The expected detector input is already expressed in `tracking_frame`. If its `header.frame_id` differs, the detector
looks up `tracking_frame <- input_frame` at the point-cloud timestamp and transforms the points. A missing transform
drops the scan; the implementation does not fall back to the latest transform.

The detector looks up the configured `frame.sensor_frame`, (`os_sensor` for Ouster LiDARs for example) in `tracking_frame` at the cloud timestamp and uses its translation as the DDA ray origin. A complete timestamped TF tree is therefore required. A missing cloud-frame or sensor-frame transform drops the scan; neither lookup falls back to the latest transform.

The tracker does not transform or relabel its input. It rejects dynamic clouds whose frame differs from `tracking_frame` and advances the EKF using the cloud timestamp.

Set `detector.debug: true` to print per-scan timing breakdowns for ROS conversion/TF/output handling, detector
preprocessing/output assembly, and HMM-MOS ray casting, EDF, belief update, convolution, and segmentation.

## Dynamic-point detector

The detector stores HMM occupancy beliefs in sparse `8 x 8 x 8` voxel blocks addressed by integer block coordinates.
This follows the voxel-block spatial hashing organization and hash constants described by [Niessner et al.](https://niessnerlab.org/papers/2013/4hashing/niessner2013hashing.pdf), while replacing the paper's TSDF/color voxel payload with the three HMM states `unobserved`, `occupied`, and `free`.

Each scan is processed as one batch:

1. Filter invalid, range-excluded, and optionally ground points.
2. Aggregate endpoint hits and traverse all observed voxels with 3D DDA rays.
3. Construct a Gaussian Euclidean distance field from the current and previous occupied voxelized scans.
4. Update each observed voxel's three-state belief with the fixed HMM transition matrix and soft EDF likelihood.
5. Seed motion from confident occupied/free state changes.
6. Median-filter a spatial convolution score and accumulate it over the local scan window.
7. Apply Otsu thresholding, previous-scan persistence, and current-scan nearest-neighbour dilation.
8. Publish the original current-scan points belonging to the resulting dynamic voxels.

The EDF nearest-neighbour search uses cilantro's nanoflann-backed 3D KD-tree. Current and previous occupied voxels
take the exact zero-distance fast path and therefore do not issue a KD-tree query. The tracker uses cilantro radius
connected components in place of PCL Euclidean clustering, preserving the configured tolerance and cluster-size
limits. PCL remains only for the ROS `PointCloud2` conversion and point container boundary.

`occupancy_sigma` represents combined point and pose uncertainty and defaults to the voxel size. The fixed transition
matrix uses `transition_epsilon` to retain strong state self-transition probabilities. `belief_threshold` controls
when a voxel commits to a discrete occupied/free state. The convolution, window, Otsu, and dilation parameters under
`detector.hmm_mos` implement the second HMM-MOS stage. The first `local_window_size` scans warm up the score history
and intentionally produce no dynamic output. `global_window_size` resets stale beliefs and, together with
`active_radius`, bounds the hashed map.

## Attribution and disclaimer

The package vendors and uses the MIT-licensed [cilantro](https://github.com/kzampog/cilantro) library for KD-tree nearest-neighbour search and connected-component point-cloud clustering. Its license is retained in `external/cilantro/LICENSE` and installed as `share/dynamic_obstacle_tracker/licenses/cilantro-LICENSE`.

The HMM occupancy update, spatiotemporal convolution, Otsu segmentation, persistence, and dilation in
`src/detector/temporal_voxel_map.cpp` are adapted from the MIT-licensed [HMM-MOS](https://github.com/vb44/HMM-MOS) implementation accompanying “Moving Object Segmentation in Point Cloud Data using Hidden Markov Models.” The dense dataset I/O and map were replaced by this package's block-hashed,
timestamped ROS 2 implementation.

The upstream HMM-MOS package identifies the following MIT notice:

```text
MIT License

Copyright (c) 2024 Vedant Bhandari, Jasmin James, Tyson Phillips, Ross McAree

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

The tracker structure and implementation in `include/dynamic_obstacle_tracker/tracker/obstacle_tracker.hpp` and `src/tracker/obstacle_tracker.cpp` are adapted from the obstacle tracker in the MIT-licensed `global_mapper_ros` package from the [acl-mapping](https://gitlab.com/mit-acl/lab/acl-mapping/-/tree/mighty) repository.

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

## Citations

```bibtex
@inproceedings{zampogiannis2018cilantro,
  author={Zampogiannis, Konstantinos and Fermuller, Cornelia and Aloimonos, Yiannis},
  title={cilantro: A Lean, Versatile, and Efficient Library for Point Cloud Data Processing},
  booktitle={Proceedings of the 26th ACM International Conference on Multimedia},
  year={2018},
  pages={1364--1367},
  doi={10.1145/3240508.3243655}
}
```

```bibtex
@inproceedings{ryll2019efficient,
  title={Efficient Trajectory Planning for High Speed Flight in Unknown Environments},
  author={Ryll, Markus and Ware, John and Carter, John and Roy, Nick},
  booktitle={2019 International Conference on Robotics and Automation (ICRA)},
  pages={732--738},
  year={2019},
  organization={IEEE}
}
```

```bibtex
@inproceedings{tordesillas2019faster,
  title={{FASTER}: Fast and Safe Trajectory Planner for Flights in Unknown Environments},
  author={Tordesillas, Jesus and Lopez, Brett T and How, Jonathan P},
  booktitle={2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year={2019},
  organization={IEEE}
}
```

```bibtex
@article{tordesillas2018real,
  title={Real-Time Planning with Multi-Fidelity Models for Agile Flights in Unknown Environments},
  author={Tordesillas, Jesus and Lopez, Brett T and Carter, John and Ware, John and How, Jonathan P},
  journal={arXiv preprint arXiv:1810.01035},
  year={2018}
}
```

```bibtex
@misc{bhandari2024movingobjectsegmentationpoint,
      title={Moving Object Segmentation in Point Cloud Data using Hidden Markov Models},
      author={Vedant Bhandari and Jasmin James and Tyson Phillips and P. Ross McAree},
      year={2024},
      eprint={2410.18638},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2410.18638},
}
```

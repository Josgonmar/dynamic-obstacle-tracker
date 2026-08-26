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

The detector subscribes to `topics.deskewed_cloud_topic` and publishes:

- `topics.dynamic_cloud_topic` (`sensor_msgs/PointCloud2`)
- `static_points` (`sensor_msgs/PointCloud2`)

An empty `topics.dynamic_cloud_topic` uses the shared C++ default `/dynamic_point_detector/dynamic_points` for both detector output and tracker input.

Static points are only available when `detector.publish_static_cloud` is enabled, and are only materialized when `static_points` has at least one subscriber.

The tracker subscribes to the same resolved dynamic-cloud topic and publishes:

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

In either complete pipeline launch, `input_cloud_topic` only overrides the detector's raw deskewed input. The detector output and tracker input use `topics.dynamic_cloud_topic`, falling back to
`/dynamic_point_detector/dynamic_points` when it is empty. When launching only the tracker, its
`input_cloud_topic` argument overrides `topics.dynamic_cloud_topic`; an empty argument follows the same YAML then C++-default fallback. Use an absolute `topics.dynamic_cloud_topic` when connecting nodes in different namespaces.

## Frame contract

`frame.tracking_frame` is the single coordinate frame used by the detector output, tracker state, velocities, and predicted trajectories. It must be a continuous fixed frame such as `odom` or a continuous `world` frame, not `base_link` or a sensor frame.

The expected detector input is already expressed in `tracking_frame`. If its `header.frame_id` differs, the detector looks up `tracking_frame <- input_frame` at the point-cloud timestamp and transforms the points. A missing transform drops the scan; the implementation does not fall back to the latest transform.

The detector looks up the configured `frame.sensor_frame`, (`os_sensor` for Ouster LiDARs for example) in `tracking_frame` at the cloud timestamp and uses its translation as the DDA ray origin. A complete timestamped TF tree is therefore required. A missing cloud-frame or sensor-frame transform drops the scan; neither lookup falls back to the latest transform.

The tracker does not transform or relabel its input. It rejects dynamic clouds whose frame differs from `tracking_frame` and advances the EKF using the cloud timestamp.

Set `detector.debug: true` to print per-scan timing breakdowns for ROS conversion/TF/output handling, detector preprocessing/output assembly, and HMM-MOS ray casting, EDF, belief update, convolution, and segmentation. Set `tracker.debug: true` to report point, detection, association, and track-lifecycle counts plus timings for ROS conversion, downsampling, clustering, feature extraction, KF prediction, Hungarian association, state updates, marker and prediction generation, publication, and the complete callback.

## Dynamic-point detector

The detector stores HMM occupancy beliefs in sparse `8 x 8 x 8` voxel blocks addressed by integer block coordinates.
This follows the voxel-block spatial hashing organization and hash constants described by [Niessner et al.](https://niessnerlab.org/papers/2013/4hashing/niessner2013hashing.pdf), while replacing the paper's TSDF/color voxel payload with the three HMM states `unobserved`, `occupied`, and `free`.

Each scan is processed as one batch:

1. Reject non-finite points and points outside `minimum_range`/`maximum_range`. When ground removal is enabled, reject points below `ground_height + ground_clearance`; this is an absolute Z threshold in `tracking_frame`, not a fitted ground plane.
2. Quantize accepted endpoints into voxels, aggregate repeated hits, and traverse observed free-space voxels from the timestamped sensor origin with 3D DDA rays.
3. Construct a Gaussian Euclidean distance field from current and previous occupied scan voxels using a cilantro KD-tree.
4. Update every observed voxel's three-state belief using the fixed HMM transition model and soft EDF likelihood, committing a state only when its probability exceeds `belief_threshold`.
5. Seed motion from confident occupied-to-free and free-to-occupied state changes.
6. Apply the HMM-MOS spatial convolution, a `3 x 3 x 3` median filter, and temporal accumulation over `local_window_size` scans.
7. Segment the accumulated scores using Otsu thresholding, retain eligible previous-scan detections, and dilate to neighbouring current-hit voxels.
8. Publish the original, non-voxelized points whose endpoint voxels were classified as dynamic. Static output contains the other accepted points when requested.

The sparse map follows the sensor rather than remaining centered at the launch origin. `active_radius` removes blocks outside a sensor-centered sphere during periodic garbage collection, while `global_window_size` also removes blocks that have not been observed recently. `active_radius` should therefore be larger than `maximum_range`.

This is geometric motion segmentation: an object must produce a sufficiently persistent occupancy change to be detected. It does not classify semantic motion inside a voxel. Cloud deskew errors, inaccurate timestamped TF, odometry drift, coarse voxels, or aggressive thresholds can respectively create false changes or suppress slow/small motion.

## Dynamic-obstacle tracker

The tracker consumes only the dynamic points produced by the detector (or an equivalent cloud supplied directly). It does not run another motion classifier or transform the cloud.

For each accepted cloud it:

1. Removes non-finite points and optionally downsamples them with the cilantro voxel grid configured by `tracker.preprocessing.voxel_size`. A value of zero disables this stage.
2. Uses cilantro radius-neighbour connected components with `cluster_tolerance`, `min_cluster_size`, and `max_cluster_size`. Cluster sizes refer to the downsampled cloud.
3. Represents every accepted component by its arithmetic-mean centroid, raw XYZ axis-aligned bounding-box dimensions and center offset, and centroid covariance. The latter is the sample point covariance divided by the cluster size plus the isotropic `centroid_measurement_noise` variance floor. Components larger than `cluster_bbox_cutoff_size` are rejected.
4. Deletes expired states and predicts every remaining nine-state constant-acceleration KF `[position, velocity, acceleration]` once to the current cloud timestamp.
5. Constructs the complete track-to-detection squared Mahalanobis cost matrix from predicted position covariance, adaptive measurement covariance, and detection centroid covariance. A pair must pass both `maximum_association_distance`, a hard Euclidean limit that prevents covariance growth from accepting a remote cluster, and `mahalanobis_gate_squared`. A rectangular Hungarian solve then produces a global one-to-one assignment.
6. Corrects matched states, creates tentative states for unmatched detections, and increments the miss count of unmatched tracks. A tentative track becomes confirmed after `confirmation_hits`; it is removed after `tentative_max_missed_scans`, while a confirmed track is predicted and published for up to `time_to_delete_old_obstacles` seconds without a measurement.
7. With `use_adaptive_kf`, process and measurement covariance estimates are updated from the current innovation. Raw box dimensions and the offset between the mean centroid and AABB center are smoothed separately. Internal velocity and acceleration are bounded by `max_obstacle_velocity` and `max_obstacle_acceleration` before they can drive a coast.
8. After `min_observations_for_prediction`, publishes a prediction when KF speed exceeds `cutoff_length_threshold`. Velocity is capped at `max_obstacle_velocity`, and the output trajectory is a constant-velocity piecewise polynomial from the current cloud time through `prediction_horizon`.

Bounding-box and centroid markers use the KF track ID. A matched track is anchored to the current observed AABB center so its box remains on the detected obstacle; an unmatched track uses the KF centroid plus the smoothed AABB-center offset while coasting. Prediction arrows start at that same output position and end at the predicted position after `prediction_horizon`, so arrow direction shows velocity direction and arrow length shows predicted displacement.

The trajectory topic contains one `DynamicObstacleTrajectory` message per eligible confirmed obstacle rather than an array. Its filtered position, smoothed raw XYZ bounding box, covariance diagonals, polynomial coefficients, and times are all expressed in `tracking_frame` and stamped with the input cloud time. Confirmed tracks continue to produce markers and, when velocity-eligible, predictions while coasting; tentative tracks are NOT published.

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

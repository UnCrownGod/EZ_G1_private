# Unitree G1 Vision-Manipulation Roadmap

## Goals
- Detect test tube rack and drop-off pad using onboard YOLO model.
- Localize AprilTags / landmarks to ground robot pose.
- Execute grasp-and-place routine in simulation, then real robot.

## Architecture Overview
1. **Perception Layer**
   - Camera stream abstraction (USB/ROS image/recorded bag).
   - YOLO detector for rack & destination; optional low-res preview.
   - AprilTag pose estimation for robot-to-world alignment.
   - Publish structured ROS 2 messages (`vision_msgs`, `geometry_msgs`).
2. **Fusion Layer**
   - Maintain TF tree (`base`, `camera`, `tag`, `rack`, `target`).
   - Project detections into base frame using camera intrinsics + depth hints.
   - Provide grasp candidate selection API for behavior layer.
3. **Behavior / Control Layer**
   - Finite-state machine orchestrating seek -> align -> approach -> grasp -> place.
   - Interact with Unitree SDK 2 client (real or sim) for velocity and arm commands.
   - Safety checks: timeouts, watchdogs, E-stop integration.
4. **Bridges**
   - ROS 2 nodes wrapping perception and control logic.
   - FastAPI bridge for external debugging/visualization.
5. **Simulation**
   - Gazebo/Isaac sim with Unitree G1 URDF + camera plugin.
   - Test scenario: rack on table, drop zone; autop-run using same ROS graph.

## Immediate Tasks
- [x] Audit existing codebase and configs.
- [x] Refactor ROS 2 nodes for parameter-driven config, structured messages, and threading.
- [ ] Create reusable perception pipeline module with graceful shutdown.
- [ ] Expand control clients to include arm commands + simulated feedback.
- [ ] Add Unitree G1 URDF + basic Gazebo world (if licensing permits) or placeholder mock.
- [ ] Document end-to-end workflow in README.

## Dependencies & Tooling
- ROS 2 Humble / Iron (rclpy, vision_msgs, geometry_msgs, tf-transformations).
- OpenCV (with contrib), Ultralytics, numpy, PyYAML.
- Unitree SDK 2 Python bindings or gRPC interface (TBD).
- Gazebo/Ignition or Isaac (simulation).

## Notes
- All config paths must be ROS parameters to allow launch files to override.
- Keep CPU/GPU heavy inference off ROS executor thread; use background workers.
- Provide deterministic seeds for simulation/test reproducibility.

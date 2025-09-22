# Unitree G1 MuJoCo Demo Plan

## Objective
Simulate the Unitree G1 picking a box from a table inside MuJoCo, using the same perception/control stack we are building for the real robot. The MuJoCo scene must include:
- G1 base + arm model (URDF/MJCF converted as needed)
- Table with target box (spawned at configurable pose)
- Camera sensor providing images to the perception node
- Controller bridge to drive locomotion + arm joints

## Assets Required
1. **Robot description**: Official Unitree G1 URDF or vendor MJCF. If only ROS URDF exists, convert to MJCF with `mjcf.from_path` (MuJoCo 3.x) or Mujoco-HAPTIX converter.
2. **Gripper mesh/actuators**: Ensure finger joints have appropriate limits and mimic real hardware.
3. **Environment meshes**: Simple cube for the rack/box, plane for table, optional environment textures.
4. **Perception pipeline**: Use the existing vision node with simulated camera feed; either render MuJoCo RGB frames to disk or stream via ROS topic (e.g., `image_transport` bridge).

## Repository Layout
```
vision/sim/mujoco/
├─ assets/                # MJCF/URDF meshes (to be added)
├─ configs/               # YAML for table/box poses, camera intrinsics
├─ scenes/                # Generated MJCF scene XMLs
├─ scripts/
│   ├─ build_scene.py     # Compose MJCF from assets + config
│   └─ run_demo.py        # Launch MuJoCo sim + ROS bridges
└─ README.md              # This file
```

## Immediate To-Do
- [ ] Acquire/verify Unitree G1 URDF and joints (arm + gripper).
- [ ] Draft `build_scene.py` to assemble a MuJoCo model programmatically.
- [ ] Prototype `run_demo.py` to spawn simulation, publish camera frames, and accept velocity/arm commands.
- [ ] Integrate with ROS 2 via `ros2_control` or custom Python bridge (decide based on available bindings).
- [ ] Add launch file that wires MuJoCo sim + vision node + control FSM.

## Notes
- MuJoCo runs on Windows, but ROS 2 tooling (rclpy, rviz, tf) is far smoother on Ubuntu 22.04. Consider developing the sim on Ubuntu WSL2 or a dedicated Linux machine.
- The new perception pipeline can process camera frames if we expose MuJoCo renderer via Python.
- Unit tests should stub the MuJoCo renderer to keep CI light.

# Checkpoint 14: real-robot run guide

The learner reported a successful full real-robot run on 2026-09-08. Formal grading is separate. This guide assumes the course robot connection, robot driver, `zenoh-pointcloud` repository, ROS 2 Humble, and this project already exist in the course workspace.

## Update and build (cloud terminal)

```bash
cd ~/ros2_ws/src/manipulation_project
git pull --ff-only origin main
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select object_detection moveit2_scripts
source install/setup.bash
```

## Prepare camera reception and Python PCL

After connecting to the real robot, run in a dedicated terminal:

```bash
bash ~/ros2_ws/src/manipulation_project/object_detection/scripts/prepare_real_perception.sh
```

The script checks Python PCL first and installs `python3-pcl` if needed (sudo may be required). It then checks for actual nonempty PointCloud2 messages on `/camera/depth/color/points`, not merely a topic name. If reception already works, it exits without starting another bridge.

If Zenoh is missing, the helper calls the course `~/ros2_ws/src/zenoh-pointcloud/install_zenoh.sh`. It updates only the known obsolete course endpoint, backs up a changed configuration, starts its own bridge, and verifies reception. Keep this terminal open if the script started a bridge; Ctrl+C stops that bridge. An existing nonworking bridge is reported rather than automatically terminated.

The historical endpoint repair is not a discovery service. If The Construct supplies a different address, pass that confirmed endpoint explicitly:

```bash
bash ~/ros2_ws/src/manipulation_project/object_detection/scripts/prepare_real_perception.sh \
  --endpoint 'tcp/ADDRESS:7447'
```

Replace ADDRESS with the platform-provided address. Do not run this placeholder literally. For Python PCL alone:

```bash
bash ~/ros2_ws/src/manipulation_project/object_detection/scripts/check_pcl.sh
```

Preparation sends no robot motion commands. Repository signature errors, missing network routes, or an unavailable remote endpoint may still require platform support; installation alone cannot repair those services.

## Plan and execute

In another terminal, source the workspace and start MoveIt once:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch real_moveit_config move_group.launch.py
```

Use the existing real MoveIt RViz for trajectory review. The pick launch intentionally does not start another RViz. Preview without robot commands:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch moveit2_scripts pick_and_place_perception_real.launch.py execute:=false
```

To execute the full real-robot workflow:

```bash
ros2 launch moveit2_scripts pick_and_place_perception_real.launch.py
```

Default sequence: recover/confirm home, open, pregrasp, descend 60 mm, close at 0.643, lift 60 mm, shoulder +pi transfer, release, and return home. All arm segments must plan successfully before motion starts. Execution failures stop subsequent stages. A new launch replans; it does not reuse a previous process's preview.

The real calibration uses:

```text
x = detected_x - thickness / 2 + 0.003
y = detected_y + width / 2 - 0.002
grasp_z = detected_z + height / 2 + 0.1923
pregrasp_z = grasp_z + 0.060
```

Units are metres. These are lab-specific empirical corrections, not a universal camera or tool calibration. Detection output is in base_link; the observed world-to-base_link transform is identity in this lab. Keep the verified real tool0 orientation. The detector publishes measured object values independently of these grasp offsets.

Success log: `REAL_PICK_PLACE_CONTINUATION PASS: complete sequence finished.` Also verify physically that the object was held, transferred, and released; trajectory completion alone does not establish grasp success.
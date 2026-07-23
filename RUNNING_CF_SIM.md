# Running OpenVINS with CF sim

How to set up and run `ov_msckf`'s stereo-inertial estimator against the
simulated `px4vision_custom` drone (ZED stereo camera + MAVROS IMU) on
**ROS 2 Jazzy / Ubuntu 24.04**. See the project's [ReadMe.md](ReadMe.md) for
OpenVINS itself; this doc only covers this workspace's `cf_sim_launch.py` path.


## 1. Install Ceres Solver (from source)

`ov_msckf`'s `CMakeLists.txt` does a hard `find_package(Ceres REQUIRED)`, and
Noble has no `libceres-dev` apt package, so build it from source.

Install Ceres' build deps first, so its CMake auto-detects SuiteSparse for
sparse Cholesky factorization:

```bash
sudo apt install -y \
    libeigen3-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev
```

Clone, configure with SuiteSparse enabled, build, and install:

```bash
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
mkdir build && cd build
cmake .. -DSUITESPARSE=ON
make -j$(nproc)
sudo make install
sudo ldconfig   # refresh the linker cache so colcon/CMake can find libceres.so
```

## 2. Install `image_transport` / `tf2` / MAVROS dependencies

`ov_msckf` publishes debug images through `image_transport` and broadcasts
poses with `tf2_ros::TransformBroadcaster` (`ov_msckf/src/ros/ROS2Visualizer.cpp`),
and `cf_sim_launch.py` consumes IMU data via MAVROS:

```bash
sudo apt install -y \
    ros-jazzy-image-transport \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-cv-bridge \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-ros-py \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras
```

## 3. Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select ov_core ov_init ov_msckf
source install/setup.bash
```

Check the CMake output for the line `OPENCV: ... | BOOST: ... | CERES: ...`
from `ov_msckf/CMakeLists.txt` — if `CERES` is blank, step 1 didn't finish
(usually a missing `sudo ldconfig`).

## 4. Run

`cf_sim_launch.py` only starts `run_subscribe_msckf`; it does not launch the
drone simulation itself. Make sure the PX4/Gazebo sim + MAVROS are already up
and publishing:

- `/px4vision_custom_0/zedx_left/image`
- `/px4vision_custom_0/zedx_right/image`
- `/px4vision_custom_0/mavros/imu/data`

Then, in a sourced terminal:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch ov_msckf cf_sim_launch.py
```

This runs `run_subscribe_msckf` with `config/cf/cf_stereo.yaml` +
`config/cf/cf_stereo_ros.yaml`, remapped to the topics above.

## Troubleshooting

- **Segfault in `run_subscribe_msckf`** — OpenCV ABI mismatch: the OpenCV
  version `ov_msckf` links against must match the one `cv_bridge` was built
  against (see the comment at the top of `ov_msckf/CMakeLists.txt`).
- **CMake can't find Ceres** — re-run `sudo ldconfig`, or confirm
  `/usr/local/lib` is on the linker search path.
- **No IMU/image data reaching OpenVINS** — the sim isn't fully up yet;
  `cf_sim_launch.py` will sit idle until something publishes on the three
  topics above.

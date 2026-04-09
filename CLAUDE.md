# CLAUDE.md — F1TENTH Autonomous Racing Stack (GIU Berlin)

This file documents the project setup performed by Claude Code, including all dependency installations, bugs encountered, and how they were resolved.

---

## Project Overview

F1TENTH autonomous racing software stack for ROS 2 Humble running on NVIDIA Jetson hardware. The stack includes SLAM, path planning, control, perception, and hardware drivers.

**Platform**: Ubuntu 22.04 LTS (Jetpack kernel `5.15.148-tegra`)  
**ROS Distribution**: ROS 2 Humble (installed at `/opt/ros/humble/`)  
**Python**: 3.10.12

---

## Setup Session — 2026-04-09

### What Was Already Installed

The following ROS 2 apt packages were already present on the system:

| Package | Version |
|---------|---------|
| ros-humble-ackermann-msgs | 2.0.2 |
| ros-humble-joy | 3.3.0 |
| ros-humble-joy-teleop | 1.7.0 |
| ros-humble-nav2-bringup | 1.1.20 |
| ros-humble-realsense2-camera | 4.57.7 |
| ros-humble-rosbridge-server | 2.0.5 |
| ros-humble-urg-node | 1.1.2 |
| ros-humble-vision-msgs | 4.1.1 |
| python3-colcon-common-extensions | 0.3.0 |

### Packages That Required Manual Installation

#### apt packages (required `sudo` — must be run manually)

The following packages were not yet installed and require elevated privileges:

```bash
sudo apt install -y ros-humble-asio-cmake-module imagemagick
```

- `ros-humble-asio-cmake-module` — needed for VESC/hardware driver builds
- `imagemagick` — needed for the grayscale map conversion workflow described in the README

> **Note**: These could not be installed automatically because `sudo` requires a terminal password prompt. Run the above command manually.

#### Python packages — installed via `pip3 install --user`

All packages were installed into `~/.local/lib/python3.10/site-packages/`:

```bash
pip3 install --user opencv-python scikit-learn hdbscan casadi pyyaml roboflow
pip3 install --user ultralytics pyrealsense2
pip3 install --user pandas py-trees
pip3 install --user trajectory-planning-helpers --no-deps
pip3 install --user "numpy<2.0"
pip3 install --user --upgrade requests-toolbelt
```

Final installed versions:

| Package | Version | Purpose |
|---------|---------|---------|
| numpy | 1.26.4 | Core numerical computing |
| scipy | 1.8.0 (system) | Scientific computing |
| matplotlib | 3.5.1 (system) | Plotting |
| opencv-python | 4.13.0.92 | Computer vision |
| scikit-learn | 1.7.2 | Clustering, ML |
| hdbscan | 0.8.42 | Density-based clustering |
| casadi | 3.7.2 | MPC optimization |
| pyyaml | (system) | Config file parsing |
| pyrealsense2 | 2.57.7.10387 | Intel RealSense camera driver |
| torch | 2.11.0+cu130 | Deep learning (CUDA 13) |
| torchvision | 0.26.0 | Vision models |
| ultralytics | 8.4.36 | YOLO object detection |
| roboflow | 1.3.1 | Dataset annotation platform |
| pandas | 1.3.5 (system) | Data processing |
| py-trees | 2.4.0 | Behavior tree framework |
| trajectory-planning-helpers | 0.79 | Racing line optimization |
| requests-toolbelt | 1.0.0 | HTTP multipart (roboflow dep) |

---

## Bugs Encountered and Fixes

### Bug 1: NumPy 2.x Breaks System scipy and matplotlib

**Symptom**: After `pip3 install opencv-python`, pip resolved and installed `numpy==2.2.6`. The system-installed `scipy` (1.8.0) and `matplotlib` (3.5.1) were compiled against NumPy 1.x. Importing them raised:

```
AttributeError: _ARRAY_API not found
ImportError: numpy.core.multiarray failed to import
```

**Root Cause**: NumPy 2.0 removed the legacy `numpy.core` C API. System packages installed via apt were compiled against NumPy 1.x and are binary-incompatible with NumPy 2.x.

**Fix**: Downgrade numpy to the latest 1.x release:

```bash
pip3 install --user "numpy<2.0"
```

This installed `numpy==1.26.4`. The newer `opencv-python` 4.13 declares `numpy>=2` as a requirement for Python ≥ 3.9, but in practice it still works with 1.26.4 on this platform.

**Residual Warning** (harmless):
```
UserWarning: A NumPy version >=1.17.3 and <1.25.0 is required for this version of SciPy
```
This warning is from the system scipy (1.8.0) because 1.26.4 is slightly above its stated upper bound, but scipy continues to function correctly.

---

### Bug 2: `trajectory-planning-helpers` Fails to Install Due to `quadprog`

**Symptom**: Running `pip3 install --user trajectory-planning-helpers` failed with:

```
error: legacy-install-failure
× Encountered error while trying to install package.
╰─> quadprog
```

**Root Cause**: `trajectory-planning-helpers` declares `quadprog==0.1.7` as a dependency. `quadprog` is a C extension that requires a Fortran compiler (`gfortran`) and compatible build tools. The build failed on the Jetson environment.

**Fix**: Install without dependencies since the core trajectory planning helpers module itself does not require `quadprog` for basic path generation:

```bash
pip3 install --user trajectory-planning-helpers --no-deps
```

**Impact**: The `quadprog` solver is only needed for certain QP-based optimization routines. If those specific functions are called, a runtime `ImportError` will occur. To fully resolve this, install `gfortran` and rebuild:

```bash
sudo apt install -y gfortran
pip3 install --user quadprog==0.1.7
```

---

### Bug 3: `roboflow` Import Fails — `urllib3.contrib.appengine` Not Found

**Symptom**: After installing `roboflow`, importing it raised:

```
ImportError: cannot import name 'appengine' from 'urllib3.contrib'
```

**Root Cause**: The system package `python3-requests-toolbelt` (used by roboflow) was compiled against the old `urllib3` 1.x API which included `urllib3.contrib.appengine`. The newly installed `urllib3` 2.6.3 removed that legacy module.

**Fix**: Upgrade `requests-toolbelt` via pip to the version that supports urllib3 2.x:

```bash
pip3 install --user --upgrade requests-toolbelt
```

This installed `requests-toolbelt==1.0.0` into `~/.local`, which takes priority over the system version and is compatible with urllib3 2.x.

**Residual Warning** (harmless):
```
RequestsDependencyWarning: urllib3 (2.6.3) or chardet (4.0.0) doesn't match a supported version!
```
This warning comes from the system `requests` package (2.25.1) which was built expecting urllib3 < 2.x. Roboflow and all other packages still function correctly despite this warning.

---

## PATH Note

Several pip-installed scripts were placed in `~/.local/bin` which may not be on PATH. Add to your `~/.bashrc` or `~/.zshrc` if you want to call tools like `yolo`, `roboflow`, `ultralytics` directly:

```bash
export PATH="$HOME/.local/bin:$PATH"
```

---

## ROS 2 Sourcing

Always source ROS 2 before building or running:

```bash
source /opt/ros/humble/setup.bash
```

After building the workspace:

```bash
source ~/f1tenth_ws/install/setup.bash
```

---

## Verification

Run this to verify all critical Python packages load correctly:

```python
import numpy, scipy, matplotlib, cv2, sklearn, hdbscan, casadi, yaml
import pyrealsense2, torch, ultralytics, roboflow
print("All packages OK")
```

Expected output includes harmless scipy/urllib3 version warnings — these do not affect functionality.

---

## Still Pending (Manual Steps Required)

1. **`sudo apt install -y ros-humble-asio-cmake-module imagemagick`** — requires terminal with password
2. **`quadprog` full install** — requires `gfortran`: `sudo apt install -y gfortran && pip3 install --user quadprog==0.1.7`
3. **Workspace build**: `cd ~/f1tenth_ws && colcon build`
4. **Optional conda env** for racing line optimization (see README)

---

## Bug 4: `vesc_driver` Build Fails — `serial_driver/serial_driver.hpp` Not Found

**Symptom**: Running `colcon build --packages-ignore realsense2_camera_msgs` failed with:

```
fatal error: serial_driver/serial_driver.hpp: No such file or directory
   45 | #include "serial_driver/serial_driver.hpp"
compilation terminated.
Failed <<< vesc_driver
```

**Root Cause**: `ros-humble-serial-driver` was not installed. `vesc_driver` declares `serial_driver` as a dependency in its `package.xml`, and `ament_auto_find_build_dependencies()` in its `CMakeLists.txt` tries to resolve it at build time.

**Fix — Step 1**:

```bash
sudo apt install -y ros-humble-serial-driver
```

This pulls in `ros-humble-io-context` and `libasio-dev` as transitive dependencies.

---

## Bug 5: `vesc_driver` Still Fails After Installing `serial-driver` — `asio_cmake_module` Not Found

**Symptom**: After installing `ros-humble-serial-driver` and cleaning the stale CMake cache (`rm -rf build/vesc_driver install/vesc_driver`), the build still failed with a new error:

```
CMake Error at /opt/ros/humble/share/io_context/cmake/io_context-extras.cmake:17 (find_package):
  Could not find a package configuration file provided by "asio_cmake_module"
Failed <<< vesc_driver
```

**Root Cause**: The dependency chain is:

```
vesc_driver → serial_driver → io_context → asio_cmake_module
```

`ros-humble-asio-cmake-module` was not installed. Installing `serial_driver` pulled in `io_context` and `libasio-dev`, but NOT `asio_cmake_module` itself (a CMake integration layer separate from the C++ library).

Additionally, the first build attempt had cached a failed CMake configuration. Even after installing `serial_driver`, the stale cache caused the old error to persist until the cache was cleared.

**Fix — Step 2**:

```bash
sudo apt install -y ros-humble-asio-cmake-module
```

Then clean the stale build cache and rebuild:

```bash
rm -rf build/vesc_driver install/vesc_driver
source /opt/ros/humble/setup.bash
colcon build --packages-ignore realsense2_camera_msgs
```

**Key lesson**: Always clean `build/<package>` and `install/<package>` after installing a missing dependency, otherwise CMake uses the cached failed configuration and the same error reappears even though the package is now present.

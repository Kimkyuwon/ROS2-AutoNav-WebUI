# ROS2 Web GUI

A web-based graphical user interface for ROS2 robotics applications, providing intuitive control and visualization of SLAM, localization, and data recording/playback functionality through a modern web browser.

## Features

### SLAM and Localization
- **LiDAR SLAM**
  - YAML configuration management with live parameter editing
  - Real-time terminal output monitoring (latest 10 lines)
  - Start/Stop SLAM processes with single click
  - Automatic map saving via pose graph optimization
  - Compatible with FAST-LIO mapping

- **Localization**
  - YAML configuration management
  - Real-time localization process control
  - Terminal output monitoring
  - Compatible with FAST-LIO localization

- **Multi-Session SLAM**
  - Load and merge multiple SLAM maps
  - Run optimization in separate terminal window
  - Crash-isolated execution

### Data Management
- **Bag Player**
  - Play ROS2 bag files with topic selection
  - Timeline control with play/pause
  - 3D visualization support

- **Bag Recorder**
  - Record live ROS2 topics to bag files
  - Selective topic recording
  - Real-time recording status display

- **File Player (ConPR Format)**
  - Play CSV-based trajectory data (pose, IMU)
  - Variable playback speed (0.01x ~ 20.0x)
  - Convert to ROS2 bag format
  - Support for Livox LiDAR and camera data

### Visualization
- **3D Display (Three.js)**
  - Real-time PointCloud2 visualization
  - Path and Odometry display
  - Interactive camera control
  - WebSocket-based low-latency rendering

### Network Monitoring
- **Latency Indicator**
  - Real-time network latency measurement
  - Color-coded status (green/yellow/red)
  - Ideal for remote smartphone access

## Demo

**Web Interface:**
- Clean, modern UI with tabbed navigation
- Sticky navigation for easy access
- Real-time status updates
- Mobile-friendly responsive design

**Access:**
- Local: `http://localhost:8080`
- Network: `http://<YOUR_IP>:8080`

## Prerequisites

### 1. Ubuntu and ROS2

**Ubuntu:** >= 20.04

**ROS2:** >= Jazzy. [ROS2 Installation](https://docs.ros.org/en/jazzy/Installation.html)

For ROS2 Jazzy Desktop Full, all core dependencies are included.

### 2. Python Dependencies

All required Python packages are included in ROS2 Desktop Full:
- `rclpy` - ROS2 Python library
- `std_msgs`, `sensor_msgs`, `geometry_msgs` - Standard message types
- `rosbag2_py` - Bag file handling
- `cv_bridge` - OpenCV-ROS bridge (optional, for camera support)

### 3. rosbridge_server (For 3D Visualization)

```bash
sudo apt install ros-jazzy-rosbridge-server
```

### 4. Optional Dependencies

**livox_ros_driver2** (for Livox LiDAR data playback):
```bash
cd ~/your_workspace/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd ~/your_workspace
colcon build --packages-select livox_ros_driver2
```

**FAST-LIO** (for SLAM/Localization features):
```bash
cd ~/your_workspace/src
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO
git submodule update --init
cd ~/your_workspace
colcon build --packages-select fast_lio
```

## Installation

### Clone the Repository

```bash
cd ~/your_workspace/src
git clone https://github.com/YOUR_USERNAME/web_gui.git
```

### Build

```bash
cd ~/your_workspace
colcon build --packages-select web_gui
source install/setup.bash
```

**Important:** After modifying source files in `src/web_gui/`, you must rebuild:
```bash
colcon build --packages-select web_gui
```

For faster development iteration, use symlink install (one-time setup):
```bash
colcon build --symlink-install
```

## Run

### ROS2 Launch (Recommended)

```bash
source /opt/ros/jazzy/setup.bash
source ~/your_workspace/install/setup.bash
ros2 launch web_gui web_gui.launch.py
```

### Access Web Interface

Once the server starts, you'll see:
```
[INFO] [web_gui_node]: ======================================
[INFO] [web_gui_node]: Web server started on port 8080
[INFO] [web_gui_node]: Local access:   http://localhost:8080
[INFO] [web_gui_node]: Network access: http://YOUR_IP:8080
[INFO] [web_gui_node]: ======================================
```

Open the URL in your web browser.

## Usage

### LiDAR SLAM

1. **Load Configuration**
   - Click "Config Load" to open file browser
   - Default: `/path/to/FAST_LIO_ROS2/config/mapping_config.yaml`
   - Edit parameters in real-time

2. **Save Configuration**
   - Click "Set Config File" to save changes
   - Configuration is saved with comments preserved

3. **Start SLAM**
   - Click "Start SLAM" button
   - Monitor real-time terminal output
   - Green status indicator shows running state

4. **Stop SLAM**
   - Click "Stop SLAM" button
   - Process terminates gracefully (SIGINT → SIGTERM → SIGKILL)

5. **Save Map**
   - Click "Save Map" to trigger pose graph optimization
   - Map is saved to configured output directory

### Localization

Same workflow as SLAM, but uses `localization_config.yaml` and `localization.launch.py`.

### Bag Recorder

1. Enter bag name in "Bag Name" field
2. Click "Enter Bag Name"
3. Click "Select Topic" to choose topics
4. Click "Record" to start recording
5. Click "Stop" to finish recording
6. Files saved to `/home/YOUR_USERNAME/dataset/BAG_NAME/`

### 3D Visualization

1. Click "Select Topics" button
2. Choose PointCloud2, Path, or Odometry topics
3. Click "Confirm"
4. Use mouse to interact:
   - **Drag**: Rotate camera
   - **Scroll**: Zoom in/out
   - **Right-click drag**: Pan

## Configuration

### ROS2 Environment Variables

The web_gui service automatically sets:
```bash
ROS_DOMAIN_ID=0
ROS_LOCALHOST_ONLY=1
```

For manual ROS2 commands in other terminals, set the same variables:
```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
source /opt/ros/jazzy/setup.bash
```

### Firewall Settings (for Network Access)

**Ubuntu/Linux:**
```bash
sudo ufw allow 8080/tcp
sudo ufw reload
```

**Windows:**
- Windows Defender Firewall → Advanced Settings → Inbound Rules
- New Rule → Port → TCP 8080 → Allow

### Change Port (Optional)

Edit `web_gui/web_server.py`:
```python
# Change port number
web_thread = threading.Thread(target=run_web_server, args=(node, 8080), daemon=True)
# Change 8080 to your desired port
```

Then rebuild:
```bash
colcon build --packages-select web_gui
```

## Troubleshooting

### Web Interface Not Loading

**Check if service is running:**
```bash
sudo systemctl status web_gui
# or
ros2 node list | grep web_gui
```

**Check port availability:**
```bash
sudo netstat -tlnp | grep 8080
```

**Restart service:**
```bash
sudo systemctl restart web_gui
```

### Changes Not Reflecting

**Did you rebuild?**
```bash
colcon build --packages-select web_gui
```

**Hard refresh browser:**
- Press `Ctrl + F5` (Windows/Linux)
- Press `Cmd + Shift + R` (Mac)

**Restart web_gui:**
```bash
sudo systemctl restart web_gui
```

### ROS2 Topics Not Visible

**Check ROS environment:**
```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
source /opt/ros/jazzy/setup.bash
ros2 daemon stop && ros2 daemon start
ros2 topic list
```

### 3D Visualization Not Working

**Check rosbridge:**
```bash
ros2 node list | grep rosbridge
```

**Should see:**
```
/rosbridge_websocket
```

**If missing, restart web_gui:**
```bash
sudo systemctl restart web_gui
```

## Directory Structure

```
web_gui/
├── web_gui/
│   ├── web_server.py          # Main Python backend (HTTP + ROS2)
│   └── __init__.py
├── web/
│   ├── index.html             # Main HTML page
│   └── static/
│       ├── script.js          # JavaScript (UI logic, API calls)
│       ├── style.css          # CSS styling
│       └── threejs_display.js # Three.js 3D visualization
├── launch/
│   └── web_gui.launch.py      # ROS2 launch file
├── resource/
├── package.xml
├── setup.py
├── README.md                  # This file
└── Project.md                 # Detailed project documentation
```

## API Endpoints

### SLAM
- `POST /api/slam/load_config_file` - Load SLAM config
- `POST /api/slam/save_config_file` - Save SLAM config
- `POST /api/slam/start_mapping` - Start SLAM
- `POST /api/slam/stop_mapping` - Stop SLAM
- `GET /api/slam/get_terminal_output` - Get terminal output

### Localization
- `POST /api/localization/load_config_file` - Load localization config
- `POST /api/localization/save_config_file` - Save localization config
- `POST /api/localization/start_mapping` - Start localization
- `POST /api/localization/stop_mapping` - Stop localization
- `GET /api/localization/get_terminal_output` - Get terminal output

### Bag Recorder
- `POST /api/recorder/set_bag_name` - Set bag name
- `GET /api/recorder/get_topics` - Get available topics
- `POST /api/recorder/record` - Start/stop recording
- `GET /api/recorder/state` - Get recorder state

### Network
- `GET /api/ping` - Ping server for latency measurement

## Development

### Code Modification Workflow

1. Stop the service:
```bash
sudo systemctl stop web_gui
```

2. Edit source files in `src/web_gui/`

3. Rebuild:
```bash
cd ~/your_workspace
colcon build --packages-select web_gui
```

4. Restart the service:
```bash
sudo systemctl start web_gui
```

5. Hard refresh browser (`Ctrl + F5`)

### Adding New Features

**Backend (Python):**
- Add functions to `web_gui/web_server.py`
- Add API endpoints in `do_GET()` or `do_POST()`

**Frontend (JavaScript):**
- Add functions to `web/static/script.js`
- Use `apiCall(endpoint, data)` for backend communication

**UI (HTML):**
- Add elements to `web/index.html`
- Style in `web/static/style.css`

## Related Works

### SLAM
- [FAST-LIO](https://github.com/hku-mars/FAST_LIO) - Fast LiDAR-Inertial Odometry
- [FAST-LIO-LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION) - Localization module
- [ikd-Tree](https://github.com/hku-mars/ikd-Tree) - Dynamic KD-Tree for 3D kNN search

### Visualization
- [Three.js](https://threejs.org/) - 3D graphics library
- [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite) - WebSocket interface to ROS

## Acknowledgments

Thanks to the developers of:
- [FAST-LIO](https://github.com/hku-mars/FAST_LIO) for robust SLAM algorithms
- [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite) for ROS-web communication
- [Three.js](https://threejs.org/) for 3D visualization capabilities

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

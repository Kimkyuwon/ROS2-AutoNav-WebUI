#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.serialization import serialize_message
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, Imu, CameraInfo
from geometry_msgs.msg import PointStamped
from rosgraph_msgs.msg import Clock
from cv_bridge import CvBridge
import cv2
import struct
import glob
import threading
import json
import os
import time
from http.server import HTTPServer, SimpleHTTPRequestHandler
from urllib.parse import parse_qs, urlparse
import subprocess
import signal
import yaml
from pathlib import Path as PathLib
import rosbag2_py

# Try to import ruamel.yaml for better formatting
try:
    from ruamel.yaml import YAML
    RUAMEL_AVAILABLE = True
except ImportError:
    RUAMEL_AVAILABLE = False
    print("Warning: ruamel.yaml not available. Comments and formatting may not be preserved.")

# Try to import Livox custom messages
try:
    from livox_ros_driver2.msg import CustomMsg, CustomPoint
    LIVOX_AVAILABLE = True
except ImportError:
    LIVOX_AVAILABLE = False
    print("Warning: livox_ros_driver2 messages not available. LiDAR publishing will be disabled.")

# Try to import pose_graph_optimization service
try:
    from pose_graph_optimization.srv import SaveMap
    SAVEMAP_AVAILABLE = True
except ImportError:
    SAVEMAP_AVAILABLE = False
    print("Warning: pose_graph_optimization SaveMap service not available. Map saving will be disabled.")

class WebGUINode(Node):
    def __init__(self):
        super().__init__('web_gui_node')

        # SLAM GUI state
        self.slam_map1 = ""
        self.slam_map2 = ""
        self.slam_output = ""
        self.slam_status = "Ready"
        self.slam_process = None
        self.slam_terminal_output = ""
        self.slam_output_lock = threading.Lock()

        # Localization state
        self.localization_process = None
        self.localization_terminal_output = ""
        self.localization_output_lock = threading.Lock()

        # Bag Player state
        self.bag_path = ""
        self.bag_playing = False
        self.bag_paused = False
        self.bag_process = None

        # Bag Recorder state
        self.recorder_bag_name = ""
        self.recorder_recording = False
        self.recorder_process = None
        self.bag_topics = []
        self.bag_selected_topics = []
        self.bag_duration = 0.0  # Duration in seconds
        self.bag_current_time = 0.0  # Current playback time in seconds
        self.bag_start_offset = 0.0  # Start offset for playback
        self.bag_start_real_time = 0.0  # Real time when playback started
        self.bag_pause_time = 0.0  # Time when paused

        # File Player state
        self.player_path = ""
        self.player_playing = False
        self.player_paused = False
        self.player_loop = False
        self.player_skip_stop = True
        self.player_auto_start = False
        self.player_speed = 1.0
        self.player_timestamp = 0
        self.player_slider_pos = 0
        self.player_initial_stamp = 0
        self.player_last_stamp = 0
        self.player_data_loaded = False
        self.player_processed_stamp = 0
        self.player_prev_time = 0

        # File Player ROS2 Publishers
        self.pose_pub = self.create_publisher(PointStamped, '/pose/position', 1000)
        self.imu_pub = self.create_publisher(Imu, '/imu', 1000)
        self.clock_pub = self.create_publisher(Clock, '/clock', 1)

        # LiDAR and Camera publishers
        if LIVOX_AVAILABLE:
            self.livox_pub = self.create_publisher(CustomMsg, '/livox/lidar', 1000)
        else:
            self.livox_pub = None

        self.cam_pub = self.create_publisher(Image, '/camera/color/image', 1000)
        self.cam_info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 1000)

        # CV Bridge for image conversion
        self.cv_bridge = CvBridge()

        # File Player ROS2 Subscribers
        self.start_sub = self.create_subscription(
            Bool, '/file_player_start', self.file_player_start_callback, 1)
        self.stop_sub = self.create_subscription(
            Bool, '/file_player_stop', self.file_player_stop_callback, 1)

        # SLAM Subscribers
        self.slam_complete_sub = self.create_subscription(
            Bool, '/lt_mapping_complete', self.slam_complete_callback, 10)

        # File Player data structures
        self.data_stamp = {}
        self.pose_data = {}
        self.imu_data = {}
        self.livox_file_list = []  # List of LiDAR .bin files
        self.cam_file_list = []    # List of camera image files
        self.livox_cache = {}      # Cache for loaded LiDAR data
        self.cam_cache = {}        # Cache for loaded camera images

        # Playback thread
        self.playback_thread = None
        self.playback_active = False

        # Timer for playback (matching C++ implementation)
        self.create_timer(0.0001, self.timer_callback)  # 100us = 0.0001s

        # Timer for bag playback time tracking
        self.create_timer(0.1, self.bag_timer_callback)  # 100ms = 0.1s

        # Setup reusable environment for subprocess calls
        self._setup_ros_environment()

        self.get_logger().info('Web GUI Node initialized with full ROS2 integration')

    def _setup_ros_environment(self):
        """
        Setup reusable ROS2 environment for subprocess calls.
        This avoids re-sourcing setup.bash on every subprocess call.
        """
        # Get sourced environment once and cache it
        bash_cmd = (
            'source /opt/ros/jazzy/setup.bash && '
            'source /home/kkw/localization_ws/install/setup.bash && '
            'env'
        )
        try:
            result = subprocess.run(
                ['bash', '-c', bash_cmd],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                # Parse environment variables
                self._ros_env = os.environ.copy()
                for line in result.stdout.split('\n'):
                    if '=' in line:
                        key, _, value = line.partition('=')
                        self._ros_env[key] = value
                self.get_logger().info('ROS2 environment cached successfully')
            else:
                # Fallback to current environment
                self._ros_env = os.environ.copy()
                self.get_logger().warn('Failed to source ROS2 environment, using current environment')
        except Exception as e:
            self._ros_env = os.environ.copy()
            self.get_logger().error(f'Error setting up ROS2 environment: {str(e)}')

    def _read_process_output(self, process, output_lock, output_attr_name, max_lines=10):
        """
        Thread function to read process output and store in terminal output buffer.

        Args:
            process: The subprocess.Popen object to read from
            output_lock: Threading lock for output synchronization
            output_attr_name: Name of the attribute to store output (e.g., 'slam_terminal_output')
            max_lines: Maximum number of lines to keep in buffer (default: 10)
        """
        try:
            for line in iter(process.stdout.readline, ''):
                if line:
                    with output_lock:
                        current_output = getattr(self, output_attr_name)
                        current_output += line
                        # Keep only last max_lines lines
                        lines = current_output.split('\n')
                        if len(lines) > max_lines:
                            # Keep last max_lines lines (including any incomplete line at the end)
                            current_output = '\n'.join(lines[-max_lines:])
                        setattr(self, output_attr_name, current_output)
        except Exception as e:
            self.get_logger().error(f'Error reading process output: {str(e)}')

    def _stop_process(self, process, process_name, output_lock=None, output_attr_name=None):
        """
        Stop a running process gracefully using SIGINT, SIGTERM, and SIGKILL as needed.

        Args:
            process: The subprocess.Popen object to stop
            process_name: Name of the process for logging
            output_lock: Optional threading lock for output synchronization
            output_attr_name: Optional name of output attribute to append termination message

        Returns:
            bool: True if process was stopped, False if no process was running
        """
        try:
            if process and process.poll() is None:
                self.get_logger().info(f'Stopping {process_name} process (PID: {process.pid})...')

                # Get process group ID
                try:
                    pgid = os.getpgid(process.pid)
                    self.get_logger().info(f'Process group ID: {pgid}')

                    # Send SIGINT (Ctrl+C) to the entire process group
                    os.killpg(pgid, signal.SIGINT)
                    self.get_logger().info('Sent SIGINT to process group')

                    # Wait for process to terminate
                    try:
                        process.wait(timeout=5)
                        self.get_logger().info(f'{process_name} process terminated gracefully')
                    except subprocess.TimeoutExpired:
                        self.get_logger().warn('Process did not terminate with SIGINT, sending SIGTERM')
                        os.killpg(pgid, signal.SIGTERM)
                        try:
                            process.wait(timeout=5)
                            self.get_logger().info(f'{process_name} process terminated with SIGTERM')
                        except subprocess.TimeoutExpired:
                            self.get_logger().warn('Process did not terminate with SIGTERM, sending SIGKILL')
                            os.killpg(pgid, signal.SIGKILL)
                            process.wait(timeout=2)
                            self.get_logger().info(f'{process_name} process killed with SIGKILL')

                except ProcessLookupError:
                    self.get_logger().warn('Process already terminated')
                except Exception as e:
                    self.get_logger().error(f'Error during process termination: {str(e)}')
                    # Fallback: try to terminate the process directly
                    process.terminate()
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()

                # Add termination message to terminal output if requested
                if output_lock and output_attr_name:
                    with output_lock:
                        current_output = getattr(self, output_attr_name)
                        current_output += f'\n[{process_name} process stopped by user]\n'
                        setattr(self, output_attr_name, current_output)

                return True
            else:
                self.get_logger().warn(f'No {process_name} process is running')
                return False
        except Exception as e:
            self.get_logger().error(f'Failed to stop {process_name} process: {str(e)}')
            import traceback
            traceback.print_exc()
            return False

    def _kill_processes_by_pattern(self, patterns):
        """
        Kill processes matching the given patterns.

        Args:
            patterns: List of pattern strings to search for in process command lines
        """
        try:
            # Get all processes
            result = subprocess.run(['ps', 'aux'], capture_output=True, text=True)
            lines = result.stdout.split('\n')

            for line in lines:
                # Check if line matches any pattern
                for pattern in patterns:
                    if pattern in line:
                        parts = line.split()
                        if len(parts) > 1:
                            pid = int(parts[1])
                            self.get_logger().info(f'Killing process matching "{pattern}": PID {pid}')
                            try:
                                os.kill(pid, signal.SIGTERM)
                            except ProcessLookupError:
                                pass
                        break  # Move to next line after finding a match

            time.sleep(0.5)
        except Exception as e:
            self.get_logger().error(f'Error killing processes by pattern: {str(e)}')

    # SLAM Functions
    def set_slam_map1(self, path):
        self.slam_map1 = path
        self.slam_status = f"Map 1 loaded - {path}"
        self.get_logger().info(f'Map 1 set to: {path}')

    def set_slam_map2(self, path):
        self.slam_map2 = path
        self.slam_status = f"Map 2 loaded - {path}"
        self.get_logger().info(f'Map 2 set to: {path}')

    def set_slam_output(self, directory_name):
        """Set output directory name (not full path, just directory name)"""
        # Extract just the directory name if a full path is provided
        if '/' in directory_name:
            directory_name = os.path.basename(directory_name.rstrip('/'))

        self.slam_output = directory_name
        self.slam_status = f"Output directory set to - {directory_name}"
        self.get_logger().info(f'Output directory name set to: {directory_name}')

    def start_slam_mapping(self):
        """Start FAST_LIO mapping"""
        self.get_logger().info('=== Starting FAST_LIO SLAM Mapping ===')

        # Kill any existing SLAM processes first
        self.kill_slam_processes()
        time.sleep(0.5)

        # Clear terminal output
        with self.slam_output_lock:
            self.slam_terminal_output = ""

        # Launch mapping and capture output
        try:
            # Create command with environment setup
            bash_cmd = (
                'source /opt/ros/jazzy/setup.bash && '
                'source /home/kkw/localization_ws/install/setup.bash && '
                'ros2 launch fast_lio mapping.launch.py'
            )

            # Run command and capture output
            cmd = ['bash', '-c', bash_cmd]

            self.get_logger().info('Starting FAST_LIO mapping with output capture')

            # Launch process and capture output
            self.slam_process = subprocess.Popen(
                cmd,
                env=self._ros_env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True
            )

            # Start thread to read output
            output_thread = threading.Thread(
                target=self._read_process_output,
                args=(self.slam_process, self.slam_output_lock, 'slam_terminal_output'),
                daemon=True
            )
            output_thread.start()

            self.get_logger().info('FAST_LIO mapping started with PID: {}'.format(self.slam_process.pid))
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to start FAST_LIO mapping: {str(e)}')
            import traceback
            traceback.print_exc()
            return False

    def stop_slam_mapping(self):
        """Stop SLAM mapping process (like Ctrl+C)"""
        result = self._stop_process(
            self.slam_process,
            'SLAM',
            self.slam_output_lock,
            'slam_terminal_output'
        )
        if result:
            self.slam_process = None
        return result

    def save_slam_map(self, directory):
        """Save SLAM map to specified directory"""
        try:
            if not SAVEMAP_AVAILABLE:
                self.get_logger().error('SaveMap service not available')
                return False, 'SaveMap service not available'

            self.get_logger().info(f'Requesting to save SLAM map to directory: {directory}')

            # Create service client
            client = self.create_client(SaveMap, 'save_trajectory')

            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('save_trajectory service not available')
                return False, 'save_trajectory service not available. Is pose_graph_optimization node running?'

            # Create request
            request = SaveMap.Request()
            request.directory_name = directory

            # Call service
            future = client.call_async(request)

            # Wait for response (with timeout)
            timeout = 30.0  # 30 seconds
            start_time = time.time()
            while not future.done():
                if time.time() - start_time > timeout:
                    self.get_logger().error('Service call timed out')
                    return False, 'Service call timed out after 30 seconds'
                rclpy.spin_once(self, timeout_sec=0.1)

            # Get response
            response = future.result()

            if response.success:
                self.get_logger().info(f'Map saved successfully: {response.message}')
                return True, response.message
            else:
                self.get_logger().error(f'Map save failed: {response.message}')
                return False, response.message

        except Exception as e:
            self.get_logger().error(f'Failed to save map: {str(e)}')
            import traceback
            traceback.print_exc()
            return False, str(e)

    def start_localization_mapping(self):
        """Start Localization mapping process"""
        if self.localization_process and self.localization_process.poll() is None:
            self.get_logger().warn('Localization mapping is already running')
            return True

        # Kill any existing Localization processes first
        self.kill_localization_processes()
        time.sleep(0.5)

        # Clear terminal output
        with self.localization_output_lock:
            self.localization_terminal_output = ""

        # Launch localization and capture output
        try:
            # Create command with environment setup
            bash_cmd = (
                'source /opt/ros/jazzy/setup.bash && '
                'source /home/kkw/localization_ws/install/setup.bash && '
                'ros2 launch fast_lio localization.launch.py'
            )

            # Run command and capture output
            cmd = ['bash', '-c', bash_cmd]

            self.get_logger().info('Starting FAST_LIO localization with output capture')

            # Launch process and capture output
            self.localization_process = subprocess.Popen(
                cmd,
                env=self._ros_env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True
            )

            # Start thread to read output
            output_thread = threading.Thread(
                target=self._read_process_output,
                args=(self.localization_process, self.localization_output_lock, 'localization_terminal_output'),
                daemon=True
            )
            output_thread.start()

            self.get_logger().info('FAST_LIO localization started with PID: {}'.format(self.localization_process.pid))
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to start FAST_LIO localization: {str(e)}')
            import traceback
            traceback.print_exc()
            return False

    def stop_localization_mapping(self):
        """Stop Localization mapping process (like Ctrl+C)"""
        result = self._stop_process(
            self.localization_process,
            'Localization',
            self.localization_output_lock,
            'localization_terminal_output'
        )
        if result:
            self.localization_process = None
        return result

    def kill_localization_processes(self):
        """Kill any running Localization processes"""
        self._kill_processes_by_pattern(['localization.launch.py'])

    def run_slam_optimization(self):
        if not self.slam_map1 or not self.slam_map2:
            self.slam_status = "Error: Please load both Map 1 and Map 2"
            return False

        if not self.slam_output:
            self.slam_status = "Error: Please set output directory"
            return False

        self.slam_status = "Running Multi-Session Optimization..."
        self.get_logger().info('=== Starting Multi-Session SLAM Optimization ===')
        self.get_logger().info(f'Map 1: {self.slam_map1}')
        self.get_logger().info(f'Map 2: {self.slam_map2}')
        self.get_logger().info(f'Output: {self.slam_output}')

        # Kill any existing processes first
        self.kill_slam_processes()
        time.sleep(0.5)

        # Update parameters
        self.update_slam_parameters()
        time.sleep(0.1)  # Wait for file write to complete

        # Launch optimization in new terminal
        try:
            # Create command to run in new terminal
            bash_cmd = (
                'source /opt/ros/jazzy/setup.bash && '
                'source /home/kkw/localization_ws/install/setup.bash && '
                'ros2 launch long_term_mapping lt_mapper.launch.py'
            )

            # Open new terminal and run the command
            cmd = [
                'gnome-terminal',
                '--title=Multi-Session SLAM Optimization',
                '--',
                'bash', '-c',
                f'{bash_cmd}; echo ""; echo "Press Enter to close this window..."; read'
            ]

            self.get_logger().info(f'Opening new terminal for optimization')

            # Launch in new terminal - it's independent from web_gui
            subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True  # Create new process group
            )

            self.get_logger().info('Optimization launched in new terminal')
            self.slam_status = "Optimization launched in new terminal"
            return True
        except Exception as e:
            self.slam_status = f"Error: Failed to start optimization - {str(e)}"
            self.get_logger().error(f'Failed to start optimization: {str(e)}')
            import traceback
            traceback.print_exc()
            return False

    def update_slam_parameters(self):
        param_file = "/home/kkw/localization_ws/src/long_term_mapping/config/params.yaml"

        try:
            with open(param_file, 'r') as f:
                config = yaml.safe_load(f)

            # Update parameters
            if '/**' in config and 'ros__parameters' in config['/**']:
                config['/**']['ros__parameters']['directory1'] = self.slam_map1
                config['/**']['ros__parameters']['directory2'] = self.slam_map2
                config['/**']['ros__parameters']['output_directory'] = self.slam_output

            # Save with proper YAML formatting (default_flow_style=False for readability)
            with open(param_file, 'w') as f:
                yaml.dump(config, f, default_flow_style=False, sort_keys=False)

            self.get_logger().info('SLAM parameters updated successfully')
            self.get_logger().info(f'  directory1: {self.slam_map1}')
            self.get_logger().info(f'  directory2: {self.slam_map2}')
            self.get_logger().info(f'  output_directory: {self.slam_output}')
        except Exception as e:
            self.get_logger().error(f'Failed to update SLAM parameters: {str(e)}')

    def slam_complete_callback(self, msg):
        if msg.data:
            self.slam_status = "Optimization complete!"
            self.get_logger().info('Optimization completed successfully')

    def get_slam_state(self):
        return {
            'map1': self.slam_map1,
            'map2': self.slam_map2,
            'output': self.slam_output,
            'status': self.slam_status
        }

    # Bag Recorder Functions
    def set_recorder_bag_name(self, bag_name):
        """Set the bag name for recording"""
        self.recorder_bag_name = bag_name
        self.get_logger().info(f'Recorder bag name set to: {bag_name}')
        return True

    def get_recorder_topics(self):
        """Get list of current ROS2 topics"""
        try:
            # Use cached ROS environment
            cmd = ['bash', '-c', 'source /opt/ros/jazzy/setup.bash && ros2 topic list']
            result = subprocess.run(
                cmd,
                env=self._ros_env,
                capture_output=True,
                text=True,
                timeout=5
            )

            if result.returncode == 0:
                topics = [t.strip() for t in result.stdout.strip().split('\n') if t.strip()]
                self.get_logger().info(f'Found {len(topics)} ROS2 topics: {topics}')
                return topics
            else:
                self.get_logger().error(f'Failed to get ROS2 topics. Return code: {result.returncode}')
                self.get_logger().error(f'stderr: {result.stderr}')
                return []
        except Exception as e:
            self.get_logger().error(f'Error getting topics: {str(e)}')
            return []

    def record_bag(self, topics):
        """Start or stop bag recording"""
        if self.recorder_recording:
            # Stop recording
            if self.recorder_process:
                self.get_logger().info('Stopping bag recording...')
                self.recorder_process.terminate()
                try:
                    self.recorder_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.recorder_process.kill()
                self.recorder_process = None
            self.recorder_recording = False
            return True
        else:
            # Start recording
            if not self.recorder_bag_name:
                self.get_logger().error('Bag name not set')
                return False

            if not topics or len(topics) == 0:
                self.get_logger().error('No topics selected')
                return False

            # Build ros2 bag record command
            # ros2 bag record will automatically create the directory
            output_dir = f'/home/kkw/dataset/{self.recorder_bag_name}'
            cmd = [
                'bash', '-c',
                f'cd /home/kkw/dataset && '
                f'source /opt/ros/jazzy/setup.bash && '
                f'ros2 bag record -o {self.recorder_bag_name} ' + ' '.join(topics)
            ]

            self.get_logger().info(f'Starting bag recording in: {output_dir}')
            self.get_logger().info(f'Recording topics: {", ".join(topics)}')

            try:
                self.recorder_process = subprocess.Popen(
                    cmd,
                    env=self._ros_env,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    start_new_session=True
                )
                self.recorder_recording = True
                self.get_logger().info('Bag recording started')
                return True
            except Exception as e:
                self.get_logger().error(f'Failed to start recording: {str(e)}')
                return False

    def get_recorder_state(self):
        """Get current recorder state"""
        return {
            'bag_name': self.recorder_bag_name,
            'recording': self.recorder_recording
        }

    # File Player Functions
    def load_player_data(self, path):
        """Load file player data from the specified path"""
        self.player_path = path
        self.player_data_loaded = False

        try:
            # Check if data_stamp.csv exists
            stamp_file = os.path.join(path, 'data_stamp.csv')
            if not os.path.exists(stamp_file):
                self.get_logger().error(f'data_stamp.csv not found in {path}')
                return False

            # Load data stamps
            self.data_stamp = {}
            with open(stamp_file, 'r') as f:
                for line in f:
                    try:
                        parts = line.strip().split(',')
                        if len(parts) == 2:
                            stamp = int(parts[0])
                            data_name = parts[1]
                            self.data_stamp[stamp] = data_name
                    except ValueError as e:
                        self.get_logger().warn(f'Skipping malformed line in data_stamp.csv: {line.strip()} - {str(e)}')
                        continue

            if not self.data_stamp:
                self.get_logger().error('No valid data found in data_stamp.csv')
                return False

            timestamps = sorted(self.data_stamp.keys())
            self.player_initial_stamp = timestamps[0]
            self.player_last_stamp = timestamps[-1]
            self.player_timestamp = self.player_initial_stamp

            self.get_logger().info(f'Loaded {len(self.data_stamp)} data stamps')

            # Load pose data
            pose_file = os.path.join(path, 'pose.csv')
            if os.path.exists(pose_file):
                self.pose_data = {}
                with open(pose_file, 'r') as f:
                    for line in f:
                        try:
                            parts = line.strip().split(',')
                            if len(parts) == 4:
                                stamp = int(parts[0])
                                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                                self.pose_data[stamp] = (x, y, z)
                        except ValueError as e:
                            self.get_logger().warn(f'Skipping malformed line in pose.csv: {line.strip()} - {str(e)}')
                            continue
                self.get_logger().info(f'Loaded {len(self.pose_data)} pose data points')

            # Load IMU data (stamp, q_x, q_y, q_z, q_w, w_x, w_y, w_z, a_x, a_y, a_z)
            imu_file = os.path.join(path, 'imu.csv')
            if os.path.exists(imu_file):
                self.imu_data = {}
                with open(imu_file, 'r') as f:
                    for line in f:
                        try:
                            parts = line.strip().split(',')
                            if len(parts) >= 11:
                                stamp = int(parts[0])
                                # Store IMU data as tuple (q_x, q_y, q_z, q_w, w_x, w_y, w_z, a_x, a_y, a_z)
                                imu = tuple(float(p) for p in parts[1:11])
                                self.imu_data[stamp] = imu
                        except ValueError as e:
                            self.get_logger().warn(f'Skipping malformed line in imu.csv: {line.strip()} - {str(e)}')
                            continue
                self.get_logger().info(f'Loaded {len(self.imu_data)} IMU data points')

            # Load LiDAR file list
            lidar_dir = os.path.join(path, 'LiDAR')
            if os.path.exists(lidar_dir):
                self.livox_file_list = sorted(glob.glob(os.path.join(lidar_dir, '*.bin')))
                self.get_logger().info(f'Found {len(self.livox_file_list)} LiDAR files')
            else:
                self.livox_file_list = []
                self.get_logger().warn('LiDAR directory not found')

            # Load Camera file list (directory name: 'Camera')
            cam_dir = os.path.join(path, 'Camera')
            if os.path.exists(cam_dir):
                # Support multiple image formats
                patterns = ['*.jpg', '*.png', '*.jpeg', '*.JPG', '*.PNG']
                self.cam_file_list = []
                for pattern in patterns:
                    self.cam_file_list.extend(glob.glob(os.path.join(cam_dir, pattern)))
                self.cam_file_list = sorted(self.cam_file_list)
                self.get_logger().info(f'Found {len(self.cam_file_list)} camera images in Camera/')
            else:
                self.cam_file_list = []
                self.get_logger().warn('Camera directory not found (expected: {}/Camera/)'.format(path))

            self.player_data_loaded = True
            return True

        except Exception as e:
            self.get_logger().error(f'Failed to load player data: {str(e)}')
            import traceback
            traceback.print_exc()
            return False

    def load_livox_data(self, stamp):
        """Load LiDAR data from .bin file for given timestamp"""
        if not LIVOX_AVAILABLE or not self.livox_pub:
            return None

        # Check cache first
        if stamp in self.livox_cache:
            return self.livox_cache[stamp]

        # Find matching .bin file
        bin_filename = f"{stamp}.bin"
        bin_path = os.path.join(self.player_path, 'LiDAR', bin_filename)

        if not os.path.exists(bin_path):
            return None

        try:
            # Read binary file
            with open(bin_path, 'rb') as f:
                data = f.read()

            # Parse CustomPoint data
            # Each point: x(float32), y(float32), z(float32), reflectivity(uint8), tag(uint8), line(uint8), offset_time(uint32)
            # Total: 4+4+4+1+1+1+4 = 19 bytes per point
            point_size = 19
            num_points = len(data) // point_size

            msg = CustomMsg()
            msg.header.stamp = Time(nanoseconds=stamp).to_msg()
            msg.header.frame_id = 'livox'
            msg.timebase = stamp
            msg.point_num = num_points
            msg.lidar_id = 0
            msg.rsvd = [0, 0, 0]

            # Parse points
            for i in range(num_points):
                offset = i * point_size
                point_data = data[offset:offset + point_size]

                if len(point_data) < point_size:
                    break

                # Unpack: 3 floats (x,y,z), 3 uint8 (reflectivity, tag, line), 1 uint32 (offset_time)
                x, y, z = struct.unpack('fff', point_data[0:12])
                reflectivity, tag, line = struct.unpack('BBB', point_data[12:15])
                offset_time, = struct.unpack('I', point_data[15:19])

                point = CustomPoint()
                point.x = x
                point.y = y
                point.z = z
                point.reflectivity = reflectivity
                point.tag = tag
                point.line = line
                point.offset_time = offset_time

                msg.points.append(point)

            # Cache the message
            self.livox_cache[stamp] = msg
            return msg

        except Exception as e:
            self.get_logger().error(f'Failed to load LiDAR data for stamp {stamp}: {str(e)}')
            return None

    def load_camera_data(self, stamp):
        """Load camera image for given timestamp"""
        # Check cache first
        if stamp in self.cam_cache:
            return self.cam_cache[stamp]

        # Find matching image file in 'Camera' directory
        # Image files might be named as: stamp.jpg or stamp.png
        img_path = None
        for ext in ['.jpg', '.png', '.jpeg', '.JPG', '.PNG']:
            test_path = os.path.join(self.player_path, 'Camera', f'{stamp}{ext}')
            if os.path.exists(test_path):
                img_path = test_path
                break

        if not img_path:
            return None

        try:
            # Read image using OpenCV
            cv_image = cv2.imread(img_path)
            if cv_image is None:
                return None

            # Convert to ROS Image message
            img_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            img_msg.header.stamp = Time(nanoseconds=stamp).to_msg()
            img_msg.header.frame_id = 'camera'

            # Create CameraInfo message (with default values)
            cam_info_msg = CameraInfo()
            cam_info_msg.header.stamp = img_msg.header.stamp
            cam_info_msg.header.frame_id = 'camera'
            cam_info_msg.height = cv_image.shape[0]
            cam_info_msg.width = cv_image.shape[1]

            # Cache the messages
            self.cam_cache[stamp] = (img_msg, cam_info_msg)
            return (img_msg, cam_info_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to load camera data for stamp {stamp}: {str(e)}')
            return None

    def timer_callback(self):
        """Timer callback to update processed_stamp (matches C++ implementation)"""
        current_time = time.time()

        if self.player_playing and not self.player_paused:
            if self.player_prev_time > 0:
                dt = current_time - self.player_prev_time
                # processed_stamp increases based on real time and play_rate
                self.player_processed_stamp += int(dt * 1e9 * self.player_speed)

        self.player_prev_time = current_time

        if not self.player_playing:
            self.player_processed_stamp = 0

    def bag_timer_callback(self):
        """Timer callback to update bag current time during playback"""
        if self.bag_playing and not self.bag_paused:
            current_real_time = time.time()
            elapsed_time = current_real_time - self.bag_start_real_time
            self.bag_current_time = self.bag_start_offset + elapsed_time

            # Stop when reaching end
            if self.bag_current_time >= self.bag_duration:
                self.bag_current_time = self.bag_duration
                # Note: We don't auto-stop here, let ros2 bag play finish naturally

    def player_play_toggle(self):
        """Toggle play/stop"""
        if not self.player_data_loaded:
            self.get_logger().warn('No data loaded. Please load data first.')
            return False

        self.player_playing = not self.player_playing
        self.player_paused = False

        if self.player_playing:
            self.get_logger().info('Starting playback...')
            self.player_prev_time = time.time()
            if not self.playback_active:
                self.playback_active = True
                self.playback_thread = threading.Thread(target=self.playback_worker, daemon=True)
                self.playback_thread.start()
        else:
            # When stopping (End button pressed), reset to beginning but keep worker active
            self.get_logger().info('Stopping playback - resetting to beginning...')
            self.player_processed_stamp = 0
            self.player_timestamp = self.player_initial_stamp
            self.player_slider_pos = 0
            self.player_paused = False
            # Keep playback_active = True so worker thread stays alive for replay

        return True

    def player_pause_toggle(self):
        """Toggle pause/resume"""
        if self.player_playing:
            self.player_paused = not self.player_paused
            status = "Paused" if self.player_paused else "Resumed"
            self.get_logger().info(f'Playback {status}')
            return True
        return False

    def get_bag_info(self):
        """Get bag file info including topics and duration"""
        if not self.bag_path:
            self.get_logger().warn('No bag file loaded.')
            return {'topics': [], 'duration': 0.0}

        try:
            # Use ros2 bag info to get topic list and duration
            cmd = ['ros2', 'bag', 'info', self.bag_path]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)

            if result.returncode != 0:
                self.get_logger().error(f'Failed to get bag info: {result.stderr}')
                return {'topics': [], 'duration': 0.0}

            # Parse output to extract topics and duration
            topics = []
            duration = 0.0
            lines = result.stdout.split('\n')

            self.get_logger().info('Parsing bag info output:')

            for line in lines:
                self.get_logger().info(f'  Line: {line.strip()}')

                # Parse duration (e.g., "Duration: 123.456s")
                if 'Duration' in line and 's' in line:
                    try:
                        # Extract duration value
                        duration_str = line.split(':')[1].strip()
                        # Remove 's' and convert to float
                        duration = float(duration_str.replace('s', '').strip())
                        self.get_logger().info(f'  Found duration: {duration}s')
                    except:
                        pass

                # Parse topics - looking for lines with "Topic: /topic_name | Count: X | Connection: Y"
                if 'Topic:' in line and '|' in line:
                    try:
                        # Extract topic name between "Topic:" and first "|"
                        parts = line.split('|')
                        topic_part = parts[0]
                        topic_name = topic_part.split('Topic:')[1].strip()
                        topics.append(topic_name)
                        self.get_logger().info(f'  Found topic: {topic_name}')
                    except:
                        pass

            self.bag_topics = topics
            self.bag_duration = duration
            self.get_logger().info(f'Bag info: {len(topics)} topics, duration: {duration}s')
            self.get_logger().info(f'Topics: {topics}')

            return {'topics': topics, 'duration': duration}

        except subprocess.TimeoutExpired:
            self.get_logger().error('Timeout while getting bag info')
            return {'topics': [], 'duration': 0.0}
        except Exception as e:
            self.get_logger().error(f'Failed to get bag info: {str(e)}')
            import traceback
            traceback.print_exc()
            return {'topics': [], 'duration': 0.0}

    def bag_play_toggle(self, selected_topics=None, start_offset=None):
        """Toggle bag play/stop with optional topic selection and start offset"""
        if not self.bag_path:
            self.get_logger().warn('No bag file loaded. Please load a bag file first.')
            return False

        if self.bag_playing:
            # Stop playback
            self.get_logger().info('Stopping bag playback...')
            if self.bag_process:
                self.bag_process.terminate()
                try:
                    self.bag_process.wait(timeout=5)
                except:
                    self.bag_process.kill()
                self.bag_process = None
            self.bag_playing = False
            self.bag_paused = False
            self.bag_current_time = 0.0
            self.bag_start_real_time = 0.0
        else:
            # Start playback using ros2 bag play
            self.get_logger().info(f'Starting bag playback: {self.bag_path}')
            try:
                # Build command with topic selection
                cmd = ['ros2', 'bag', 'play', self.bag_path]

                # Add start offset if specified
                if start_offset is not None and start_offset > 0:
                    cmd.extend(['--start-offset', str(start_offset)])
                    self.bag_start_offset = start_offset
                    self.bag_current_time = start_offset
                    self.get_logger().info(f'Starting from offset: {start_offset}s')
                else:
                    self.bag_start_offset = 0.0
                    self.bag_current_time = 0.0

                # Add topic filter if topics are selected
                if selected_topics and len(selected_topics) > 0:
                    self.bag_selected_topics = selected_topics
                    cmd.append('--topics')
                    cmd.extend(selected_topics)
                    self.get_logger().info(f'Playing selected topics: {selected_topics}')
                else:
                    self.get_logger().info('Playing all topics')

                self.get_logger().info(f'Command: {" ".join(cmd)}')

                # Debug: print environment variables
                self.get_logger().info(f'ROS_DOMAIN_ID: {self._ros_env.get("ROS_DOMAIN_ID", "not set")}')
                self.get_logger().info(f'ROS_DISTRO: {self._ros_env.get("ROS_DISTRO", "not set")}')

                self.bag_process = subprocess.Popen(cmd,
                                                     env=self._ros_env,
                                                     stdout=subprocess.PIPE,
                                                     stderr=subprocess.STDOUT)  # Combine stderr with stdout
                self.bag_playing = True
                self.bag_paused = False
                self.bag_start_real_time = time.time()
                self.get_logger().info('Bag playback started successfully')

                # Start thread to read output
                import threading
                def read_output():
                    for line in iter(self.bag_process.stdout.readline, b''):
                        if line:
                            self.get_logger().info(f'[bag play] {line.decode().strip()}')
                threading.Thread(target=read_output, daemon=True).start()
            except Exception as e:
                self.get_logger().error(f'Failed to start bag playback: {str(e)}')
                return False

        return True

    def bag_pause_toggle(self):
        """Toggle bag playback pause/resume"""
        if not self.bag_playing or not self.bag_process:
            self.get_logger().warn('No bag playback in progress')
            return False

        try:
            if self.bag_paused:
                # Resume playback
                self.get_logger().info('Resuming bag playback...')
                os.kill(self.bag_process.pid, signal.SIGCONT)
                self.bag_paused = False
                # Adjust start time to account for pause duration
                pause_duration = time.time() - self.bag_pause_time
                self.bag_start_real_time += pause_duration
            else:
                # Pause playback
                self.get_logger().info('Pausing bag playback...')
                os.kill(self.bag_process.pid, signal.SIGSTOP)
                self.bag_paused = True
                self.bag_pause_time = time.time()

            return True
        except Exception as e:
            self.get_logger().error(f'Failed to pause/resume bag: {str(e)}')
            return False

    def set_bag_position(self, position_ratio):
        """Set bag playback position (0.0 to 1.0)"""
        if self.bag_duration <= 0:
            return False

        target_time = position_ratio * self.bag_duration

        # If playing, restart from new position
        if self.bag_playing:
            # Stop current playback
            self.bag_play_toggle()
            time.sleep(0.1)
            # Restart from new position
            self.bag_play_toggle(self.bag_selected_topics, target_time)
        else:
            # Just update the position
            self.bag_current_time = target_time
            self.bag_start_offset = target_time

        self.get_logger().info(f'Set bag position to {target_time}s ({position_ratio*100}%)')
        return True

    def get_bag_state(self):
        """Get current bag player state"""
        return {
            'path': self.bag_path,
            'playing': self.bag_playing,
            'paused': self.bag_paused,
            'topics': self.bag_topics,
            'selected_topics': self.bag_selected_topics,
            'duration': self.bag_duration,
            'current_time': self.bag_current_time
        }

    def playback_worker(self):
        """Worker thread for playing back data (matches C++ DataStampThread)"""
        self.get_logger().info('Playback worker started')

        timestamps = sorted(self.data_stamp.keys())

        while self.playback_active:
            time.sleep(0.001)  # 1ms sleep

            if not self.player_playing:
                time.sleep(0.01)
                continue

            # Find current timestamp based on processed_stamp
            target_stamp = self.player_initial_stamp + self.player_processed_stamp

            # Find data to publish
            for stamp in timestamps:
                if stamp > target_stamp:
                    break

                if stamp <= target_stamp and stamp > self.player_timestamp:
                    # This data should be published
                    data_type = self.data_stamp.get(stamp, "")

                    if data_type == "pose" and stamp in self.pose_data:
                        x, y, z = self.pose_data[stamp]
                        msg = PointStamped()
                        msg.header.stamp = Time(nanoseconds=stamp).to_msg()
                        msg.header.frame_id = 'imu_link'
                        msg.point.x = x
                        msg.point.y = y
                        msg.point.z = z
                        self.pose_pub.publish(msg)

                    elif data_type == "imu" and stamp in self.imu_data:
                        imu_values = self.imu_data[stamp]
                        msg = Imu()
                        msg.header.stamp = Time(nanoseconds=stamp).to_msg()
                        msg.header.frame_id = 'imu_link'
                        # IMU data: q_x, q_y, q_z, q_w, w_x, w_y, w_z, a_x, a_y, a_z
                        msg.orientation.x = imu_values[0]
                        msg.orientation.y = imu_values[1]
                        msg.orientation.z = imu_values[2]
                        msg.orientation.w = imu_values[3]
                        msg.angular_velocity.x = imu_values[4]
                        msg.angular_velocity.y = imu_values[5]
                        msg.angular_velocity.z = imu_values[6]
                        msg.linear_acceleration.x = imu_values[7]
                        msg.linear_acceleration.y = imu_values[8]
                        msg.linear_acceleration.z = imu_values[9]
                        self.imu_pub.publish(msg)

                    elif data_type == "livox":
                        # Load and publish LiDAR data
                        livox_msg = self.load_livox_data(stamp)
                        if livox_msg and self.livox_pub:
                            self.livox_pub.publish(livox_msg)
                        else:
                            self.get_logger().warn(f'Failed to load LiDAR data for stamp {stamp}', throttle_duration_sec=5.0)

                    elif data_type == "cam":
                        # Load and publish camera data
                        cam_data = self.load_camera_data(stamp)
                        if cam_data:
                            img_msg, cam_info_msg = cam_data
                            self.cam_pub.publish(img_msg)
                            self.cam_info_pub.publish(cam_info_msg)
                        else:
                            self.get_logger().warn(f'Failed to load camera data for stamp {stamp}', throttle_duration_sec=5.0)

                    # Publish clock every 10ms
                    clock_msg = Clock()
                    clock_msg.clock = Time(nanoseconds=stamp).to_msg()
                    self.clock_pub.publish(clock_msg)

                    self.player_timestamp = stamp

            # Update slider position
            if self.player_last_stamp > self.player_initial_stamp:
                progress = (target_stamp - self.player_initial_stamp) / (self.player_last_stamp - self.player_initial_stamp)
                self.player_slider_pos = int(progress * 10000)

            # Check if reached end
            if target_stamp >= self.player_last_stamp:
                if self.player_loop:
                    self.get_logger().info('Looping playback...')
                    self.player_processed_stamp = 0
                    self.player_timestamp = self.player_initial_stamp
                else:
                    self.get_logger().info('Playback finished')
                    self.player_playing = False
                    self.player_processed_stamp = 0

        self.get_logger().info('Playback worker stopped')

    def reset_player_position(self, position):
        """Reset playback position (0-10000)"""
        if not self.player_data_loaded:
            return

        ratio = position / 10000.0
        total_duration = self.player_last_stamp - self.player_initial_stamp

        # Update processed_stamp to match the slider position
        self.player_processed_stamp = int(ratio * total_duration)

        # Update timestamp to the target position
        target_stamp = int(self.player_initial_stamp + self.player_processed_stamp)
        self.player_timestamp = target_stamp
        self.player_slider_pos = position

        # Reset timer for smooth playback after seeking
        self.player_prev_time = time.time()

        self.get_logger().info(f'Reset position to {position} (stamp: {target_stamp}, processed: {self.player_processed_stamp}ns)')

    def save_rosbag(self):
        """Save loaded data to rosbag2 format"""
        if not self.player_data_loaded:
            self.get_logger().error('No data loaded. Please load data first.')
            return False

        try:
            bag_path = os.path.join(self.player_path, "output")
            self.get_logger().info(f'Starting rosbag conversion to: {bag_path}')

            # Create writer
            writer = rosbag2_py.SequentialWriter()

            storage_options = rosbag2_py.StorageOptions(
                uri=bag_path,
                storage_id='sqlite3'
            )

            converter_options = rosbag2_py.ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )

            writer.open(storage_options, converter_options)

            # Create topics with correct TopicMetadata format (id is required)
            from rosbag2_py import TopicMetadata

            pose_topic = TopicMetadata(
                id=0,
                name='/pose/position',
                type='geometry_msgs/msg/PointStamped',
                serialization_format='cdr'
            )
            writer.create_topic(pose_topic)

            imu_topic = TopicMetadata(
                id=1,
                name='/imu',
                type='sensor_msgs/msg/Imu',
                serialization_format='cdr'
            )
            writer.create_topic(imu_topic)

            # Create LiDAR topic if available
            topic_id = 2
            if LIVOX_AVAILABLE and len(self.livox_file_list) > 0:
                livox_topic = TopicMetadata(
                    id=topic_id,
                    name='/livox/lidar',
                    type='livox_ros_driver2/msg/CustomMsg',
                    serialization_format='cdr'
                )
                writer.create_topic(livox_topic)
                topic_id += 1

            # Create Camera topics if available
            if len(self.cam_file_list) > 0:
                cam_topic = TopicMetadata(
                    id=topic_id,
                    name='/camera/color/image',
                    type='sensor_msgs/msg/Image',
                    serialization_format='cdr'
                )
                writer.create_topic(cam_topic)
                topic_id += 1

                cam_info_topic = TopicMetadata(
                    id=topic_id,
                    name='/camera/color/camera_info',
                    type='sensor_msgs/msg/CameraInfo',
                    serialization_format='cdr'
                )
                writer.create_topic(cam_info_topic)
                topic_id += 1

            # Write pose data
            self.get_logger().info(f'Writing {len(self.pose_data)} pose messages...')
            count = 0
            for stamp, (x, y, z) in sorted(self.pose_data.items()):
                msg = PointStamped()
                msg.header.stamp = Time(nanoseconds=stamp).to_msg()
                msg.header.frame_id = 'imu_link'
                msg.point.x = x
                msg.point.y = y
                msg.point.z = z

                writer.write(
                    '/pose/position',
                    serialize_message(msg),
                    stamp
                )
                count += 1
                if count % 100 == 0:
                    self.get_logger().info(f'Pose: {count}/{len(self.pose_data)}')

            # Write IMU data
            self.get_logger().info(f'Writing {len(self.imu_data)} IMU messages...')
            count = 0
            for stamp, imu_values in sorted(self.imu_data.items()):
                msg = Imu()
                msg.header.stamp = Time(nanoseconds=stamp).to_msg()
                msg.header.frame_id = 'imu_link'
                msg.orientation.x = imu_values[0]
                msg.orientation.y = imu_values[1]
                msg.orientation.z = imu_values[2]
                msg.orientation.w = imu_values[3]
                msg.angular_velocity.x = imu_values[4]
                msg.angular_velocity.y = imu_values[5]
                msg.angular_velocity.z = imu_values[6]
                msg.linear_acceleration.x = imu_values[7]
                msg.linear_acceleration.y = imu_values[8]
                msg.linear_acceleration.z = imu_values[9]

                writer.write(
                    '/imu',
                    serialize_message(msg),
                    stamp
                )
                count += 1
                if count % 100 == 0:
                    self.get_logger().info(f'IMU: {count}/{len(self.imu_data)}')

            # Write LiDAR data
            if LIVOX_AVAILABLE and len(self.livox_file_list) > 0:
                # Get LiDAR stamps from data_stamp
                livox_stamps = [stamp for stamp, dtype in self.data_stamp.items() if dtype == "livox"]
                self.get_logger().info(f'Writing {len(livox_stamps)} LiDAR messages...')
                count = 0
                for stamp in sorted(livox_stamps):
                    livox_msg = self.load_livox_data(stamp)
                    if livox_msg:
                        writer.write(
                            '/livox/lidar',
                            serialize_message(livox_msg),
                            stamp
                        )
                        count += 1
                        if count % 10 == 0:
                            self.get_logger().info(f'LiDAR: {count}/{len(livox_stamps)}')

            # Write Camera data
            if len(self.cam_file_list) > 0:
                # Get camera stamps from data_stamp
                cam_stamps = [stamp for stamp, dtype in self.data_stamp.items() if dtype == "cam"]
                self.get_logger().info(f'Writing {len(cam_stamps)} camera messages...')
                count = 0
                for stamp in sorted(cam_stamps):
                    cam_data = self.load_camera_data(stamp)
                    if cam_data:
                        img_msg, cam_info_msg = cam_data
                        writer.write(
                            '/camera/color/image',
                            serialize_message(img_msg),
                            stamp
                        )
                        writer.write(
                            '/camera/color/camera_info',
                            serialize_message(cam_info_msg),
                            stamp
                        )
                        count += 1
                        if count % 10 == 0:
                            self.get_logger().info(f'Camera: {count}/{len(cam_stamps)}')

            del writer
            self.get_logger().info('Rosbag conversion complete!')
            return True

        except Exception as e:
            self.get_logger().error(f'Failed to save rosbag: {str(e)}')
            import traceback
            traceback.print_exc()
            return False

    def file_player_start_callback(self, msg):
        if msg.data and not self.player_playing:
            self.player_play_toggle()

    def file_player_stop_callback(self, msg):
        if msg.data and self.player_playing:
            self.player_play_toggle()

    def get_player_state(self):
        return {
            'path': self.player_path,
            'playing': self.player_playing,
            'paused': self.player_paused,
            'loop': self.player_loop,
            'skip_stop': self.player_skip_stop,
            'auto_start': self.player_auto_start,
            'speed': self.player_speed,
            'timestamp': self.player_timestamp,
            'slider_pos': self.player_slider_pos,
            'data_loaded': self.player_data_loaded
        }

    def kill_slam_processes(self):
        """Kill all SLAM-related processes and bag playback"""
        try:
            # Kill bag playback process if running
            if self.bag_process and self.bag_process.poll() is None:
                self.get_logger().info(f'Terminating bag playback process PID: {self.bag_process.pid}')
                self.bag_process.terminate()
                try:
                    self.bag_process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.get_logger().info('Bag process did not terminate, killing...')
                    self.bag_process.kill()
                self.bag_process = None
                self.bag_playing = False

            # First try to terminate the subprocess gracefully
            if self.slam_process and self.slam_process.poll() is None:
                self.get_logger().info(f'Terminating SLAM process PID: {self.slam_process.pid}')
                self.slam_process.terminate()
                try:
                    self.slam_process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.get_logger().info('Process did not terminate, killing...')
                    self.slam_process.kill()
                self.slam_process = None

            # Then kill any remaining related processes
            subprocess.run(['pkill', '-9', '-f', 'LTmapping'], check=False,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run(['pkill', '-9', '-f', 'rviz2'], check=False,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run(['pkill', '-9', '-f', 'lt_mapper.launch.py'], check=False,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            self.get_logger().info('SLAM processes killed')
            self.slam_status = "Ready"
        except Exception as e:
            self.get_logger().error(f'Error killing processes: {str(e)}')


# File browser functions
def browse_directory(start_path="/home"):
    """Get list of directories and files in the given path"""
    try:
        entries = []
        path = PathLib(start_path)

        # Add parent directory option
        if path.parent != path:
            entries.append({
                'name': '..',
                'path': str(path.parent),
                'is_dir': True,
                'is_file': False
            })

        # List directories and files
        if path.exists() and path.is_dir():
            # First add directories
            for entry in sorted(path.iterdir()):
                if entry.is_dir():
                    entries.append({
                        'name': entry.name,
                        'path': str(entry),
                        'is_dir': True,
                        'is_file': False
                    })

            # Then add files
            for entry in sorted(path.iterdir()):
                if entry.is_file():
                    entries.append({
                        'name': entry.name,
                        'path': str(entry),
                        'is_dir': False,
                        'is_file': True
                    })

        return {
            'success': True,
            'current_path': str(path),
            'entries': entries
        }
    except Exception as e:
        return {
            'success': False,
            'error': str(e),
            'current_path': start_path,
            'entries': []
        }


class WebRequestHandler(SimpleHTTPRequestHandler):
    node = None
    web_dir = None

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=WebRequestHandler.web_dir, **kwargs)

    def do_GET(self):
        parsed_path = urlparse(self.path)

        if parsed_path.path == '/api/slam/state':
            self.send_json_response(self.node.get_slam_state())
        elif parsed_path.path == '/api/slam/get_terminal_output':
            with self.node.slam_output_lock:
                output = self.node.slam_terminal_output
            self.send_json_response({'success': True, 'output': output})
        elif parsed_path.path == '/api/player/state':
            self.send_json_response(self.node.get_player_state())
        elif parsed_path.path == '/api/bag/state':
            self.send_json_response(self.node.get_bag_state())
        elif parsed_path.path == '/api/bag/get_info':
            info = self.node.get_bag_info()
            self.send_json_response({'success': True, 'topics': info['topics'], 'duration': info['duration']})
        elif parsed_path.path == '/api/recorder/state':
            self.send_json_response(self.node.get_recorder_state())
        elif parsed_path.path == '/api/recorder/get_topics':
            topics = self.node.get_recorder_topics()
            self.send_json_response({'success': True, 'topics': topics})
        elif parsed_path.path.startswith('/api/browse'):
            # Parse query parameters
            query = parse_qs(parsed_path.query)
            path = query.get('path', ['/home'])[0]
            result = browse_directory(path)
            self.send_json_response(result)
        elif parsed_path.path == '/api/ping':
            # Simple ping endpoint for latency measurement
            self.send_json_response({'success': True, 'timestamp': time.time()})
        else:
            # Serve static files
            if parsed_path.path == '/':
                self.path = '/index.html'
            super().do_GET()

    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)
        data = json.loads(post_data.decode('utf-8'))

        parsed_path = urlparse(self.path)
        response = {'success': False}

        # SLAM API endpoints
        if parsed_path.path == '/api/slam/set_map1':
            self.node.set_slam_map1(data.get('path', ''))
            response = {'success': True, 'status': self.node.slam_status}
        elif parsed_path.path == '/api/slam/set_map2':
            self.node.set_slam_map2(data.get('path', ''))
            response = {'success': True, 'status': self.node.slam_status}
        elif parsed_path.path == '/api/slam/set_output':
            self.node.set_slam_output(data.get('path', ''))
            response = {'success': True, 'status': self.node.slam_status}
        elif parsed_path.path == '/api/slam/optimize':
            success = self.node.run_slam_optimization()
            response = {'success': success, 'status': self.node.slam_status}
        elif parsed_path.path == '/api/slam/start_mapping':
            success = self.node.start_slam_mapping()
            response = {'success': success, 'message': 'SLAM mapping started' if success else 'Failed to start SLAM mapping'}
        elif parsed_path.path == '/api/slam/stop_mapping':
            success = self.node.stop_slam_mapping()
            response = {'success': success, 'message': 'SLAM mapping stopped' if success else 'Failed to stop SLAM mapping'}
        elif parsed_path.path == '/api/slam/save_map':
            directory = data.get('directory', 'map')
            success, message = self.node.save_slam_map(directory)
            response = {'success': success, 'message': message}

        # Localization API endpoints
        elif parsed_path.path == '/api/localization/start_mapping':
            success = self.node.start_localization_mapping()
            response = {'success': success, 'message': 'Localization mapping started' if success else 'Failed to start Localization mapping'}
        elif parsed_path.path == '/api/localization/stop_mapping':
            success = self.node.stop_localization_mapping()
            response = {'success': success, 'message': 'Localization mapping stopped' if success else 'Failed to stop Localization mapping'}
        elif parsed_path.path == '/api/localization/get_terminal_output':
            with self.node.localization_output_lock:
                output = self.node.localization_terminal_output
            response = {'success': True, 'output': output}

        # Bag Player API endpoints
        elif parsed_path.path == '/api/bag/load':
            path = data.get('path', '')
            self.node.bag_path = path
            # Automatically get bag info when loading
            info = self.node.get_bag_info()
            response = {
                'success': True,
                'message': 'Bag path set',
                'path': path,
                'topics': info['topics'],
                'duration': info['duration']
            }
        elif parsed_path.path == '/api/bag/play':
            selected_topics = data.get('topics', [])
            start_offset = data.get('start_offset', None)
            success = self.node.bag_play_toggle(selected_topics, start_offset)
            response = {'success': success, 'playing': self.node.bag_playing}
        elif parsed_path.path == '/api/bag/pause':
            success = self.node.bag_pause_toggle()
            response = {'success': success, 'paused': self.node.bag_paused}
        elif parsed_path.path == '/api/bag/set_position':
            position = data.get('position', 0)  # 0-10000
            position_ratio = position / 10000.0
            success = self.node.set_bag_position(position_ratio)
            response = {'success': success}

        # Bag Recorder API endpoints
        elif parsed_path.path == '/api/recorder/set_bag_name':
            bag_name = data.get('bag_name', '')
            success = self.node.set_recorder_bag_name(bag_name)
            response = {'success': success}
        elif parsed_path.path == '/api/recorder/record':
            topics = data.get('topics', [])
            success = self.node.record_bag(topics)
            response = {'success': success, 'recording': self.node.recorder_recording}

        # SLAM Config API endpoints
        elif parsed_path.path == '/api/slam/load_config_file':
            config_path = data.get('path', '')
            try:
                with open(config_path, 'r') as f:
                    config_data = yaml.safe_load(f)

                # Extract parameters from ROS2 yaml format
                if '/**' in config_data and 'ros__parameters' in config_data['/**']:
                    params = config_data['/**']['ros__parameters']
                    response = {'success': True, 'config': params}
                else:
                    # If not in ROS2 format, return as is
                    response = {'success': True, 'config': config_data}

                self.node.get_logger().info(f'Loaded config from: {config_path}')
            except Exception as e:
                self.node.get_logger().error(f'Failed to load config: {str(e)}')
                response = {'success': False, 'message': str(e)}

        elif parsed_path.path == '/api/slam/save_config_file':
            config_path = data.get('path', '')
            config_params = data.get('config', {})
            try:
                if RUAMEL_AVAILABLE:
                    # Use ruamel.yaml to preserve comments and formatting
                    from ruamel.yaml.comments import CommentedMap, CommentedSeq
                    from ruamel.yaml.scalarstring import DoubleQuotedScalarString

                    yaml_handler = YAML()
                    yaml_handler.preserve_quotes = True
                    yaml_handler.default_flow_style = False  # Ensure block style
                    yaml_handler.width = 1000
                    yaml_handler.indent(mapping=4, sequence=4, offset=0)

                    # Helper function to convert dict to CommentedMap recursively
                    def convert_to_commented_map(obj, original=None):
                        if isinstance(obj, dict):
                            cm = CommentedMap()
                            for key, value in obj.items():
                                orig_value = original.get(key) if isinstance(original, dict) else None
                                cm[key] = convert_to_commented_map(value, orig_value)
                            return cm
                        elif isinstance(obj, list):
                            # Convert all lists to flow style (single line with brackets)
                            # Preserve float types in list elements
                            converted_list = []
                            for i, item in enumerate(obj):
                                orig_item = original[i] if isinstance(original, list) and i < len(original) else None
                                if isinstance(orig_item, float) and isinstance(item, (int, float)):
                                    converted_list.append(float(item))
                                else:
                                    converted_list.append(convert_to_commented_map(item, orig_item))
                            cs = CommentedSeq(converted_list)
                            cs.fa.set_flow_style()
                            return cs
                        elif isinstance(obj, str):
                            # Wrap strings in double quotes
                            return DoubleQuotedScalarString(obj)
                        elif isinstance(original, float) and isinstance(obj, (int, float)):
                            # Preserve float type
                            return float(obj)
                        else:
                            return obj

                    # Read existing config file
                    with open(config_path, 'r') as f:
                        config_data = yaml_handler.load(f)

                    # Update parameters in ROS2 yaml format
                    if '/**' in config_data and 'ros__parameters' in config_data['/**']:
                        ros_params = config_data['/**']['ros__parameters']

                        # Helper function to preserve numeric types (float vs int)
                        def preserve_numeric_type(old_value, new_value):
                            # If old value was float, keep new value as float even if it's whole number
                            if isinstance(old_value, float) and isinstance(new_value, (int, float)):
                                return float(new_value)
                            # For lists, recursively preserve types
                            elif isinstance(old_value, list) and isinstance(new_value, list):
                                return [preserve_numeric_type(old_value[i] if i < len(old_value) else new_value[i], new_value[i])
                                        for i in range(len(new_value))]
                            return new_value

                        # Update all parameters
                        for key, value in config_params.items():
                            # Get old value to check its type
                            old_value = ros_params.get(key)

                            # Convert nested dictionaries to CommentedMap to preserve block style
                            if isinstance(value, dict):
                                ros_params[key] = convert_to_commented_map(value, old_value)
                            # Preserve numeric types (especially float)
                            elif old_value is not None:
                                ros_params[key] = preserve_numeric_type(old_value, value)
                            else:
                                ros_params[key] = value

                        # Format matrix parameters (9 elements = 3x3 matrix)
                        matrix_params = ['extrinsic_R', 'extrinsic_g2o_R']
                        for param in matrix_params:
                            if param in ros_params and isinstance(ros_params[param], list) and len(ros_params[param]) == 9:
                                # Create flow style list with custom formatting
                                formatted_list = CommentedSeq(ros_params[param])
                                formatted_list.fa.set_flow_style()
                                ros_params[param] = formatted_list

                        # Format vector parameters (3 elements)
                        vector_params = ['extrinsic_T', 'extrinsic_g2o_T']
                        for param in vector_params:
                            if param in ros_params and isinstance(ros_params[param], list):
                                formatted_list = CommentedSeq(ros_params[param])
                                formatted_list.fa.set_flow_style()
                                ros_params[param] = formatted_list
                    else:
                        config_data = convert_to_commented_map(config_params)

                    # Save with ruamel.yaml
                    with open(config_path, 'w') as f:
                        yaml_handler.dump(config_data, f)

                    # Post-process: Fix 3x3 matrix formatting
                    with open(config_path, 'r') as f:
                        content = f.read()

                    # Format 9-element arrays as 3x3 matrices
                    import re

                    # Find extrinsic_R and extrinsic_g2o_R patterns
                    def format_matrix(match):
                        indent = match.group(1)
                        param_name = match.group(2)
                        values = match.group(3)

                        # Parse values
                        nums = [v.strip() for v in values.split(',')]
                        if len(nums) == 9:
                            # Format as 3x3 matrix
                            line1 = f"{indent}{param_name}: [{nums[0]}, {nums[1]}, {nums[2]},"
                            line2 = f"{indent}            {nums[3]}, {nums[4]}, {nums[5]},"
                            line3 = f"{indent}            {nums[6]}, {nums[7]}, {nums[8]}]"
                            return f"{line1}\n{line2}\n{line3}"
                        return match.group(0)

                    # Replace 9-element arrays
                    content = re.sub(
                        r'^(\s*)(extrinsic_R|extrinsic_g2o_R):\s*\[([\d\.,\s\-]+)\]',
                        format_matrix,
                        content,
                        flags=re.MULTILINE
                    )

                    # Write back
                    with open(config_path, 'w') as f:
                        f.write(content)

                else:
                    # Fallback to regular yaml (no comment preservation)
                    with open(config_path, 'r') as f:
                        config_data = yaml.safe_load(f)

                    if '/**' in config_data and 'ros__parameters' in config_data['/**']:
                        config_data['/**']['ros__parameters'] = config_params
                    else:
                        config_data = config_params

                    class IndentDumper(yaml.Dumper):
                        def increase_indent(self, flow=False, indentless=False):
                            return super(IndentDumper, self).increase_indent(flow, False)

                    with open(config_path, 'w') as f:
                        yaml.dump(
                            config_data,
                            f,
                            Dumper=IndentDumper,
                            default_flow_style=False,
                            sort_keys=False,
                            indent=4,
                            width=1000,
                            allow_unicode=True
                        )

                self.node.get_logger().info(f'Saved config to: {config_path}')
                response = {'success': True, 'message': 'Config saved successfully'}
            except Exception as e:
                self.node.get_logger().error(f'Failed to save config: {str(e)}')
                import traceback
                traceback.print_exc()
                response = {'success': False, 'message': str(e)}

        # File Player API endpoints
        elif parsed_path.path == '/api/player/load_data':
            path = data.get('path', '')
            success = self.node.load_player_data(path)
            response = {'success': success, 'message': 'Data loaded' if success else 'Failed to load data'}
        elif parsed_path.path == '/api/player/play':
            success = self.node.player_play_toggle()
            response = {'success': success, 'playing': self.node.player_playing}
        elif parsed_path.path == '/api/player/pause':
            success = self.node.player_pause_toggle()
            response = {'success': success, 'paused': self.node.player_paused}
        elif parsed_path.path == '/api/player/save_bag':
            success = self.node.save_rosbag()
            response = {'success': success, 'message': 'Bag saved successfully' if success else 'Failed to save bag'}
        elif parsed_path.path == '/api/player/set_speed':
            self.node.player_speed = data.get('speed', 1.0)
            response = {'success': True}
        elif parsed_path.path == '/api/player/set_loop':
            self.node.player_loop = data.get('loop', False)
            response = {'success': True}
        elif parsed_path.path == '/api/player/set_skip_stop':
            self.node.player_skip_stop = data.get('skip_stop', False)
            response = {'success': True}
        elif parsed_path.path == '/api/player/set_auto_start':
            self.node.player_auto_start = data.get('auto_start', False)
            response = {'success': True}
        elif parsed_path.path == '/api/player/set_slider':
            position = data.get('position', 0)
            self.node.reset_player_position(position)
            response = {'success': True}

        self.send_json_response(response)

    def send_json_response(self, data):
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode('utf-8'))

    def log_message(self, format, *args):
        pass


def get_local_ip():
    """Get the local IP address"""
    try:
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "localhost"


def run_web_server(node, port=8080):
    WebRequestHandler.node = node

    # Determine web directory
    web_dir = None
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory('web_gui')
        web_dir = os.path.join(share_dir, 'web')
        node.get_logger().info(f'Using web directory: {web_dir}')
    except Exception as e:
        # Fallback for development
        web_dir = os.path.join(os.path.dirname(__file__), '..', 'web')
        web_dir = os.path.abspath(web_dir)
        node.get_logger().info(f'Using fallback web directory: {web_dir}')

    # Check if web directory exists
    if not os.path.exists(web_dir):
        node.get_logger().error(f'Web directory not found: {web_dir}')
        return

    # Check if index.html exists
    index_path = os.path.join(web_dir, 'index.html')
    if not os.path.exists(index_path):
        node.get_logger().error(f'index.html not found: {index_path}')
        return

    WebRequestHandler.web_dir = web_dir
    server = HTTPServer(('0.0.0.0', port), WebRequestHandler)

    # Get local IP for network access
    local_ip = get_local_ip()

    node.get_logger().info(f'======================================')
    node.get_logger().info(f'Web server started on port {port}')
    node.get_logger().info(f'Local access:   http://localhost:{port}')
    node.get_logger().info(f'Network access: http://{local_ip}:{port}')
    node.get_logger().info(f'======================================')

    server.serve_forever()


def main(args=None):
    rclpy.init(args=args)

    node = WebGUINode()

    # Start web server in a separate thread
    web_thread = threading.Thread(target=run_web_server, args=(node, 8080), daemon=True)
    web_thread.start()

    local_ip = get_local_ip()
    node.get_logger().info(f'Web GUI is running with full ROS2 integration.')
    node.get_logger().info(f'Open http://localhost:8080 or http://{local_ip}:8080 in your browser.')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.kill_slam_processes()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

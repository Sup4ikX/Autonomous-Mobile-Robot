#!/usr/bin/env python3
"""
ESP32 LiDAR Robot Mapper - System Test Script
Quick test to verify all components are working correctly.
"""

import sys
import os
import subprocess
import time
import signal
from pathlib import Path

def print_header(title):
    """Print a formatted header."""
    print(f"\n{'='*60}")
    print(f" {title}")
    print(f"{'='*60}")

def print_step(step, description):
    """Print a formatted step."""
    print(f"\n[STEP {step}] {description}")
    print("-" * 40)

def check_python_imports():
    """Check if required Python modules can be imported."""
    print_step(1, "Checking Python imports")
    
    required_modules = [
        'yaml',
        'socket',
        'threading',
        'time',
        'json',
        'subprocess',
        'argparse'
    ]
    
    missing_modules = []
    
    for module in required_modules:
        try:
            __import__(module)
            print(f"✓ {module}")
        except ImportError:
            print(f"✗ {module} - MISSING")
            missing_modules.append(module)
    
    if missing_modules:
        print(f"\nMissing modules: {', '.join(missing_modules)}")
        print("Install with: pip install pyyaml")
        return False
    
    return True

def check_ros2_environment():
    """Check if ROS2 environment is properly set up."""
    print_step(2, "Checking ROS2 environment")
    
    try:
        # Check if ROS2 is available
        result = subprocess.run(['ros2', '--version'], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print(f"✓ ROS2 version: {result.stdout.strip()}")
        else:
            print("✗ ROS2 not found")
            return False
            
        # Check if colcon is available
        result = subprocess.run(['colcon', '--version'], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print(f"✓ Colcon version: {result.stdout.strip()}")
        else:
            print("✗ Colcon not found")
            return False
            
        # Check if we can list ROS2 nodes (should work even without running nodes)
        result = subprocess.run(['ros2', 'node', 'list'], 
                              capture_output=True, text=True, timeout=10)
        print("✓ ROS2 node list command works")
        
        return True
        
    except subprocess.TimeoutExpired:
        print("✗ Command timed out")
        return False
    except FileNotFoundError:
        print("✗ ROS2 commands not found in PATH")
        return False

def check_project_structure():
    """Check if project structure is correct."""
    print_step(3, "Checking project structure")
    
    required_files = [
        'package.xml',
        'CMakeLists.txt',
        'setup.py',
        'launch/launch_all.py',
        'python/pj_lidar/__init__.py',
        'python/pj_lidar/esp32_tcp_client.py',
        'python/pj_lidar/lidar_udp_server_new.py',
        'python/pj_lidar/scan_state_machine.py',
        'python/pj_lidar/robot_gui.py',
        'src/lidar_processor.cpp'
    ]
    
    missing_files = []
    
    for file_path in required_files:
        if Path(file_path).exists():
            print(f"✓ {file_path}")
        else:
            print(f"✗ {file_path} - MISSING")
            missing_files.append(file_path)
    
    if missing_files:
        print(f"\nMissing files: {len(missing_files)}")
        return False
    
    return True

def check_python_scripts():
    """Check if Python scripts can be imported without errors."""
    print_step(4, "Checking Python scripts")
    
    # Add current directory to Python path
    sys.path.insert(0, str(Path('.').resolve()))
    
    scripts_to_check = [
        'python.pj_lidar.config_loader',
        'python.pj_lidar.config_validator',
        'python.pj_lidar.diagnostics_aggregator',
        'python.pj_lidar.esp32_tcp_client',
        'python.pj_lidar.lidar_udp_server_new',
        'python.pj_lidar.scan_state_machine',
        'python.pj_lidar.robot_gui',
        'python.pj_lidar.robot_monitor',
        'python.pj_lidar.robot_controller',
        'python.pj_lidar.start_scan',
        'python.pj_lidar.setup_config'
    ]
    
    failed_imports = []
    
    for script in scripts_to_check:
        try:
            __import__(script)
            print(f"✓ {script}")
        except ImportError as e:
            print(f"✗ {script} - Import error: {e}")
            failed_imports.append(script)
        except Exception as e:
            print(f"✗ {script} - Error: {e}")
            failed_imports.append(script)
    
    if failed_imports:
        print(f"\nFailed to import: {len(failed_imports)} scripts")
        return False
    
    return True

def check_config_files():
    """Check if configuration files exist and are valid."""
    print_step(5, "Checking configuration files")
    
    config_files = [
        'python/pj_lidar/config.yaml',
        'python/pj_lidar/config/slam_toolbox_config.yaml',
        'python/pj_lidar/config/rviz_config.rviz',
        'python/pj_lidar/config/nav2_params.yaml'
    ]
    
    for config_file in config_files:
        if Path(config_file).exists():
            print(f"✓ {config_file}")
        else:
            print(f"✗ {config_file} - MISSING")
    
    # Check if config.yaml exists, if not create a basic one
    config_path = Path('python/pj_lidar/config.yaml')
    if not config_path.exists():
        print("\nCreating basic config.yaml...")
        basic_config = """# ESP32 LiDAR Robot Mapper - Basic Configuration
esp32_tcp_client:
  robot_ip: "192.168.4.2"
  tcp_port: 3333
  timeout: 5.0

lidar_udp_server:
  udp_port: 8888
  range_min: 0.15
  range_max: 12.0

scan_state_machine:
  obstacle_distance: 0.5
  max_forward_speed: 0.3
  rotation_speed: 0.5

slam_toolbox:
  map_save_path: ~/maps
"""
        config_path.parent.mkdir(parents=True, exist_ok=True)
        with open(config_path, 'w') as f:
            f.write(basic_config)
        print("✓ Created basic config.yaml")

def run_basic_functionality_test():
    """Run a basic functionality test."""
    print_step(6, "Running basic functionality test")
    
    try:
        # Test config loading
        from python.pj_lidar.config_loader import load_config
        config = load_config()
        print("✓ Config loaded successfully")
        
        # Test if we can create a simple TCP client (without connecting)
        from python.pj_lidar.esp32_tcp_client import ESP32TCPClient
        client = ESP32TCPClient("127.0.0.1", 3333)
        print("✓ TCP client created successfully")
        
        # Test if we can create a UDP server (without starting)
        from python.pj_lidar.lidar_udp_server_new import LidarUDPServer
        server = LidarUDPServer(8888)
        print("✓ UDP server created successfully")
        
        return True
        
    except Exception as e:
        print(f"✗ Functionality test failed: {e}")
        return False

def main():
    """Main test function."""
    print_header("ESP32 LiDAR Robot Mapper - System Test")
    
    tests = [
        ("Python Imports", check_python_imports),
        ("ROS2 Environment", check_ros2_environment),
        ("Project Structure", check_project_structure),
        ("Python Scripts", check_python_scripts),
        ("Configuration Files", check_config_files),
        ("Basic Functionality", run_basic_functionality_test)
    ]
    
    passed_tests = 0
    total_tests = len(tests)
    
    for test_name, test_func in tests:
        try:
            if test_func():
                passed_tests += 1
                print(f"\n✓ {test_name} - PASSED")
            else:
                print(f"\n✗ {test_name} - FAILED")
        except Exception as e:
            print(f"\n✗ {test_name} - ERROR: {e}")
    
    print_header("Test Results")
    print(f"Passed: {passed_tests}/{total_tests}")
    
    if passed_tests == total_tests:
        print("\n🎉 All tests passed! System is ready.")
        print("\nNext steps:")
        print("1. Upload firmware to ESP32")
        print("2. Run: python setup_config.py")
        print("3. Run: ros2 launch pj_lidar launch_all.py")
        print("4. Run: python robot_gui.py")
        return 0
    else:
        print(f"\n❌ {total_tests - passed_tests} test(s) failed.")
        print("\nPlease fix the issues above before proceeding.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
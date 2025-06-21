#!/usr/bin/env python3
"""
Modernized Robot Controller with ROS2 and MyCobot Integration
"""

import os
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple, Union

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import tkinter as tk
from tkinter import ttk, messagebox

# Check pymycobot version before importing
try:
    import pymycobot
    from packaging import version
    
    MIN_REQUIRED_VERSION = '3.6.1'
    CURRENT_VERSION = pymycobot.__version__
    
    print(f'Current pymycobot library version: {CURRENT_VERSION}')
    
    if version.parse(CURRENT_VERSION) < version.parse(MIN_REQUIRED_VERSION):
        raise RuntimeError(
            f'The pymycobot library version must be {MIN_REQUIRED_VERSION} or higher. '
            f'Current version is {CURRENT_VERSION}. Please upgrade the library.'
        )
    else:
        print('pymycobot library version meets requirements!')
        from pymycobot import MyCobot280
except ImportError as e:
    print(f"Error importing pymycobot: {e}")
    sys.exit(1)

@dataclass
class RobotState:
    """Dataclass to store robot state information"""
    angles: List[float] = None
    coords: List[float] = None
    speed: int = 50
    error_status: str = "Normal"
    model: int = 0  # Default movement model

    def __post_init__(self):
        if self.angles is None:
            self.angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if self.coords is None:
            self.coords = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class RobotControlPanel(Node):
    """Robot Control Panel with ROS2 integration"""
    
    ERROR_CODES = {
        -1: 'Communication Error',
        0: 'No Error',
        1: 'Joint 1 exceeds limit position',
        2: 'Joint 2 exceeds limit position',
        3: 'Joint 3 exceeds limit position',
        4: 'Joint 4 exceeds limit position',
        5: 'Joint 5 exceeds limit position',
        6: 'Joint 6 exceeds limit position',
        16: 'Collision protection',
        17: 'Collision protection',
        18: 'Collision protection',
        19: 'Collision protection',
        32: 'Kinematics inverse solution has no solution',
        33: 'Linear motion has no adjacent solution',
        34: 'Linear motion has no adjacent solution',
    }
    
    COORD_LABELS = ["x", "y", "z", "rx", "ry", "rz"]
    
    def __init__(self):
        # Initialize ROS2 node first
        super().__init__('robot_control_panel')
        
        # Setup GUI
        self.root = tk.Tk()
        self.root.title("Robot Control Panel")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Set styles
        self.style = ttk.Style()
        self.style.configure("TButton", padding=6, relief="flat", background="#ccc")
        self.style.configure("TLabel", padding=3)
        self.style.configure("TFrame", padding=10)
        
        # Initialize robot state
        self.robot = RobotState()
        
        # Connect to robot
        self.connect_to_robot()
        
        # Create UI components
        self.setup_ui()
        
        # Initialize ROS2 subscriptions and publishers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info("Robot Control Panel initialized")

    def connect_to_robot(self) -> None:
        """Connect to the MyCobot robot"""
        try:
            # Find robot port
            port = os.popen("ls /dev/ttyUSB0").readline().strip()
            if not port:
                self.get_logger().error("Robot not found on /dev/ttyUSB0")
                messagebox.showerror("Connection Error", "Robot not found on /dev/ttyUSB0")
                sys.exit(1)
                
            self.get_logger().info(f"Connecting to robot on port: {port}, baud: 115200")
            
            # Initialize robot connection
            self.mc = MyCobot280(port, 115200)
            time.sleep(0.1)
            
            # Set fresh mode - always execute the latest command first
            self.mc.set_fresh_mode(1)
            time.sleep(0.1)
            
            # Check for errors
            self.check_robot_errors()
            
            # Get initial robot data
            self.update_robot_data()
            
        except Exception as e:
            self.get_logger().error(f"Failed to connect to robot: {e}")
            messagebox.showerror("Connection Error", f"Failed to connect to robot: {e}")
            sys.exit(1)

    def setup_ui(self) -> None:
        """Setup the UI layout and components"""
        # Create frames
        self.main_frame = ttk.Frame(self.root, padding="10")
        self.main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Left frames
        self.joint_input_frame = ttk.LabelFrame(self.main_frame, text="Joint Input")
        self.joint_input_frame.grid(row=0, column=0, padx=5, pady=5, sticky="nw")
        
        self.joint_display_frame = ttk.LabelFrame(self.main_frame, text="Current Position")
        self.joint_display_frame.grid(row=1, column=0, padx=5, pady=5, sticky="nw")
        
        # Middle frames
        self.coord_input_frame = ttk.LabelFrame(self.main_frame, text="Coordinate Input")
        self.coord_input_frame.grid(row=0, column=1, padx=5, pady=5, sticky="nw")
        
        self.settings_frame = ttk.LabelFrame(self.main_frame, text="Settings")
        self.settings_frame.grid(row=1, column=1, padx=5, pady=5, sticky="nw")
        
        # Right frames
        self.error_frame = ttk.LabelFrame(self.main_frame, text="Error Status")
        self.error_frame.grid(row=0, column=2, padx=5, pady=5, sticky="nw")
        
        self.command_frame = ttk.LabelFrame(self.main_frame, text="Commands")
        self.command_frame.grid(row=1, column=2, padx=5, pady=5, sticky="nw")
        
        # Setup input fields
        self.setup_joint_inputs()
        self.setup_coord_inputs()
        self.setup_settings()
        self.setup_error_display()
        self.setup_commands()
        self.setup_position_display()

    def setup_joint_inputs(self) -> None:
        """Setup joint input fields"""
        self.joint_vars = []
        
        for i in range(6):
            ttk.Label(self.joint_input_frame, text=f"Joint {i+1}").grid(row=i, column=0, padx=3, pady=3)
            var = tk.StringVar(value=str(self.robot.angles[i]))
            self.joint_vars.append(var)
            ttk.Entry(self.joint_input_frame, textvariable=var, width=10).grid(row=i, column=1, padx=3, pady=3)
        
        ttk.Button(
            self.joint_input_frame, 
            text="Apply Joint Values", 
            command=self.apply_joint_values
        ).grid(row=6, column=0, columnspan=2, padx=3, pady=5)

    def setup_coord_inputs(self) -> None:
        """Setup coordinate input fields"""
        self.coord_vars = []
        
        for i, label in enumerate(self.COORD_LABELS):
            ttk.Label(self.coord_input_frame, text=f"{label}").grid(row=i, column=0, padx=3, pady=3)
            var = tk.StringVar(value=str(self.robot.coords[i]))
            self.coord_vars.append(var)
            ttk.Entry(self.coord_input_frame, textvariable=var, width=10).grid(row=i, column=1, padx=3, pady=3)
            ttk.Label(self.coord_input_frame, text="mm").grid(row=i, column=2, padx=3, pady=3)
        
        ttk.Button(
            self.coord_input_frame, 
            text="Apply Coordinates", 
            command=self.apply_coord_values
        ).grid(row=6, column=0, columnspan=3, padx=3, pady=5)

    def setup_settings(self) -> None:
        """Setup settings inputs"""
        self.speed_var = tk.StringVar(value=str(self.robot.speed))
        
        ttk.Label(self.settings_frame, text="Speed:").grid(row=0, column=0, padx=3, pady=3)
        ttk.Entry(self.settings_frame, textvariable=self.speed_var, width=10).grid(row=0, column=1, padx=3, pady=3)
        
        ttk.Button(
            self.settings_frame, 
            text="Update Speed", 
            command=self.update_speed
        ).grid(row=1, column=0, columnspan=2, padx=3, pady=5)
        
        # Refresh data button
        ttk.Button(
            self.settings_frame, 
            text="Refresh Robot Data", 
            command=self.update_robot_data
        ).grid(row=2, column=0, columnspan=2, padx=3, pady=5)

    def setup_error_display(self) -> None:
        """Setup error display area"""
        self.error_var = tk.StringVar(value=self.robot.error_status)
        self.error_display = ttk.Label(
            self.error_frame, 
            textvariable=self.error_var, 
            width=30, 
            padding=10,
            background="white", 
            borderwidth=1, 
            relief="sunken"
        )
        self.error_display.grid(row=0, column=0, padx=5, pady=5)

    def setup_commands(self) -> None:
        """Setup command buttons"""
        ttk.Button(
            self.command_frame, 
            text="HOME", 
            command=self.go_home
        ).grid(row=0, column=0, padx=3, pady=3, sticky="ew")
        
        ttk.Button(
            self.command_frame, 
            text="Check Errors", 
            command=self.check_robot_errors
        ).grid(row=1, column=0, padx=3, pady=3, sticky="ew")

    def setup_position_display(self) -> None:
        """Setup position display area"""
        self.joint_display_vars = []
        self.coord_display_vars = []
        
        # Separate joint and coordinate sections with labels
        ttk.Label(self.joint_display_frame, text="Joint Values", font=('Arial', 10, 'bold')).grid(
            row=0, column=0, columnspan=2, pady=(0, 5))
        
        ttk.Label(self.joint_display_frame, text="Coordinates", font=('Arial', 10, 'bold')).grid(
            row=0, column=3, columnspan=3, pady=(0, 5))
        
        # Create display variables and labels for joints and coordinates
        for i in range(6):
            # Joint display
            ttk.Label(self.joint_display_frame, text=f"Joint {i+1}:").grid(row=i+1, column=0, padx=3, pady=2)
            j_var = tk.StringVar(value=f"{self.robot.angles[i]}°")
            self.joint_display_vars.append(j_var)
            ttk.Label(
                self.joint_display_frame, 
                textvariable=j_var, 
                width=8, 
                background="white", 
                borderwidth=1, 
                relief="sunken"
            ).grid(row=i+1, column=1, padx=3, pady=2)
            
            # Coordinate display
            ttk.Label(self.joint_display_frame, text=f"{self.COORD_LABELS[i]}:").grid(row=i+1, column=3, padx=3, pady=2)
            c_var = tk.StringVar(value=str(self.robot.coords[i]))
            self.coord_display_vars.append(c_var)
            ttk.Label(
                self.joint_display_frame, 
                textvariable=c_var, 
                width=8, 
                background="white", 
                borderwidth=1, 
                relief="sunken"
            ).grid(row=i+1, column=4, padx=3, pady=2)
            ttk.Label(self.joint_display_frame, text="mm").grid(row=i+1, column=5, padx=2, pady=2)

    def update_robot_data(self) -> None:
        """Get current robot data and update UI"""
        try:
            # Try to get robot data with timeout
            start_time = time.time()
            robot_data = None
            
            while time.time() - start_time < 2.0:
                robot_data = self.mc.get_angles_coords()
                if robot_data and isinstance(robot_data, list) and len(robot_data) >= 12:
                    break
                time.sleep(0.1)
            
            if not robot_data or len(robot_data) < 12:
                self.get_logger().warning("Failed to get robot data")
                return
                
            # Update robot state
            self.robot.angles = robot_data[:6]
            self.robot.coords = robot_data[6:]
            
            # Update display
            self.update_position_display()
            
            # Check for errors
            self.check_robot_errors()
            
        except Exception as e:
            self.get_logger().error(f"Error updating robot data: {e}")
            self.error_var.set(f"Error: {str(e)}")

    def update_position_display(self) -> None:
        """Update position display with current robot data"""
        for i in range(6):
            self.joint_display_vars[i].set(f"{self.robot.angles[i]}°")
            self.coord_display_vars[i].set(str(self.robot.coords[i]))

    def check_robot_errors(self) -> None:
        """Check robot for errors and update UI"""
        try:
            error_code = self.mc.get_error_information()
            
            if error_code is None:
                self.robot.error_status = "Unable to get error information"
            else:
                self.robot.error_status = self.ERROR_CODES.get(
                    error_code, 
                    f"Unknown error code: {error_code}"
                )
                
                if error_code > 0:
                    # Critical error
                    self.error_display.configure(background="tomato")
                elif error_code == -1:
                    # Communication error
                    self.error_display.configure(background="orange")
                else:
                    # No error
                    self.error_display.configure(background="pale green")
            
            self.error_var.set(self.robot.error_status)
            self.get_logger().info(f"Robot error status: {self.robot.error_status}")
            
        except Exception as e:
            self.get_logger().error(f"Error checking robot status: {e}")
            self.error_var.set(f"Error checking status: {str(e)}")
            self.error_display.configure(background="tomato")

    def go_home(self) -> None:
        """Send robot to home position"""
        try:
            # Set home position values
            home_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
            # Update UI
            for i, var in enumerate(self.joint_vars):
                var.set(str(home_angles[i]))
            
            # Send command to robot
            self.mc.sync_send_angles(home_angles, self.robot.speed)
            
            # Update robot state and display
            self.robot.angles = home_angles
            self.update_position_display()
            
            self.get_logger().info("Robot sent to home position")
            
        except Exception as e:
            self.get_logger().error(f"Error sending robot home: {e}")
            messagebox.showerror("Command Error", f"Failed to send robot home: {e}")

    def update_speed(self) -> None:
        """Update robot speed setting"""
        try:
            new_speed = int(float(self.speed_var.get()))
            if 1 <= new_speed <= 100:
                self.robot.speed = new_speed
                self.get_logger().info(f"Speed updated to {new_speed}")
            else:
                messagebox.showwarning("Invalid Input", "Speed must be between 1 and 100")
                self.speed_var.set(str(self.robot.speed))
        except ValueError:
            messagebox.showwarning("Invalid Input", "Please enter a valid number for speed")
            self.speed_var.set(str(self.robot.speed))

    def apply_joint_values(self) -> None:
        """Apply joint values from UI to robot"""
        try:
            # Read values from input fields
            joint_values = [float(var.get()) for var in self.joint_vars]
            
            # Send to robot
            self.get_logger().info(f"Sending joint values: {joint_values}, speed: {self.robot.speed}")
            self.mc.sync_send_angles(joint_values, self.robot.speed)
            
            # Update robot state
            self.robot.angles = joint_values
            self.update_position_display()
            
        except ValueError as e:
            messagebox.showerror("Input Error", "Please enter valid numbers for all joint values")
        except Exception as e:
            self.get_logger().error(f"Error applying joint values: {e}")
            messagebox.showerror("Command Error", f"Failed to apply joint values: {e}")

    def apply_coord_values(self) -> None:
        """Apply coordinate values from UI to robot"""
        try:
            # Read values from input fields
            coord_values = [float(var.get()) for var in self.coord_vars]
            
            # Send to robot
            self.get_logger().info(f"Sending coordinates: {coord_values}, speed: {self.robot.speed}, model: {self.robot.model}")
            self.mc.sync_send_coords(coord_values, self.robot.speed, self.robot.model)
            
            # Update robot state - need to get actual angles from robot since inverse kinematics happened
            self.update_robot_data()
            
        except ValueError as e:
            messagebox.showerror("Input Error", "Please enter valid numbers for all coordinates")
        except Exception as e:
            self.get_logger().error(f"Error applying coordinates: {e}")
            messagebox.showerror("Command Error", f"Failed to apply coordinates: {e}")

    def joy_callback(self, msg: Joy) -> None:
        """Handle joystick input and publish to cmd_vel"""
        try:
            twist = Twist()
            # Map joystick axes to twist message
            twist.linear.x = msg.axes[1]  # Forward/Backward
            twist.angular.z = msg.axes[0]  # Left/Right turning
            
            # Publish twist message
            self.cmd_vel_pub.publish(twist)
            
        except Exception as e:
            self.get_logger().error(f"Error in joy callback: {e}")

    def on_closing(self) -> None:
        """Handle window closing"""
        self.get_logger().info("Shutting down robot control panel")
        self.destroy_node()
        self.root.destroy()
        sys.exit(0)

    def run(self) -> None:
        """Run the control panel"""
        # Start periodic updates
        self.root.after(1000, self.periodic_update)
        
        try:
            # Start Tkinter event loop
            self.root.mainloop()
        except KeyboardInterrupt:
            self.get_logger().info("KeyboardInterrupt received, shutting down")
            self.on_closing()

    def periodic_update(self) -> None:
        """Periodic update function"""
        # Check for errors periodically
        self.check_robot_errors()
        
        # Schedule next update
        self.root.after(5000, self.periodic_update)


def main(args=None):
    """Main function"""
    try:
        # Initialize ROS2
        rclpy.init(args=args)
        
        # Create control panel
        control_panel = RobotControlPanel()
        
        # Create executor
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(control_panel)
        
        # Start a thread for ROS2 spinning
        import threading
        ros_thread = threading.Thread(target=executor.spin, daemon=True)
        ros_thread.start()
        
        # Run the GUI
        control_panel.run()
        
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        # Ensure ROS2 shuts down properly
        rclpy.shutdown()


if __name__ == "__main__":
    main()
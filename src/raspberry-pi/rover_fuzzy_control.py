#!/usr/bin/env python3
"""
4-Wheel Rover Center Tracking with Fuzzy Logic Control
-----------------------------------------------------
This script uses:
1. Pi Camera v2 to detect objects and calculate center offset and distance
2. Fuzzy logic for smooth rover control of a 4-wheel rover

Hardware Configuration:
- 4 motors (FR, FL, RR, RL) with forward, reverse, and enable pins
- Pi Camera v2 for object detection and distance estimation
"""

import os
import sys
import argparse
import time
import math
import RPi.GPIO as GPIO

import cv2
import numpy as np
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("Warning: Ultralytics YOLO not available. Detection features will be disabled.")

# Pi Camera v2 specifications
CAM_HORIZONTAL_FOV = 62.2  # degrees
CAM_VERTICAL_FOV = 48.8  # degrees
CAM_FOCAL_LENGTH_MM = 3.04  # mm

# Default settings
DEFAULT_RESOLUTION = "640x480"
DEFAULT_CONFIDENCE = 0.5
DEFAULT_MAX_DISTANCE = 300  # cm - maximum distance range

# GPIO pin configurations
# Front Right (FR) motor
FR_FORWARD = 20
FR_REVERSE = 16
FR_ENABLE = 12

# Front Left (FL) motor
FL_FORWARD = 21
FL_REVERSE = 26
FL_ENABLE = 13

# Rear Right (RR) motor
RR_FORWARD = 17
RR_REVERSE = 27
RR_ENABLE = 18

# Rear Left (RL) motor
RL_FORWARD = 22
RL_REVERSE = 23
RL_ENABLE = 19

# Target distance for maintaining separation from objects (in cm)
TARGET_DISTANCE = 70 # 0.9m = 90cm

# Offset interval for considering object as centered (in degrees)
OFFSET_INTERVAL = 0.1  # Consider angles between -0.1 and +0.1 as centered


class FourWheelMotorController:
    """Class for controlling a 4-wheel rover's motors"""
    
    def __init__(self, 
                 fr_fwd=FR_FORWARD, fr_rev=FR_REVERSE, fr_en=FR_ENABLE,
                 fl_fwd=FL_FORWARD, fl_rev=FL_REVERSE, fl_en=FL_ENABLE,
                 rr_fwd=RR_FORWARD, rr_rev=RR_REVERSE, rr_en=RR_ENABLE,
                 rl_fwd=RL_FORWARD, rl_rev=RL_REVERSE, rl_en=RL_ENABLE):
        """Initialize motor controller with GPIO pins for all 4 wheels"""
        # Store pin configurations
        self.pins = {
            'fr': {'fwd': fr_fwd, 'rev': fr_rev, 'en': fr_en},
            'fl': {'fwd': fl_fwd, 'rev': fl_rev, 'en': fl_en},
            'rr': {'fwd': rr_fwd, 'rev': rr_rev, 'en': rr_en},
            'rl': {'fwd': rl_fwd, 'rev': rl_rev, 'en': rl_en}
        }
        
        # Set up GPIO
        GPIO.setmode(GPIO.BCM)
        
        # Setup all pins
        for wheel in self.pins.values():
            GPIO.setup(wheel['fwd'], GPIO.OUT)
            GPIO.setup(wheel['rev'], GPIO.OUT)
            GPIO.setup(wheel['en'], GPIO.OUT)
            
            # Initialize PWM for enable pins
            wheel['pwm'] = GPIO.PWM(wheel['en'], 100)  # 100Hz frequency
            wheel['pwm'].start(0)  # Start with 0% duty cycle (stopped)
        
        print(f"4-wheel motor controller initialized on pins: FR={fr_fwd},{fr_rev},{fr_en}, FL={fl_fwd},{fl_rev},{fl_en}, RR={rr_fwd},{rr_rev},{rr_en}, RL={rl_fwd},{rl_rev},{rl_en}")
    
    def set_wheel_speed(self, wheel, speed):
        """Set speed for a single wheel
        
        Args:
            wheel: Wheel identifier ('fr', 'fl', 'rr', or 'rl')
            speed: Speed (-100 to 100, negative for reverse)
        """
        if wheel not in self.pins:
            return
        
        # Get direction
        forward = speed >= 0
        duty_cycle = abs(speed)
        
        # Ensure duty cycle is in range
        duty_cycle = max(0, min(100, duty_cycle))
        
        # Set direction pins
        GPIO.output(self.pins[wheel]['fwd'], forward)
        GPIO.output(self.pins[wheel]['rev'], not forward)
        
        # Set speed via PWM
        self.pins[wheel]['pwm'].ChangeDutyCycle(duty_cycle)
    
    def set_side_speeds(self, left_speed, right_speed):
        """Set speeds for left and right sides of the rover
        
        Args:
            left_speed: Speed for left side wheels (-100 to 100)
            right_speed: Speed for right side wheels (-100 to 100)
        """
        # Set left wheels
        self.set_wheel_speed('fl', left_speed)
        self.set_wheel_speed('rl', left_speed)
        
        # Set right wheels
        self.set_wheel_speed('fr', right_speed)
        self.set_wheel_speed('rr', right_speed)
    
    def stop(self):
        """Stop all motors"""
        for wheel in self.pins:
            self.set_wheel_speed(wheel, 0)
    
    def cleanup(self):
        """Clean up GPIO resources"""
        self.stop()
        for wheel in self.pins.values():
            wheel['pwm'].stop()


class FuzzyController:
    """Fuzzy logic controller for rover navigation"""
    
    def __init__(self, target_distance=TARGET_DISTANCE, offset_interval=OFFSET_INTERVAL):
        """Initialize fuzzy controller with membership functions"""
        self.target_distance = target_distance
        self.offset_interval = offset_interval
        
        # Define fuzzy sets for inputs
        # Offset angle (in degrees)
        self.offset_angle_ranges = {
            'left_far': [-30, -20, -10],    # Left far: -30 to -10 degrees
            'left_near': [-15, -5, -self.offset_interval],  # Left near: -15 to -offset_interval degrees
            'center': [-self.offset_interval, 0, self.offset_interval],  # Center: -offset_interval to +offset_interval degrees
            'right_near': [self.offset_interval, 5, 15],    # Right near: +offset_interval to 15 degrees
            'right_far': [10, 20, 30]       # Right far: 10 to 30 degrees
        }
        
        # Distance (in cm)
        self.distance_ranges = {
            'very_close': [0, target_distance/3, target_distance*2/3],        # Very close
            'ideal': [target_distance/2, target_distance, target_distance*1.5],  # Ideal distance
            'far': [target_distance, target_distance*2, target_distance*3],   # Far
            'very_far': [target_distance*2, target_distance*4, target_distance*6]  # Very far
        }
        
        # Define fuzzy sets for outputs
        # Motor speeds (-100 to 100)
        self.left_motor_ranges = {
            'reverse_fast': [-100, -70, -40],   # Reverse fast: -100 to -40
            'reverse_slow': [-60, -30, 0],      # Reverse slow: -60 to 0
            'stop': [-20, 0, 20],               # Stop: -20 to 20
            'forward_slow': [0, 30, 60],        # Forward slow: 0 to 60
            'forward_fast': [40, 70, 100]       # Forward fast: 40 to 100
        }
        
        self.right_motor_ranges = {
            'reverse_fast': [-100, -70, -40],   # Reverse fast: -100 to -40
            'reverse_slow': [-60, -30, 0],      # Reverse slow: -60 to 0
            'stop': [-20, 0, 20],               # Stop: -20 to 20
            'forward_slow': [0, 30, 60],        # Forward slow: 0 to 60
            'forward_fast': [40, 70, 100]       # Forward fast: 40 to 100
        }
        
        # Define fuzzy rules for smooth navigation
        # Each rule is (offset_angle_set, distance_set, left_motor_set, right_motor_set)
        self.rules = [
            # When object is centered
            ('center', 'very_close', 'reverse_slow', 'reverse_slow'),    # Back up if too close
            ('center', 'ideal', 'stop', 'stop'),                         # Stop at ideal distance
            ('center', 'far', 'forward_slow', 'forward_slow'),           # Move forward slowly
            ('center', 'very_far', 'forward_fast', 'forward_fast'),      # Move forward quickly
            
            # When object is to the left
            ('left_near', 'very_close', 'reverse_fast', 'reverse_slow'),  # Turn right while backing up
            ('left_near', 'ideal', 'stop', 'forward_slow'),               # Turn right in place
            ('left_near', 'far', 'forward_slow', 'forward_fast'),         # Turn right while moving forward
            ('left_near', 'very_far', 'forward_fast', 'forward_fast'),    # Speed up right motor
            
            # When object is far to the left
            ('left_far', 'very_close', 'reverse_fast', 'stop'),          # Hard right while backing up
            ('left_far', 'ideal', 'reverse_slow', 'forward_slow'),       # Hard right turn
            ('left_far', 'far', 'stop', 'forward_fast'),                 # Sharp right
            ('left_far', 'very_far', 'forward_slow', 'forward_fast'),    # Wide right
            
            # When object is to the right
            ('right_near', 'very_close', 'reverse_slow', 'reverse_fast'),  # Turn left while backing up
            ('right_near', 'ideal', 'forward_slow', 'stop'),               # Turn left in place
            ('right_near', 'far', 'forward_fast', 'forward_slow'),         # Turn left while moving forward
            ('right_near', 'very_far', 'forward_fast', 'forward_fast'),    # Speed up left motor
            
            # When object is far to the right
            ('right_far', 'very_close', 'stop', 'reverse_fast'),          # Hard left while backing up
            ('right_far', 'ideal', 'forward_slow', 'reverse_slow'),       # Hard left turn
            ('right_far', 'far', 'forward_fast', 'stop'),                 # Sharp left
            ('right_far', 'very_far', 'forward_fast', 'forward_slow')     # Wide left
        ]
    
    def triangular_membership(self, x, abc):
        """Calculate triangular membership function
        
        Args:
            x: Input value
            abc: Three points of the triangle [a, b, c]
            
        Returns:
            Membership value (0 to 1)
        """
        a, b, c = abc
        
        if x <= a or x >= c:
            return 0
        elif a < x <= b:
            return (x - a) / (b - a)
        else:  # b < x < c
            return (c - x) / (c - b)
    
    def get_membership_values(self, value, ranges):
        """Calculate membership values for all sets
        
        Args:
            value: Input value
            ranges: Dictionary of set names to triangle points
            
        Returns:
            Dictionary of set names to membership values
        """
        memberships = {}
        for set_name, range_points in ranges.items():
            memberships[set_name] = self.triangular_membership(value, range_points)
        return memberships
    
    def defuzzify_centroid(self, aggregated_outputs, output_range):
        """Defuzzify using centroid method
        
        Args:
            aggregated_outputs: Dictionary of output sets to membership values
            output_range: Range to compute centroid over
            
        Returns:
            Defuzzified value
        """
        # Simple discrete centroid calculation
        step = 1  # Step size for discretization
        points = list(range(output_range[0], output_range[1] + 1, step))
        
        numerator = 0
        denominator = 0
        
        for x in points:
            # Find membership of x in each output set
            membership = 0
            for set_name, strength in aggregated_outputs.items():
                if set_name in (self.left_motor_ranges if output_range[0] == -100 else self.right_motor_ranges):
                    set_range = (self.left_motor_ranges if output_range[0] == -100 else self.right_motor_ranges)[set_name]
                    # Use min of strength and the membership of x in the set
                    membership = max(membership, min(strength, self.triangular_membership(x, set_range)))
            
            numerator += x * membership
            denominator += membership
        
        if denominator == 0:
            return 0
        
        return numerator / denominator
    
    def control(self, offset_angle, distance):
        """Calculate control outputs using fuzzy logic
        
        Args:
            offset_angle: Angular offset from center in degrees
            distance: Distance to object in cm
            
        Returns:
            (left_motor_speed, right_motor_speed) as -100 to 100 values
        """
        # Ensure inputs are within expected ranges
        offset_angle = max(-30, min(30, offset_angle))
        distance = max(0, min(self.target_distance * 6, distance))
        
        # Fuzzify inputs
        offset_angle_memberships = self.get_membership_values(offset_angle, self.offset_angle_ranges)
        distance_memberships = self.get_membership_values(distance, self.distance_ranges)
        
        # Apply fuzzy rules
        left_motor_outputs = {}
        right_motor_outputs = {}
        
        for rule in self.rules:
            offset_set, distance_set, left_output, right_output = rule
            
            # Calculate rule strength using min (AND operation)
            rule_strength = min(offset_angle_memberships.get(offset_set, 0),
                               distance_memberships.get(distance_set, 0))
            
            # Apply rule strength to outputs
            if rule_strength > 0:
                # Store the maximum rule strength for each output set
                left_motor_outputs[left_output] = max(left_motor_outputs.get(left_output, 0), rule_strength)
                right_motor_outputs[right_output] = max(right_motor_outputs.get(right_output, 0), rule_strength)
        
        # Defuzzify to get crisp outputs
        left_motor_speed = self.defuzzify_centroid(left_motor_outputs, [-100, 100])
        right_motor_speed = self.defuzzify_centroid(right_motor_outputs, [-100, 100])
        
        return left_motor_speed, right_motor_speed


class CameraDistanceEstimator:
    """Class for estimating distance using camera and object size"""
    
    def __init__(self, focal_length_mm=CAM_FOCAL_LENGTH_MM, sensor_width_mm=3.68):
        """Initialize distance estimator with camera parameters
        
        Args:
            focal_length_mm: Focal length of camera in mm
            sensor_width_mm: Width of camera sensor in mm
        """
        self.focal_length_mm = focal_length_mm
        self.sensor_width_mm = sensor_width_mm
        
        # Calibration parameters - these would need to be adjusted based on 
        # real-world measurements for accurate distance estimation
        self.object_size_calibration = {}  # Map of class_id to real-world width in cm
        
        # Default values for common objects - these are rough estimates
        # and should be calibrated for actual objects
        self.object_size_calibration = {
            0: 50,   # person (width in cm)
            2: 150,  # car
            5: 30,   # bus
            7: 40,   # truck
            15: 30,  # cat
            16: 40,  # dog
            27: 10,  # backpack
            28: 25,  # umbrella
            31: 35,  # handbag
            32: 15,  # tie
            37: 60,  # skateboard
            41: 10,  # cup
            42: 20,  # fork
            43: 20,  # knife
            44: 20,  # spoon
            45: 20,  # bowl
            46: 30,  # banana
            47: 8,   # apple
            48: 8,   # sandwich
            49: 10,  # orange
            67: 25,  # cell phone
        }
        
        # Default object width (in cm) to use when class is unknown
        self.default_object_width = 30  # cm
    
    def set_object_width(self, class_id, width_cm):
        """Set real-world width for a specific object class
        
        Args:
            class_id: Object class ID
            width_cm: Real-world width in cm
        """
        self.object_size_calibration[class_id] = width_cm
    
    def get_object_width(self, class_id):
        """Get real-world width for a specific object class
        
        Args:
            class_id: Object class ID
            
        Returns:
            Width in cm
        """
        return self.object_size_calibration.get(class_id, self.default_object_width)
    
    def estimate_distance(self, bbox_width_pixels, image_width_pixels, class_id=None):
        """Estimate distance to object based on apparent size
        
        This uses the pinhole camera model: distance = (object_width_real * focal_length) / object_width_pixels
        
        Args:
            bbox_width_pixels: Width of object bounding box in pixels
            image_width_pixels: Width of image in pixels
            class_id: Object class ID for real-world size lookup
            
        Returns:
            Estimated distance in cm
        """
        # Get real-world object width based on class
        object_width_cm = self.get_object_width(class_id)
        
        # Calculate the focal length in pixels
        # focal_length_pixels = (image_width_pixels * focal_length_mm) / sensor_width_mm
        focal_length_pixels = (image_width_pixels * self.focal_length_mm) / self.sensor_width_mm
        
        # Estimate distance using the pinhole camera model
        if bbox_width_pixels > 0:
            distance_cm = (object_width_cm * focal_length_pixels) / bbox_width_pixels
            return distance_cm
        else:
            return float('inf')  # Return infinity for zero-width objects


class CenterTracker:
    """Main class for tracking objects and controlling rover"""
    
    def __init__(self, args):
        """Initialize with parsed command line arguments"""
        self.args = args
        self.model = None
        self.camera = None
        self.camera_type = None
        self.motor_controller = None
        self.fuzzy_controller = None
        self.fps_buffer = []
        self.fps_avg_len = 30
        self.avg_fps = 0
        self.simulation_mode = args.simulation
        self.last_distance_measure_time = 0
        self.distance_measurement_interval = 0.1  # seconds between measurements
        self.current_distance = None
        self.camera_distance_estimator = None
        
        # Manual control mode
        self.manual_mode = False
        self.key_states = {
            ord('w'): False,  # Forward
            ord('s'): False,  # Backward
            ord('a'): False,  # Left
            ord('d'): False,  # Right
        }
        
        # Parse resolution
        if args.resolution:
            self.resW, self.resH = map(int, args.resolution.split('x'))
        else:
            self.resW, self.resH = map(int, DEFAULT_RESOLUTION.split('x'))
        
        # Initialize controllers
        self.fuzzy_controller = FuzzyController(
            target_distance=args.target_distance,
            offset_interval=args.offset_interval if hasattr(args, 'offset_interval') else OFFSET_INTERVAL
        )
        self.camera_distance_estimator = CameraDistanceEstimator()
        
        # Initialize hardware if not in simulation mode
        if not self.simulation_mode:
            self.motor_controller = FourWheelMotorController(
                fr_fwd=FR_FORWARD, fr_rev=FR_REVERSE, fr_en=FR_ENABLE,
                fl_fwd=FL_FORWARD, fl_rev=FL_REVERSE, fl_en=FL_ENABLE,
                rr_fwd=RR_FORWARD, rr_rev=RR_REVERSE, rr_en=RR_ENABLE,
                rl_fwd=RL_FORWARD, rl_rev=RL_REVERSE, rl_en=RL_ENABLE
            )
    
    def run(self):
        """Main execution method"""
        # Load model if detection is enabled
        if self.args.model and YOLO_AVAILABLE:
            self._load_model()
        
        # Initialize camera
        if not self._init_camera():
            return
        
        # Main processing loop
        try:
            while True:
                t_start = time.perf_counter()
                
                # Capture frame
                frame = self._capture_frame()
                if frame is None:
                    break
                
                # Resize frame to desired resolution
                if frame.shape[1] != self.resW or frame.shape[0] != self.resH:
                    frame = cv2.resize(frame, (self.resW, self.resH))
                
                # Draw center of frame
                center_x, center_y = self.resW // 2, self.resH // 2
                cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)
                cv2.line(frame, (center_x - 10, center_y), (center_x + 10, center_y), (255, 0, 0), 1)
                cv2.line(frame, (center_x, center_y - 10), (center_x, center_y + 10), (255, 0, 0), 1)
                
                # Process frame for detections
                tracked_obj = None
                if self.model:
                    frame, tracked_obj = self._process_detections(frame)
                    
                    # Check if manual mode is active
                    if self.manual_mode:
                        self._handle_manual_control(frame)
                    else:
                        # Calculate control outputs if we have a tracked object
                        if tracked_obj and tracked_obj.get('angle_offset') is not None:
                            # Get offset angle
                            offset_angle = tracked_obj['angle_offset']
                            
                            # Get distance (from camera estimation)
                            dist = tracked_obj.get('distance_cm', 100)  # Default to 100cm if no reading
                            
                            # Calculate control using fuzzy logic
                            left_speed, right_speed = self.fuzzy_controller.control(offset_angle, dist)
                            
                            # Apply motor control if not in simulation mode
                            if self.motor_controller and not self.simulation_mode:
                                self.motor_controller.set_side_speeds(left_speed, right_speed)
                            
                            # Display control outputs
                            control_text = [
                                f"Left motors: {left_speed:.1f}%",
                                f"Right motors: {right_speed:.1f}%"
                            ]
                            
                            # Display on frame
                            for i, text in enumerate(control_text):
                                y_pos = self.resH - 60 + i*25
                                cv2.putText(frame, text, (10, y_pos),
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                        else:
                            # No object detected, stop motors
                            if self.motor_controller and not self.simulation_mode:
                                self.motor_controller.stop()
                else:
                    # If no model is loaded, just show camera feed with overlays
                    cv2.putText(frame, "Model not loaded - showing camera feed only", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # Calculate and display FPS
                t_end = time.perf_counter()
                fps = 1.0 / (t_end - t_start)
                self._update_fps(fps)
                cv2.putText(frame, f"FPS: {self.avg_fps:.1f}", (self.resW - 120, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                # Display target distance
                cv2.putText(frame, f"Target: {self.fuzzy_controller.target_distance} cm", (self.resW - 200, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Display offset interval
                cv2.putText(frame, f"Offset Interval: ±{self.fuzzy_controller.offset_interval}°", 
                           (self.resW - 260, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Display simulation mode if active
                if self.simulation_mode:
                    cv2.putText(frame, "SIMULATION MODE", (self.resW - 200, self.resH - 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # Display manual mode status
                if self.manual_mode:
                    cv2.putText(frame, "MANUAL MODE", (10, self.resH - 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                else:
                    cv2.putText(frame, "AUTO MODE", (10, self.resH - 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Always display estimated distance if available
                if tracked_obj and 'distance_cm' in tracked_obj:
                    cv2.putText(frame, f"Est. Distance: {tracked_obj['distance_cm']:.1f} cm", 
                               (self.resW - 300, 120),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                
                # Display frame
                cv2.imshow("Rover Center Tracker", frame)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):  # Quit
                    break
                elif key == ord('s') and not self.manual_mode:  # Save screenshot (only when not conflicting with manual control)
                    timestamp = time.strftime("%Y%m%d-%H%M%S")
                    filename = f"capture_{timestamp}.jpg"
                    cv2.imwrite(filename, frame)
                    print(f"Screenshot saved as {filename}")
                elif key == ord('p'):  # Pause
                    print("Paused - press any key to continue")
                    cv2.waitKey(0)
                elif key == ord('m'):  # Toggle manual control mode
                    self.manual_mode = not self.manual_mode
                    print(f"Manual control mode: {'ON' if self.manual_mode else 'OFF'}")
                    # Stop the rover when switching modes
                    if self.motor_controller and not self.simulation_mode:
                        self.motor_controller.stop()
                    # Reset all key states when toggling mode
                    for k in self.key_states:
                        self.key_states[k] = False
                
                # Always update key states for manual control
                # This allows for immediate response to manual control
                # even if we toggle to manual mode in the middle of operation
                self._update_key_states(key)
                
        finally:
            # Clean up resources
            self._cleanup()
    
    def _init_camera(self):
        """Initialize camera based on source argument"""
        source = self.args.source
        
        # Check if the source is PiCamera
        if 'picamera' in source:
            try:
                from picamera2 import Picamera2
                self.camera = Picamera2()
                self.camera.configure(self.camera.create_video_configuration(
                    main={"format": 'XRGB8888', "size": (self.resW, self.resH)}
                ))
                self.camera.start()
                self.camera_type = 'picamera'
                print("Using Picamera2")
                return True
            except ImportError:
                print("Error: Picamera2 module not available")
                return False
        # Check if the source is USB camera
        elif 'usb' in source:
            try:
                idx = int(source[3:])
                self.camera = cv2.VideoCapture(idx)
                if not self.camera.isOpened():
                    print(f"Error: Could not open USB camera at index {idx}")
                    return False
                self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.resW)
                self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resH)
                self.camera_type = 'opencv'
                print(f"Using USB camera at index {idx}")
                return True
            except ValueError:
                print(f"Error: Invalid USB camera index: {source[3:]}")
                return False
        # Check if the source is a video file
        elif os.path.isfile(source) and source.lower().endswith(('.mp4', '.avi', '.mov', '.mkv')):
            self.camera = cv2.VideoCapture(source)
            if not self.camera.isOpened():
                print(f"Error: Could not open video file {source}")
                return False
            self.camera_type = 'opencv'
            print(f"Using video file: {source}")
            return True
        else:
            print(f"Error: Unsupported camera source: {source}")
            return False
    
    def _capture_frame(self):
        """Capture a frame from the camera"""
        if self.camera_type == 'picamera':
            frame_bgra = self.camera.capture_array()
            return cv2.cvtColor(np.copy(frame_bgra), cv2.COLOR_BGRA2BGR)
        else:  # OpenCV camera
            ret, frame = self.camera.read()
            if not ret:
                print("Failed to capture frame")
                return None
            return frame
    
    def _load_model(self):
        """Load YOLO model for object detection"""
        try:
            model_path = self.args.model
            if not os.path.exists(model_path):
                print(f"Error: Model file not found: {model_path}")
                return
            
            print(f"Loading YOLO model: {model_path}")
            self.model = YOLO(model_path, task='detect')
            print("Model loaded successfully")
        except Exception as e:
            print(f"Error loading model: {e}")
            self.model = None
    
    def _calculate_angle_offset(self, object_center_x, object_center_y):
        """Calculate angular offset from center of frame
        
        Args:
            object_center_x: x coordinate of object center in pixels
            object_center_y: y coordinate of object center in pixels
            
        Returns:
            x_angle_offset: horizontal angular offset in degrees
            y_angle_offset: vertical angular offset in degrees
        """
        # Calculate center of frame
        center_x = self.resW / 2
        center_y = self.resH / 2
        
        # Calculate pixel offset
        x_offset = object_center_x - center_x
        y_offset = center_y - object_center_y  # Inverted because y-axis in images is top to bottom
        
        # Calculate angular offset based on field of view and resolution
        x_angle_offset = (x_offset / (self.resW / 2)) * (CAM_HORIZONTAL_FOV / 2)
        y_angle_offset = (y_offset / (self.resH / 2)) * (CAM_VERTICAL_FOV / 2)
        
        return x_angle_offset, y_angle_offset
    
    def _process_detections(self, frame):
        """Process frame for object detections and center offset calculation"""
        # Make a copy of the frame for drawing
        display_frame = frame.copy()
        
        # Run inference
        results = self.model(frame, verbose=False)
        
        # Extract detections
        detections = results[0].boxes
        
        # Get class labels
        try:
            labels = self.model.names
        except:
            labels = {}
        
        # Variables for processing
        object_count = 0
        tracked_object = None
        bbox_colors = [(255,0,0), (0,255,0), (0,0,255), (255,255,0), 
                      (0,255,255), (255,0,255), (192,192,192), (128,128,128)]
        
        # Process each detection
        for i in range(len(detections)):
            # Get bounding box coordinates
            xyxy_tensor = detections[i].xyxy.cpu()
            xyxy = xyxy_tensor.numpy().squeeze()
            xmin, ymin, xmax, ymax = xyxy.astype(int)
            
            # Get class and confidence
            classidx = int(detections[i].cls.item())
            conf = detections[i].conf.item()
            
            # Class name
            classname = labels.get(classidx, f"Class {classidx}")
            
            # Check confidence threshold
            if conf > self.args.confidence:
                # Select color
                color = bbox_colors[classidx % len(bbox_colors)]
                
                # Draw bounding box
                cv2.rectangle(display_frame, (xmin, ymin), (xmax, ymax), color, 2)
                
                # Calculate center of object
                obj_center_x = (xmin + xmax) // 2
                obj_center_y = (ymin + ymax) // 2
                
                # Draw center of object
                cv2.circle(display_frame, (obj_center_x, obj_center_y), 4, color, -1)
                
                # Calculate angle offset from center
                x_angle, y_angle = self._calculate_angle_offset(obj_center_x, obj_center_y)
                
                # Calculate bbox width and height
                bbox_width = xmax - xmin
                bbox_height = ymax - ymin
                
                # Estimate distance using camera
                distance_cm = self.camera_distance_estimator.estimate_distance(
                    bbox_width, self.resW, classidx
                )
                
                # Add label with confidence and angle
                label_text = f"{classname}: {int(conf*100)}%"
                labelSize, baseLine = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(display_frame, 
                             (xmin, ymin - labelSize[1] - 10), 
                             (xmin + labelSize[0], ymin),
                             color, cv2.FILLED)
                cv2.putText(display_frame, label_text, (xmin, ymin - 7), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
                
                # Store object data
                obj_data = {
                    'classidx': classidx,
                    'classname': classname,
                    'confidence': conf,
                    'center_x': obj_center_x,
                    'center_y': obj_center_y,
                    'width': bbox_width,
                    'height': bbox_height,
                    'angle_offset': x_angle,  # We're primarily concerned with horizontal offset
                    'vertical_angle': y_angle,
                    'distance_cm': distance_cm,
                    'color': color
                }
                
                # Check if this is the object we want to track
                track_this = False
                if self.args.track_class is not None:
                    # Track specific class if requested
                    if classname.lower() == self.args.track_class.lower():
                        track_this = True
                elif tracked_object is None:
                    # Otherwise track the first object over confidence threshold
                    track_this = True
                
                if track_this:
                    tracked_object = obj_data
                
                # Count objects
                object_count += 1
        
        # Draw object count
        cv2.putText(display_frame, f"Objects: {object_count}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Draw center to object line and angle/distance info for tracked object
        if tracked_object:
            # Draw line from center to object
            center_x, center_y = self.resW // 2, self.resH // 2
            cv2.line(display_frame, 
                    (center_x, center_y), 
                    (int(tracked_object['center_x']), int(tracked_object['center_y'])), 
                    tracked_object['color'], 2)
            
            # Add angle and distance information
            info_text = [
                f"Offset: {tracked_object['angle_offset']:.1f}°"
            ]
            
            # Check if within offset interval (centered)
            offset = tracked_object['angle_offset']
            if abs(offset) <= self.fuzzy_controller.offset_interval:
                info_text[0] = f"Offset: {offset:.1f}° (CENTERED)"
            
            # Add distance if available
            if 'distance_cm' in tracked_object:
                info_text.append(f"Distance: {tracked_object['distance_cm']:.1f} cm")
            
            # Add object class
            info_text.append(f"Object: {tracked_object['classname']}")
            
            # Display information
            for i, text in enumerate(info_text):
                y_pos = 60 + i*25
                cv2.putText(display_frame, text, (10, y_pos), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, tracked_object['color'], 2)
        
        return display_frame, tracked_object
    
    def _update_fps(self, fps):
        """Update the running average FPS"""
        if len(self.fps_buffer) >= self.fps_avg_len:
            self.fps_buffer.pop(0)
        self.fps_buffer.append(fps)
        self.avg_fps = sum(self.fps_buffer) / len(self.fps_buffer)
    
    def _update_key_states(self, key):
        """Update key states for manual control
        
        Args:
            key: Key code from cv2.waitKey
        """
        # Check for key press
        if key in self.key_states:
            self.key_states[key] = True
        
        # Check for key release (on next frame)
        for k in self.key_states:
            if k != key and self.key_states[k]:
                self.key_states[k] = False
    
    def _handle_manual_control(self, frame):
        """Handle manual control based on key states"""
        left_speed = 0
        right_speed = 0
        
        # Forward (W key) - both motors forward
        if self.key_states[ord('w')]:
            left_speed = 50
            right_speed = 50
        
        # Backward (S key) - both motors backward
        elif self.key_states[ord('s')]:
            left_speed = -50
            right_speed = -50
        
        # Left (A key) - right motor forward, left motor slower or backward
        elif self.key_states[ord('a')]:
            left_speed = -20
            right_speed = 50
        
        # Right (D key) - left motor forward, right motor slower or backward
        elif self.key_states[ord('d')]:
            left_speed = 50
            right_speed = -20
            
        # The 'w' key specifically should stop when released
        # as per the user's requirement
        if not self.key_states[ord('w')] and not self.key_states[ord('s')] and \
           not self.key_states[ord('a')] and not self.key_states[ord('d')]:
            left_speed = 0
            right_speed = 0
        
        # Apply motor control if not in simulation mode
        if self.motor_controller and not self.simulation_mode:
            self.motor_controller.set_side_speeds(left_speed, right_speed)
        
        # Display manual control info
        control_text = [
            f"Left motors: {left_speed}%",
            f"Right motors: {right_speed}%",
            "Controls: W-Forward, S-Backward, A-Left, D-Right"
        ]
        
        # Display on frame
        for i, text in enumerate(control_text):
            y_pos = self.resH - 110 + i*25
            cv2.putText(frame, text, (10, y_pos),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    
    def _cleanup(self):
        """Clean up resources"""
        print("Cleaning up...")
        
        # Close camera
        if self.camera is not None:
            if self.camera_type == 'picamera':
                self.camera.stop()
            else:
                self.camera.release()
        
        # Clean up GPIO
        if not self.simulation_mode:
            if self.motor_controller:
                self.motor_controller.cleanup()
            GPIO.cleanup()
        
        # Close OpenCV windows
        cv2.destroyAllWindows()
        
        print("Average FPS:", f"{self.avg_fps:.2f}")


def main():
    """Main function"""
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description="4-Wheel Rover Center Tracking with Fuzzy Logic Control",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    # Detection parameters
    parser.add_argument('--model', type=str,
                       help="Path to YOLO model file (e.g., 'yolov8n.pt')")
    parser.add_argument('--source', type=str, default="picamera0",
                       help="Camera source: 'picamera0', 'usb0', or video file path")
    parser.add_argument('--confidence', type=float, default=DEFAULT_CONFIDENCE,
                       help="Confidence threshold for detections")
    parser.add_argument('--resolution', type=str, default=DEFAULT_RESOLUTION,
                       help="Camera resolution (WxH)")
    parser.add_argument('--track-class', type=str, default=None,
                       help="Class name to track (default: track first detected object)")
    
    # Distance parameters
    parser.add_argument('--target-distance', type=int, default=TARGET_DISTANCE,
                       help="Target distance to maintain from object in cm")
    parser.add_argument('--max-distance', type=int, default=DEFAULT_MAX_DISTANCE,
                       help="Maximum estimated distance in cm")
    
    # Offset interval parameter
    parser.add_argument('--offset-interval', type=float, default=OFFSET_INTERVAL,
                       help="Angle interval (±) to consider object as centered, in degrees")
    
    # Simulation parameters
    parser.add_argument('--simulation', action='store_true',
                       help="Run in simulation mode (no hardware control)")
    parser.add_argument('--sim-distance', type=float, default=None,
                       help="Simulated distance in cm (default: random)")
    
    args = parser.parse_args()
    
    # Run center tracker
    tracker = CenterTracker(args)
    tracker.run()


if __name__ == "__main__":
    main()

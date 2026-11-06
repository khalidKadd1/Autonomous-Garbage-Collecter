import cv2
import mediapipe as mp
import numpy as np
import json
import time
import asyncio
import websockets
import threading

class HandTracker:
    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils
        self.mp_style = mp.solutions.drawing_styles
        self.prev_lm = None
        
        # Joint control commands
        self.joint_commands = {
            0: "stop",  # Base rotation
            1: "stop",  # Shoulder
            2: "stop",  # Elbow
            3: "stop",  # Wrist pitch
            4: "stop",  # Wrist roll
            5: "stop",  # Gripper
        }
        
        # Previous finger state to detect changes
        self.prev_fingers_state = [0, 0, 0, 0, 0]
        
        # Current gesture and confidence
        self.current_gesture = "none"
        self.gesture_time = 0
        self.min_gesture_time = 5  # Minimum frames to hold a gesture before recognizing it

        # Last sent gesture to prevent duplicate sends
        self.last_sent_gesture = "none"
        self.last_sent_finger_code = -1

    def process_frame(self, frame):
        data = {
            "gesture": "none",
            "finger_code": 0,
            "joint_commands": self.joint_commands.copy()
        }
        
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = self.hands.process(rgb)
        
        if not res.multi_hand_landmarks:
            # Reset when no hand detected
            self.current_gesture = "none"
            self.gesture_time = 0
            
            # Reset all commands to stop
            for joint in self.joint_commands:
                self.joint_commands[joint] = "stop"
                
            return frame, data

        lm = res.multi_hand_landmarks[0]
        self.mp_draw.draw_landmarks(
            frame, lm, self.mp_hands.HAND_CONNECTIONS,
            self.mp_style.get_default_hand_landmarks_style(),
            self.mp_style.get_default_hand_connections_style()
        )

        # Extract normalized landmarks
        lms = [[p.x, p.y, p.z] for p in lm.landmark]
        
        # Get finger states
        fs = self.get_fingers_state(lms)
        
        # Create binary finger code (thumb is MSB, pinky is LSB)
        finger_code = (fs[0] << 4) | (fs[1] << 3) | (fs[2] << 2) | (fs[3] << 1) | fs[4]
        data["finger_code"] = finger_code
        
        # Detect gesture based on finger state
        gesture = self.detect_gesture(fs)
        
        # Update gesture tracking
        if gesture == self.current_gesture:
            self.gesture_time += 1
        else:
            self.current_gesture = gesture
            self.gesture_time = 0
            
            # Reset commands when gesture changes
            for joint in self.joint_commands:
                self.joint_commands[joint] = "stop"
        
        # Only use stable gestures
        if self.gesture_time >= self.min_gesture_time:
            data["gesture"] = gesture
            
            # Process the single gesture to update joint commands
            self.process_single_gesture(fs, finger_code)
        else:
            # Keep commands as "stop" while gesture is stabilizing
            for joint in self.joint_commands:
                self.joint_commands[joint] = "stop"
        
        # Update joint command in data
        data["joint_commands"] = self.joint_commands.copy()

        # Save current finger state for next comparison
        self.prev_fingers_state = fs.copy()

        # Overlay text information
        self.overlay_info(frame, gesture, fs, finger_code)

        return frame, data

    def process_single_gesture(self, fs, finger_code):
        # Reset all commands
        for joint in self.joint_commands:
            self.joint_commands[joint] = "stop"
            
        # Fist - stop all movements
        if finger_code == 0:
            return
        
        # Open palm - open gripper
        if finger_code == 31:  # 11111 in binary
            self.joint_commands[5] = "open"
            return
        
        # All fingers except thumb - close gripper
        if finger_code == 15:  # 01111 in binary
            self.joint_commands[5] = "close"
            return
        
        # Shoulder: Index only
        if finger_code == 8:  # 01000 in binary
            self.joint_commands[1] = "up"
            return
        
        # Shoulder: Index + Middle
        if finger_code == 12:  # 01100 in binary
            self.joint_commands[1] = "down"
            return
        
        # Elbow: Thumb only
        if finger_code == 16:  # 10000 in binary
            self.joint_commands[2] = "up"
            return
        
        # Elbow: Thumb + Index
        if finger_code == 24:  # 11000 in binary
            self.joint_commands[2] = "down"
            return
        
        # Wrist Pitch: Pinky only
        if finger_code == 1:  # 00001 in binary
            self.joint_commands[3] = "up"
            return
        
        # Wrist Pitch: Pinky + Index
        if finger_code == 9:  # 01001 in binary
            self.joint_commands[3] = "down"
            return
        
        # Wrist Roll: Thumb + Index + Pinky (REPLACED Middle only)
        if finger_code == 25:  # 11001 in binary (thumb, index, pinky up)
            self.joint_commands[4] = "right"
            return
        
        # Wrist Roll: Thumb + Pinky
        if finger_code == 17:  # 10001 in binary
            self.joint_commands[4] = "left"
            return
        
        # Base Rotation: Thumb + Index + Middle
        if finger_code == 28:  # 11100 in binary
            self.joint_commands[0] = "right"
            return
        
        # Base Rotation: Pinky + Ring + Middle
        if finger_code == 7:  # 00111 in binary
            self.joint_commands[0] = "left"
            return

    def overlay_info(self, frame, gesture, fs, finger_code):
        # Display gesture and hold time
        stability = "STABLE" if self.gesture_time >= self.min_gesture_time else "Stabilizing..."
        cv2.putText(frame, f"Gesture: {gesture} ({stability})", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Display fingers state and binary code
        finger_str = "".join(map(str, fs))
        cv2.putText(frame, f"Fingers: {finger_str} (Code: {finger_code:05b})", 
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Display joint commands
        y_pos = 90
        for joint, command in self.joint_commands.items():
            joint_name = ["Base", "Shoulder", "Elbow", "Wrist Pitch", "Wrist Roll", "Gripper"][joint]
            if command != "stop":
                color = (0, 0, 255) if command in ["up", "right", "open"] else (255, 0, 0)
            else:
                color = (0, 255, 0)
                
            cv2.putText(frame, f"{joint_name}: {command.upper()}", 
                        (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            y_pos += 30

    def get_fingers_state(self, lm):
        s = [0] * 5
        # thumb
        if lm[4][0] > lm[2][0]:
            s[0] = 1
        # other fingers
        tips = [8, 12, 16, 20]
        pips = [6, 10, 14, 18]
        for i, (t, p) in enumerate(zip(tips, pips), start=1):
            if lm[t][1] < lm[p][1]:
                s[i] = 1
        return s

    def detect_gesture(self, s):
        finger_code = (s[0] << 4) | (s[1] << 3) | (s[2] << 2) | (s[3] << 1) | s[4]
        
        # Map finger codes to gesture names
        if finger_code == 0:
            return "fist"
        elif finger_code == 16:  # 10000
            return "thumb_only"
        elif finger_code == 8:   # 01000
            return "index_only"
        elif finger_code == 4:   # 00100
            return "middle_only"
        elif finger_code == 2:   # 00010
            return "ring_only"
        elif finger_code == 1:   # 00001
            return "pinky_only"
        elif finger_code == 24:  # 11000
            return "thumb_index"
        elif finger_code == 20:  # 10100
            return "thumb_middle"
        elif finger_code == 17:  # 10001
            return "thumb_pinky"
        elif finger_code == 25:  # 11001 - NEW GESTURE
            return "thumb_index_pinky"
        elif finger_code == 12:  # 01100
            return "index_middle"
        elif finger_code == 9:   # 01001
            return "index_pinky"
        elif finger_code == 6:   # 00110
            return "middle_ring"
        elif finger_code == 7:   # 00111
            return "middle_ring_pinky"
        elif finger_code == 28:  # 11100
            return "thumb_index_middle"
        elif finger_code == 31:  # 11111
            return "open_palm"
        elif finger_code == 15:  # 01111
            return "all_but_thumb"
        else:
            return f"custom_{finger_code:05b}"


class WebSocketClient:
    def __init__(self, uri):
        self.uri = uri
        self.websocket = None
        self.connected = False
        self.loop = None
        self.thread = None
        self.running = False
        
    def start(self):
        """Start WebSocket client in a separate thread"""
        if self.thread is not None and self.thread.is_alive():
            return  # Already running
        
        self.running = True
        self.thread = threading.Thread(target=self._run_client)
        self.thread.daemon = True
        self.thread.start()
        
    def _run_client(self):
        """Run the asyncio event loop in a separate thread"""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        
        try:
            self.loop.run_until_complete(self._connect_and_run())
        except Exception as e:
            print(f"WebSocket error: {e}")
        finally:
            self.loop.close()
    
    async def _connect_and_run(self):
        """Connect to WebSocket server and keep reconnecting if disconnected"""
        while self.running:
            try:
                async with websockets.connect(self.uri) as websocket:
                    self.websocket = websocket
                    self.connected = True
                    print(f"Connected to ESP32 at {self.uri}")
                    
                    # Keep connection alive until disconnected
                    try:
                        async for message in websocket:
                            print(f"Received: {message}")
                    except websockets.exceptions.ConnectionClosed:
                        print("Connection closed")
                        
                    self.connected = False
                    self.websocket = None
            except Exception as e:
                print(f"Connection error: {e}")
                self.connected = False
                self.websocket = None
                
            # Wait before trying to reconnect
            if self.running:
                await asyncio.sleep(2)
    
    def send_message(self, message):
        """Send a message to the WebSocket server"""
        if not self.connected or self.websocket is None or self.loop is None:
            print("Not connected")
            return False
        
        # Convert message to JSON string
        json_message = json.dumps(message)
        
        # Run the coroutine in the event loop
        asyncio.run_coroutine_threadsafe(
            self._async_send(json_message), 
            self.loop
        )
        return True
    
    async def _async_send(self, message):
        """Send a message asynchronously"""
        if self.websocket is None:
            return
            
        try:
            await self.websocket.send(message)
        except Exception as e:
            print(f"Send error: {e}")
            self.connected = False
            self.websocket = None
    
    def stop(self):
        """Stop the WebSocket client"""
        self.running = False
        
        if self.loop is not None and self.thread is not None and self.thread.is_alive():
            # Schedule a task to close the connection
            if self.websocket is not None:
                asyncio.run_coroutine_threadsafe(
                    self._async_close(), 
                    self.loop
                )
            
            # Wait for the thread to finish
            self.thread.join(timeout=1.0)
    
    async def _async_close(self):
        """Close the WebSocket connection asynchronously"""
        if self.websocket is not None:
            await self.websocket.close()


def open_webcam():
    # Try AVFoundation (macOS), generic, and alternate indices
    for (idx, backend) in [(0, cv2.CAP_AVFOUNDATION),
                          (1, cv2.CAP_AVFOUNDATION),
                          (0, cv2.CAP_ANY),
                          (1, cv2.CAP_ANY),
                          (2, cv2.CAP_ANY)]:
        cap = cv2.VideoCapture(idx, backend)
        if not cap.isOpened():
            continue
        # set resolution
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # force MJPG (may help)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        # test a frame
        ret, frame = cap.read()
        if ret and frame is not None and frame.any():
            print(f"Opened camera index {idx} with backend {backend}")
            return cap
        cap.release()
    return None


def main():
    # Ask for ESP32 IP address
    esp32_ip = input("Enter ESP32 IP address: ")
    websocket_uri = f"ws://{esp32_ip}:81"
    
    # Initialize WebSocket client
    ws_client = WebSocketClient(websocket_uri)
    ws_client.start()
    
    # Wait a moment for connection attempt
    time.sleep(2)
    
    # Initialize camera
    cap = open_webcam()
    if cap is None:
        print("ERROR: Unable to open any webcam device.")
        ws_client.stop()
        return

    # Initialize hand tracker
    tracker = HandTracker()
    
    # Display control instructions
    print("IMPROVED HAND GESTURE ROBOT CONTROL")
    print("===================================")
    print("UNIVERSAL CONTROLS:")
    print("  - Fist (all fingers closed): STOP ALL JOINTS")
    print("")
    print("JOINT CONTROLS (ONE AT A TIME):")
    print("  - Shoulder:")
    print("    * Index only (01000): Move Up")
    print("    * Index + Middle (01100): Move Down")
    print("")
    print("  - Elbow:")
    print("    * Thumb only (10000): Move Up")
    print("    * Thumb + Index (11000): Move Down")
    print("")
    print("  - Wrist Pitch:")
    print("    * Pinky only (00001): Move Up")
    print("    * Pinky + Index (01001): Move Down")
    print("")
    print("  - Wrist Roll:")
    print("    * Thumb + Index + Pinky (11001): Roll Right")
    print("    * Thumb + Pinky (10001): Roll Left")
    print("")
    print("  - Base Rotation:")
    print("    * Thumb + Index + Middle (11100): Rotate Right")
    print("    * Pinky + Ring + Middle (00111): Rotate Left")
    print("")
    print("  - Gripper:")
    print("    * All fingers (11111): Open Gripper")
    print("    * All except thumb (01111): Close Gripper")
    print("")
    print("Hold a gesture steady to activate it")
    print("Press 'q' to quit")
    
    # For command throttling
    last_command_time = time.time()
    command_interval = 0.5  # seconds between sending the same command again
    
    # Track last sent data
    last_sent_finger_code = -1
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                print("Failed to grab frame — reconnecting...")
                cap.release()
                cap = open_webcam()
                if cap is None:
                    break
                continue

            frame = cv2.flip(frame, 1)
            processed, data = tracker.process_frame(frame)
            
            # Current time for command throttling
            current_time = time.time()
            
            # Only send stable gestures and limit by command interval
            gesture = data["gesture"]
            finger_code = data["finger_code"]
            
            if ((gesture != "none" and finger_code != last_sent_finger_code) or 
                current_time - last_command_time >= command_interval):
                
                # Send the gesture and finger code
                message = {
                    "gesture": gesture,
                    "finger_code": finger_code,
                    "joint_commands": data["joint_commands"]
                }
                
                if ws_client.send_message(message):
                    last_sent_finger_code = finger_code
                    last_command_time = current_time
            
            cv2.imshow("Robot Arm Control", processed)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # Clean up
        # Send stop commands for all joints
        stop_message = {
            "gesture": "fist",
            "finger_code": 0,
            "joint_commands": {
                "0": "stop", "1": "stop", "2": "stop", 
                "3": "stop", "4": "stop", "5": "stop"
            }
        }
        ws_client.send_message(stop_message)
        
        # Give time for the stop command to be sent
        time.sleep(0.5)
        
        # Close everything
        cap.release()
        cv2.destroyAllWindows()
        ws_client.stop()


if __name__ == "__main__":
    main()
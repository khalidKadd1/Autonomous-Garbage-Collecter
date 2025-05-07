#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

// WiFi credentials - Replace with your network details
const char* ssid = "Polito";
const char* password = "12345678";

// WebSocket server on port 81
WebSocketsServer webSocket = WebSocketsServer(81);

// Define the pins for the 6 servo joints - ESP32 compatible pins
const int SERVO_PINS[6] = {
  13,  // Base rotation
  12,  // Shoulder
  14,  // Elbow
  27,  // Wrist Pitch
  26,  // Wrist Roll
  25   // Gripper
};

// Create servo objects
Servo servos[6];

// SERVO ANGLE RANGES
// Adjust these to align the physical positions of joints
// NOTE: For the elbow (index 2), the range is REVERSED (180-0) to match shoulder alignment
const int MIN_ANGLE[6] = {
  0,    // Base rotation: 0° is minimum
  0,    // Shoulder: 0° is minimum
  180,  // Elbow: 180° is minimum (REVERSED to align with shoulder)
  0,    // Wrist Pitch: 0° is minimum
  0,    // Wrist Roll: 0° is minimum
  0     // Gripper: 0° is minimum
};

const int MAX_ANGLE[6] = {
  180,  // Base rotation: 180° is maximum 
  180,  // Shoulder: 180° is maximum
  0,    // Elbow: 0° is maximum (REVERSED to align with shoulder)
  180,  // Wrist Pitch: 180° is maximum
  180,  // Wrist Roll: 180° is maximum
  180   // Gripper: 180° is maximum
};

// INITIAL SERVO POSITIONS
int INITIAL_POSITIONS[6] = {
  150,   // Base rotation: middle position
  150,   // Shoulder: middle position 
  150,   // Elbow: middle position
  150,   // Wrist Pitch: middle position
  20,   // Wrist Roll: middle position
  90    // Gripper: middle position
};

// Current servo positions (in degrees) - initialized to starting positions
int servoPositions[6] = {0, 0, 0, 0, 0, 0};  // Will be set to INITIAL_POSITIONS in setup()

// Movement increment (degrees to move per update cycle)
int MOVEMENT_INCREMENT[6] = {
  2,  // Base rotation
  2,  // Shoulder 
  2,  // Elbow
  2,  // Wrist Pitch
  2,  // Wrist Roll
  2   // Gripper
};

// Command state tracking
String currentCommands[6] = {"stop", "stop", "stop", "stop", "stop", "stop"};

// Timing variables
unsigned long lastUpdateTime = 0;
const int UPDATE_INTERVAL = 50;  // 50ms for faster response (20 updates per second)

// Last received gesture
String lastGesture = "none";

// Debug mode - set to true for more detailed console output
bool DEBUG_MODE = true;

// WebSocket event handler
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      // Stop all servos when client disconnects
      for (int i = 0; i < 6; i++) {
        currentCommands[i] = "stop";
      }
      break;
      
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
        
        // Send a welcome message with current servo positions
        String positionInfo = "Connected to ESP32 - Current positions: ";
        for (int i = 0; i < 6; i++) {
          positionInfo += String(servoPositions[i]);
          if (i < 5) positionInfo += ", ";
        }
        webSocket.sendTXT(num, positionInfo);
      }
      break;
      
    case WStype_TEXT:
      {
        // Convert payload to string
        String message = String((char *) payload);
        if (DEBUG_MODE) {
          Serial.printf("[%u] Received text: %s\n", num, message.c_str());
        }
        
        // Parse JSON
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, message);
        
        if (error) {
          Serial.print("deserializeJson() failed: ");
          Serial.println(error.c_str());
          return;
        }
        
        // Check if we have a detected gesture
        if (doc.containsKey("gesture")) {
          String currentGesture = doc["gesture"].as<String>();
          
          // Only process if the gesture is different from last time
          if (currentGesture != lastGesture) {
            if (DEBUG_MODE) {
              Serial.print("New Gesture: ");
              Serial.println(currentGesture);
            }
            lastGesture = currentGesture;
            
            // Reset all commands to stop first
            for (int i = 0; i < 6; i++) {
              currentCommands[i] = "stop";
            }
            
            // Process the single gesture
            if (doc.containsKey("finger_code")) {
              int fingerCode = doc["finger_code"].as<int>();
              processSingleGesture(fingerCode);
            }
          }
        }
        
        // Direct joint commands (for emergency stop, etc.)
        if (doc.containsKey("joint_commands")) {
          JsonObject commands = doc["joint_commands"];
          
          // Update commands for each joint
          for (int i = 0; i < 6; i++) {
            String jointStr = String(i);
            if (commands.containsKey(jointStr)) {
              String movement = commands[jointStr].as<String>();
              currentCommands[i] = movement;
              
              if (DEBUG_MODE) {
                Serial.print("Direct Command - Joint ");
                Serial.print(i);
                Serial.print(": ");
                Serial.println(movement);
              }
            }
          }
        }
      }
      break;
  }
}

// Process single gesture based on finger code (binary representation of fingers)
void processSingleGesture(int fingerCode) {
  // Extract individual finger states
  bool thumb = (fingerCode & 16) > 0;  // 10000 in binary
  bool index = (fingerCode & 8) > 0;   // 01000 in binary
  bool middle = (fingerCode & 4) > 0;  // 00100 in binary
  bool ring = (fingerCode & 2) > 0;    // 00010 in binary
  bool pinky = (fingerCode & 1) > 0;   // 00001 in binary
  
  if (DEBUG_MODE) {
    Serial.print("Finger states: T:");
    Serial.print(thumb);
    Serial.print(" I:");
    Serial.print(index);
    Serial.print(" M:");
    Serial.print(middle);
    Serial.print(" R:");
    Serial.print(ring);
    Serial.print(" P:");
    Serial.println(pinky);
  }
  
  // Fist - stop all movements
  if (fingerCode == 0) {
    for (int i = 0; i < 6; i++) {
      currentCommands[i] = "stop";
    }
    return;
  }
  
  // Open palm - open gripper
  if (fingerCode == 31) {  // 11111 in binary
    currentCommands[5] = "open";
    return;
  }
  
  // All fingers except thumb - close gripper
  if (fingerCode == 15) {  // 01111 in binary
    currentCommands[5] = "close";
    return;
  }
  
  // Shoulder: Index only
  if (fingerCode == 8) {  // 01000 in binary
    currentCommands[1] = "up";
    return;
  }
  
  // Shoulder: Index + Middle
  if (fingerCode == 12) {  // 01100 in binary
    currentCommands[1] = "down";
    return;
  }
  
  // Elbow: Thumb only
  if (fingerCode == 16) {  // 10000 in binary
    currentCommands[2] = "up";
    return;
  }
  
  // Elbow: Thumb + Index
  if (fingerCode == 24) {  // 11000 in binary
    currentCommands[2] = "down";
    return;
  }
  
  // Wrist Pitch: Pinky only
  if (fingerCode == 1) {  // 00001 in binary
    currentCommands[3] = "up";
    return;
  }
  
  // Wrist Pitch: Pinky + Index
  if (fingerCode == 9) {  // 01001 in binary
    currentCommands[3] = "down";
    return;
  }
  
  // Wrist Roll: Middle only
  if (fingerCode == 4) {  // 00100 in binary
    currentCommands[4] = "right";
    return;
  }
  
  // Wrist Roll: Thumb + Pinky
  if (fingerCode == 17) {  // 10001 in binary
    currentCommands[4] = "left";
    return;
  }
  
  // Base Rotation: Thumb + Index + Middle
  if (fingerCode == 28) {  // 11100 in binary
    currentCommands[0] = "right";
    return;
  }
  
  // Base Rotation: Pinky + Ring + Middle
  if (fingerCode == 7) {  // 00111 in binary
    currentCommands[0] = "left";
    return;
  }
}

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  Serial.println("\nESP32 Robotic Arm Controller Starting...");
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  // Wait for connection
  int connectionAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && connectionAttempts < 30) {
    delay(500);
    Serial.print(".");
    connectionAttempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("Connected to WiFi, IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("(Use this IP address in the Python control program)");
  } else {
    Serial.println();
    Serial.println("Failed to connect to WiFi. Check your credentials.");
    // Continue anyway, maybe we're using direct USB connection
  }
  
  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server started on port 81");
  
  // ESP32 specific: allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  // Initialize all servos
  Serial.println("Initializing servos...");
  
  for (int i = 0; i < 6; i++) {
    // ESP32 specific: set servo parameters for full range
    servos[i].setPeriodHertz(50);    // Standard 50hz servo
    
    // Adjust pulse width range for maximum servo compatibility
    servos[i].attach(SERVO_PINS[i], 500, 2500); 
    
    // Validate and set initial position based on min/max angles
    if (MIN_ANGLE[i] < MAX_ANGLE[i]) {
      // Normal range (min < max)
      INITIAL_POSITIONS[i] = constrain(INITIAL_POSITIONS[i], MIN_ANGLE[i], MAX_ANGLE[i]);
    } else {
      // Reversed range (min > max)
      INITIAL_POSITIONS[i] = constrain(INITIAL_POSITIONS[i], MAX_ANGLE[i], MIN_ANGLE[i]);
    }
    
    // Copy initial positions to current positions
    servoPositions[i] = INITIAL_POSITIONS[i];
    
    // Move to initial position
    servos[i].write(servoPositions[i]);
    
    // Log
    Serial.print("Joint ");
    Serial.print(i + 1);
    Serial.print(" initialized on pin ");
    Serial.print(SERVO_PINS[i]);
    Serial.print(" at position ");
    Serial.print(servoPositions[i]);
    Serial.print("° (Range: ");
    Serial.print(MIN_ANGLE[i]);
    Serial.print("°-");
    Serial.print(MAX_ANGLE[i]);
    
    if (MIN_ANGLE[i] > MAX_ANGLE[i]) {
      Serial.print(" REVERSED");
    }
    
    Serial.println("°)");
  }
  
  // Wait for servos to reach initial position
  delay(1500);
  
  Serial.println("Servos initialized");
  Serial.println("Ready to receive commands");
  
  // Display control mapping
  Serial.println("\nGesture Control Mapping:");
  Serial.println("- Fist (00000): Stop all joints");
  Serial.println("- Shoulder: Index only (01000) UP, Index+Middle (01100) DOWN");
  Serial.println("- Elbow: Thumb only (10000) UP, Thumb+Index (11000) DOWN");
  Serial.println("- Wrist Pitch: Pinky only (00001) UP, Pinky+Index (01001) DOWN");
  Serial.println("- Wrist Roll: Middle only (00100) RIGHT, Thumb+Pinky (10001) LEFT");
  Serial.println("- Base: Thumb+Index+Middle (11100) RIGHT, Pinky+Ring+Middle (00111) LEFT");
  Serial.println("- Gripper: All fingers (11111) OPEN, All except thumb (01111) CLOSE");
}

void loop() {
  // Handle WebSocket events
  webSocket.loop();
  
  // Update servo positions at regular intervals based on current commands
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
    updateServos();
    lastUpdateTime = currentTime;
  }
}

// Update all servo positions based on current commands
void updateServos() {
  for (int joint = 0; joint < 6; joint++) {
    String direction = currentCommands[joint];
    
    // Skip if command is stop
    if (direction == "stop") {
      continue;
    }
    
    // Get current position
    int currentPos = servoPositions[joint];
    int newPos = currentPos;
    
    // Check if this joint uses reversed range (min > max)
    bool isReversedRange = (MIN_ANGLE[joint] > MAX_ANGLE[joint]);
    
    // Calculate new position based on direction and range type
    if (direction == "up" || direction == "right" || direction == "open") {
      if (isReversedRange) {
        // For reversed range, "up" means moving towards MAX (which is the smaller value)
        newPos = max(MAX_ANGLE[joint], currentPos - MOVEMENT_INCREMENT[joint]);
      } else {
        // For normal range, "up" means moving towards MAX (which is the larger value)
        newPos = min(MAX_ANGLE[joint], currentPos + MOVEMENT_INCREMENT[joint]);
      }
    }
    else if (direction == "down" || direction == "left" || direction == "close") {
      if (isReversedRange) {
        // For reversed range, "down" means moving towards MIN (which is the larger value)
        newPos = min(MIN_ANGLE[joint], currentPos + MOVEMENT_INCREMENT[joint]);
      } else {
        // For normal range, "down" means moving towards MIN (which is the smaller value)
        newPos = max(MIN_ANGLE[joint], currentPos - MOVEMENT_INCREMENT[joint]);
      }
    }
    
    // Only update if position has changed
    if (newPos != currentPos) {
      // Update position
      servoPositions[joint] = newPos;
      
      // Move the servo
      servos[joint].write(newPos);
      
      // Log the movement (but less frequently to avoid serial flooding)
      if (DEBUG_MODE && newPos % 10 == 0) {  // Log only every 10 degrees
        String jointNames[] = {"Base", "Shoulder", "Elbow", "Wrist Pitch", "Wrist Roll", "Gripper"};
        Serial.print(jointNames[joint]);
        Serial.print(" ");
        Serial.print(direction);
        if (isReversedRange) {
          Serial.print(" (reversed range)");
        }
        Serial.print(": position ");
        Serial.print(newPos);
        Serial.print("° (");
        Serial.print(MIN_ANGLE[joint]);
        Serial.print("-");
        Serial.print(MAX_ANGLE[joint]);
        Serial.println("°)");
      }
    } 
    else {
      // If we've reached a limit, stop the movement
      if ((isReversedRange && (newPos == MAX_ANGLE[joint] || newPos == MIN_ANGLE[joint])) ||
          (!isReversedRange && (newPos == MIN_ANGLE[joint] || newPos == MAX_ANGLE[joint]))) {
        
        currentCommands[joint] = "stop";
        
        if (DEBUG_MODE) {
          String limitType;
          if (isReversedRange) {
            limitType = (newPos == MAX_ANGLE[joint]) ? "minimum" : "maximum";
          } else {
            limitType = (newPos == MIN_ANGLE[joint]) ? "minimum" : "maximum";
          }
          
          Serial.print("Joint ");
          Serial.print(joint);
          Serial.print(" reached ");
          Serial.print(limitType);
          Serial.println(" limit - stopped");
        }
      }
    }
  }
}
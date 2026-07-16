import processing.serial.*;

Serial myPort;
float qI, qJ, qK, qReal;
int axisLength = 200; 

// Viewport interaction variables
float zoom = 0;      // Controls Z-translation (zoom depth)
float rotX = 0;      // Camera rotation around X-axis
float rotY = 0;      // Camera rotation around Y-axis

void setup() {
  size(800, 600, P3D);
  smooth(8); 

  // List all available serial ports
  printArray(Serial.list());
  
  // Ensure this index matches your Teensy's COM port
  String portName = Serial.list()[0]; 
  myPort = new Serial(this, portName, 115200);
  
  myPort.bufferUntil('\n'); 
}

void draw() {
  background(30); 
  lights();       

  // 1. VIEWPORT TRANSFORMATIONS (Camera Zoom & Rotation)
  translate(width/2, height/2, zoom);
  rotateX(rotX);
  rotateY(rotY);
  
  // --- DRAW EARTH-FIXED (STATIC) AXES ---
  // Mapped to a Right-Handed System: X=Right, Y=Forward, Z=Up
  strokeWeight(2);
  
  // Fixed X Axis: Right (Maps to Screen +X)
  drawDashedLine(0, 0, 0, axisLength, 0, 0, color(150, 50, 50));
  // Fixed Y Axis: Forward (Maps to Screen -Z)
  drawDashedLine(0, 0, 0, 0, 0, -axisLength, color(50, 150, 50));
  // Fixed Z Axis: Up (Maps to Screen -Y)
  drawDashedLine(0, 0, 0, 0, -axisLength, 0, color(50, 50, 150));

  // --- 2. SENSOR TRANSFORMATIONS (IMU Orientation)
  float angle = 2.0 * acos(qReal);
  float s = sqrt(1.0 - (qReal * qReal));
  
  float ax = 0, ay = 0, az = 0;
  
  if (s > 0.001) { 
    ax = qI / s;
    ay = qJ / s;
    az = qK / s;
  } else {
    ax = qI;
    ay = qJ;
    az = qK;
  }

  // MAP LOGICAL RIGHT-HANDED AXES TO PROCESSING'S LEFT-HANDED SCREEN
  float rx = ax;       // Logical X (Right) -> Screen +X
  float ry = -az;      // Logical Z (Up) -> Screen -Y
  float rz = -ay;      // Logical Y (Forward) -> Screen -Z

  // Push the matrix state to isolate the IMU rotation
  pushMatrix();
  
  // Apply rotation using the mapped axes and a negated angle to force RHR
  rotate(-angle, rx, ry, rz);

  // --- DRAW SENSOR (ROTATING) AXES ---
  strokeWeight(5);
  
  // Rotating X Axis: Right (Solid Red) -> Screen +X
  stroke(255, 50, 50);
  line(0, 0, 0, axisLength, 0, 0);
  
  // Rotating Y Axis: Forward (Solid Green) -> Screen -Z
  stroke(50, 255, 50);
  line(0, 0, 0, 0, 0, -axisLength); 
  
  // Rotating Z Axis: Up (Solid Blue) -> Screen -Y
  stroke(50, 50, 255);
  line(0, 0, 0, 0, -axisLength, 0);
  
  // Central reference hub
  noStroke();
  fill(200);
  sphere(15);
  
  popMatrix(); // Restore matrix to viewport-only state
}

// Custom function to draw a 3D dashed line
void drawDashedLine(float x1, float y1, float z1, float x2, float y2, float z2, color c) {
  stroke(c);
  float distance = dist(x1, y1, z1, x2, y2, z2);
  int dashLength = 8; 
  int segments = ceil(distance / dashLength);
  
  for (int i = 0; i < segments; i++) {
    if (i % 2 == 0) { 
      float startX = lerp(x1, x2, (float)i / segments);
      float startY = lerp(y1, y2, (float)i / segments);
      float startZ = lerp(z1, z2, (float)i / segments);
      
      float endX = lerp(x1, x2, (float)(i + 1) / segments);
      float endY = lerp(y1, y2, (float)(i + 1) / segments);
      float endZ = lerp(z1, z2, (float)(i + 1) / segments);
      
      line(startX, startY, startZ, endX, endY, endZ);
    }
  }
}

// --- INPUT EVENT LISTENERS ---

// Handles mouse dragging to rotate the entire camera field of view
void mouseDragged() {
  rotY += (mouseX - pmouseX) * 0.01;
  rotX -= (mouseY - pmouseY) * 0.01; 
}

// Handles mouse wheel scrolling to zoom in and out
void mouseWheel(MouseEvent event) {
  float count = event.getCount();
  zoom -= count * 25; 
}

// Parse incoming Teensy serial data
void serialEvent(Serial myPort) {
  String inString = myPort.readStringUntil('\n');
  
  if (inString != null) {
    inString = trim(inString); 
    float[] values = float(split(inString, ',')); 
    
    if (values.length == 4) {
      qI = values[0];
      qJ = values[1];
      qK = values[2];
      qReal = values[3];
    }
  }
}

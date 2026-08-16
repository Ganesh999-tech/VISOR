import cv2
import numpy as np
import apriltag
import time

# ==========================================
# 1. ASSUMPTIONS & CONFIGURATION
# ==========================================
# ASSUMPTION 1: The physical size of the AprilTag is exactly 15 cm (0.15 meters).
TAG_SIZE = 0.15 

# ASSUMPTION 2: Using generic camera intrinsic parameters for a 640x480 resolution.
# For production, you must calibrate the Evetar M118B029528IR to get exact values.
focal_length_x = 600.0
focal_length_y = 600.0
center_x = 320.0
center_y = 240.0

# Build the Camera Intrinsic Matrix
camera_matrix = np.array([
    [focal_length_x, 0, center_x],
    [0, focal_length_y, center_y],
    [0, 0, 1]
], dtype=np.float32)

# ASSUMPTION 3: We assume zero lens distortion for this prototype.
dist_coeffs = np.zeros((4, 1))

# Define the 3D coordinates of the tag's corners in its own local coordinate system.
half_size = TAG_SIZE / 2.0
object_points = np.array([
    [-half_size,  half_size, 0.0],
    [ half_size,  half_size, 0.0],
    [ half_size, -half_size, 0.0],
    [-half_size, -half_size, 0.0]
], dtype=np.float32)

# ==========================================
# 2. INITIALIZATION
# ==========================================
detector = apriltag.Detector()
cam = cv2.VideoCapture(0)

if not cam.isOpened():
    print("Error: Could not open camera on the Jetson. Check your USB connection.")
    exit()

print("V.I.S.O.R. Initialized. Running in Headless Mode over SSH...")
print("Looking for AprilTags... (Press Ctrl+C to stop)")

frame_count = 0

try:
    while True:
        ret, image = cam.read()
        if not ret:
            print("Failed to grab frame.")
            break
        
        grayimg = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # ==========================================
        # 3. 2D IMAGE CENTER CALCULATION
        # ==========================================
        height, width = image.shape[:2]
        frame_center_x = int(width / 2)
        frame_center_y = int(height / 2)
        
        cv2.drawMarker(image, (frame_center_x, frame_center_y), (255, 0, 0), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)

        detections = detector.detect(grayimg)
        
        if len(detections) > 0:
            for detect in detections:
                # ==========================================
                # 4. 2D OFFSET CALCULATION & VISUALIZATION
                # ==========================================
                tag_center_x, tag_center_y = int(detect.center[0]), int(detect.center[1])
                pixel_offset_x = tag_center_x - frame_center_x
                pixel_offset_y = tag_center_y - frame_center_y
                
                cv2.arrowedLine(image, (frame_center_x, frame_center_y), (tag_center_x, tag_center_y), (0, 0, 255), 2)
                cv2.putText(image, f"Offset: X:{pixel_offset_x}px, Y:{pixel_offset_y}px", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                for i in range(4):
                    pt1 = (int(detect.corners[i][0]), int(detect.corners[i][1]))
                    pt2 = (int(detect.corners[(i + 1) % 4][0]), int(detect.corners[(i + 1) % 4][1]))
                    cv2.line(image, pt1, pt2, (255, 0, 255), 2)

                # ==========================================
                # 5. 3D POSE ESTIMATION (PHYSICAL POSITION)
                # ==========================================
                image_points = np.array(detect.corners, dtype=np.float32)
                success, rotation_vector, translation_vector = cv2.solvePnP(
                    object_points, image_points, camera_matrix, dist_coeffs
                )
                
                if success:
                    x_pos = translation_vector[0][0]
                    y_pos = translation_vector[1][0]
                    z_pos = translation_vector[2][0]
                    
                    cv2.putText(image, f"3D Pos: X:{x_pos:.2f}m Y:{y_pos:.2f}m Z:{z_pos:.2f}m", (tag_center_x - 50, tag_center_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    # Print to SSH console so you can see live data without a GUI
                    print(f"Tag ID {detect.tag_id} Detected! | 2D Offset: {pixel_offset_x}px, {pixel_offset_y}px | 3D Distance: Z={z_pos:.2f}m")

            # Save the frame to disk only when a tag is detected
            cv2.imwrite("visor_output.jpg", image)
            
        frame_count += 1
        
        # Add a tiny delay to avoid overwhelming the terminal or CPU
        time.sleep(0.05)

except KeyboardInterrupt:
    # This safely stops the camera when you press Ctrl+C in your Windows SSH terminal
    print("\nCtrl+C pressed. Stopping V.I.S.O.R. loop.")

# Clean up
cam.release()
print("Camera released. Exiting.")

import cv2
import numpy as np
import time

# ==========================================
# 1. CONFIGURATION & ASSUMPTIONS
# ==========================================
# ASSUMPTION: The hopper is currently hovering at a fixed altitude of 10 meters.
ALTITUDE_Z = 10.0 # meters

# Camera Intrinsic (Focal length in pixels)
FOCAL_LENGTH = 600.0 

# Parameters for Shi-Tomasi corner detection (finding good features to track)
feature_params = dict(maxCorners=100, 
                      qualityLevel=0.3, 
                      minDistance=7, 
                      blockSize=7)

# Parameters for Lucas-Kanade optical flow
lk_params = dict(winSize=(15, 15), 
                 maxLevel=2, 
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# ==========================================
# 2. INITIALIZATION
# ==========================================
cam = cv2.VideoCapture(0)

if not cam.isOpened():
    print("Error: Could not open camera. Check your connection.")
    exit()

# Read the very first frame to find initial features
ret, old_frame = cam.read()
if not ret:
    print("Error: Could not read from camera.")
    exit()
    
old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)

# Create a mask image for drawing the optical flow tracks (the "tails" behind features)
mask = np.zeros_like(old_frame)

print("V.I.S.O.R. Optical Flow Initialized with GUI.")
print("Press 'q' or 'ESC' in the video window to stop.")

prev_time = time.time()

while True:
    ret, frame = cam.read()
    if not ret:
        break
        
    current_time = time.time()
    dt = current_time - prev_time
    prev_time = current_time
    
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # ==========================================
    # 3. OPTICAL FLOW CALCULATIONs
    # ==========================================
    if p0 is not None:
        # Calculate optical flow using Lucas-Kanade
        p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
        
        if p1 is not None:
            # Select good points (status == 1 means feature was successfully tracked)
            good_new = p1[st == 1]
            good_old = p0[st == 1]
            
            # Calculate pixel displacement for all tracked features
            if len(good_new) > 0:
                displacements = good_new - good_old
                
                # Take the median displacement to ignore noisy/erroneous tracks
                median_disp = np.median(displacements, axis=0)
                dp_x, dp_y = median_disp[0], median_disp[1]
                
                # ==========================================
                # 4. CONVERT PIXELS TO PHYSICAL VELOCITY
                # ==========================================
                # Formula: D = (pixels * Z) / focal_length
                dx_meters = (dp_x * ALTITUDE_Z) / FOCAL_LENGTH
                dy_meters = (dp_y * ALTITUDE_Z) / FOCAL_LENGTH
                
                # Velocity: V = D / dt
                vx = dx_meters / dt if dt > 0 else 0
                vy = dy_meters / dt if dt > 0 else 0
                
                # ==========================================
                # 5. LIVE VISUALIZATION (GUI)
                # ==========================================
                # Draw the tracks on the mask and overlay on the current frame
                for i, (new, old) in enumerate(zip(good_new, good_old)):
                    a, b = int(new[0]), int(new[1])
                    c, d = int(old[0]), int(old[1])
                    # Draw the tracking line (tail)
                    mask = cv2.line(mask, (a, b), (c, d), (0, 255, 0), 2)
                    # Draw the current feature point
                    frame = cv2.circle(frame, (a, b), 5, (0, 0, 255), -1)
                    
                img_output = cv2.add(frame, mask)
                
                # Add velocity data text to the live image
                cv2.putText(img_output, f"Vx: {vx:.2f} m/s", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                cv2.putText(img_output, f"Vy: {vy:.2f} m/s", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                cv2.putText(img_output, f"Features: {len(good_new)}", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Update previous points and frame for the next iteration
            old_gray = frame_gray.copy()
            p0 = good_new.reshape(-1, 1, 2)
            
            # If we lose too many features (e.g., if the camera moves too fast), re-detect them
            if len(good_new) < 10:
                p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)
                mask = np.zeros_like(old_frame) # Clear the old tracking lines

    # Show the live video window on the connected display
    cv2.imshow('V.I.S.O.R. - Lateral Drift Estimation', img_output)
    
    # Wait 1ms for a key press. If 'q' or 'ESC' (27) is pressed, break the loop
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') or key == 27:
        break

# Clean up windows and release camera
cam.release()
cv2.destroyAllWindows()
print("Camera released. Exiting.")

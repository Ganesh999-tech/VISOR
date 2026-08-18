
# V.I.S.O.R. (Vision-based Intelligent Sensing for Odometry and Recognition)

This repository contains a vision-based navigation and precision landing prototype developed for a thrust-vectoring hopper. The system is designed to run on a Jetson Orin Nano companion computer. Due to hardware availability during the prototyping phase, a Logitech C270 HD webcam was utilized for initial testing and validation.

## Repository Structure
*   `Task_1_April_Tag_Detection.py`: Headless implementation of the precision landing pipeline for SSH environments.
*   `Task_1_April_Tag_Detection_GUI.py`: GUI-enabled implementation of the precision landing pipeline.
*   `Task_2_Lateral_Drift_Estimation_GUI.py`: GUI-enabled implementation of the optical-flow drift estimation pipeline.
*   `visor_output.jpg`: Sample visualization output from the headless Tag Detection script.

## Installation & Reproducibility
To ensure a clean execution of these scripts, please install the following Python dependencies. The system was tested using Python 3.

**Required Libraries:**
`pip install opencv-python numpy apriltag`

**Execution:**
*   To run the precision landing pipeline: `python3 Task_1_April_Tag_Detection_GUI.py`
*   To run the lateral drift estimation: `python3 Task_2_Lateral_Drift_Estimation_GUI.py`

*Note: Ensure a webcam is connected and accessible by the operating system before execution. Press 'q' or 'ESC' to terminate the video windows.*

---

## Task 1: Downward Camera - Precision Landing 

### Objective
Develop a prototype using a downward-facing camera to detect an AprilTag, identify it, determine its offset relative to the image center, and estimate the 3D relative position of the landing target.

### Approach & Implementation
The system utilizes the `apriltag` Python library for robust marker recognition. Once a tag is detected in the video stream, the system calculates the 2D pixel offset between the frame's absolute center and the tag's center. 

To estimate the relative physical position of the landing target, the system solves the Perspective-n-Point (PnP) problem using OpenCV's `cv2.solvePnP`. This algorithm takes the known 3D geometry of the tag, the 2D image points, and the camera intrinsic matrix to mathematically output the translation vector (X, Y, Z in meters). 

### Engineering Trade-off: 3D Pose Estimation
| Method | Computational Cost | Accuracy on Known Target | Robustness to Vibration | Engineering Verdict |
| :--- | :--- | :--- | :--- | :--- |
| **PnP (Monocular)** | Very Low | Very High (Sub-centimeter) | High | **Selected.** Provides mathematically stable pose extraction using minimal CPU cycles. |
| **Homography Decomposition** | Low | High  | Medium | Rejected. PnP (specifically iterative Levenberg-Marquardt) is mathematically more stable. |

### Assumptions & Limitations
*   **Tag Size:** The physical AprilTag size is assumed to be exactly 15 cm (0.15m).
*   **Camera Intrinsics:** Generic 640x480 intrinsic parameters were used for the prototype (Focal Length: 600px). For production, the Evetar lens must undergo checkerboard calibration to extract exact focal lengths and distortion coefficients.
*   **Planar Assumption:** We assume the landing pad and camera sensor are perfectly parallel.

---

## Task 2: Downward Camera - Lateral Drift Estimation

### Objective
Investigate how the downward-facing camera can be used to estimate lateral motion/drift using an optical-flow-based approach.

### Approach & Implementation
To ensure high-frequency state estimation, a Sparse Optical Flow architecture was implemented. The system uses the Shi-Tomasi corner detector (`cv2.goodFeaturesToTrack`) to find high-contrast visual features on the ground. The Lucas-Kanade method (`cv2.calcOpticalFlowPyrLK`) is then used to track these specific features between frames to estimate lateral velocity.

### Mathematical Conversion (Pixels to Physical Motion)
To convert image motion into a physical motion estimate, similar triangle pinhole geometry is applied:
1.  **Displacement:** Let $ \Delta p $ be the tracked feature displacement in pixels, $ Z $ be the altitude in meters, and $ f $ be the camera focal length in pixels. The physical displacement $ \Delta D $ in meters is: 
    $$ \Delta D = \frac{\Delta p \cdot Z}{f} $$
2.  **Velocity:** If $ \Delta t $ is the time elapsed between frames (derived from frame rate), the lateral velocity $ V $ is: 
    $$ V = \frac{\Delta D}{\Delta t} $$

### Engineering Trade-off: Motion Estimation
| Method | Compute Cost | Output | Handling of Featureless Ground | Engineering Verdict |
| :--- | :--- | :--- | :--- | :--- |
| **Sparse Flow (Lucas-Kanade)** | Very Low | Sparse global vector | Fails without contrast | **Selected.** Isolates optimal features to deliver necessary velocity vectors at high frame rates. |
| **Dense Flow (Farneback)** | Medium/High | Dense pixel map | Smooths over empty areas | Rejected. Calculating motion for every pixel wastes critical compute resources. |

### Assumptions & Limitations
*   **Constant Altitude:** The translation math currently assumes a static altitude of 10.0 meters. In production, $ Z $ must dynamically ingest real-time data from an altimeter or the AprilTag PnP output.
*   **Vanishing Point & False Perspective:** The code assumes flat terrain and a perfectly horizontal camera. If the hopper tilts (pitch/roll), perspective distortion causes ground features further away to appear to move slower. In a production environment, this is mitigated by fusing IMU data to apply a Homography Warp, artificially "flattening" the image before calculating flow.

---

## Task 3: Forward-Facing Camera - Navigation Architecture

While the downward camera specializes in ground-texture tracking and terminal precision landing, the forward-facing camera serves as a secondary navigation system with distinct advantages:

*   **Information Provided:** The forward camera can detect the horizon line for absolute attitude estimation, identify upcoming vertical obstacles, and track distant landmarks for global localization.
*   **Differences from Downward Camera:** The downward camera suffers from motion blur at high speeds and low altitudes. The forward camera observes objects at a greater distance, meaning angular displacement across the sensor is slower, yielding more stable feature tracking during rapid transit.
*   **Visual-Inertial Odometry (VIO) Application:** Yes, the forward camera is highly suited for VIO. By tracking features moving across its field of view, the system can calculate relative changes in position.
*   **IMU Integration:** A camera alone struggles with scale ambiguity (knowing how big or far an object is) and high-frequency motion. By tightly coupling the camera with an IMU (which measures raw acceleration and angular velocity), the IMU bridges the gap between camera frames, while the camera corrects the IMU's inherent drift. 
*   **Motion Estimates Provided:** A fused VIO system provides a complete 6-Degrees-of-Freedom (6-DOF) state estimate (X, Y, Z translation and Roll, Pitch, Yaw rotation).

---

## Task 4: Camera Configuration Analysis

The proposed hardware includes the downward Evetar M118B029528IR (Approx. FOV: 91° H × 75° V) and the forward Evetar M118B0418IR (Approx. FOV: 77° H × 62° V). 

**Suitability for 50-100m Altitude:**
This configuration is well-suited for a maximum flight altitude of 50-100 meters. 
*   The downward camera's wide 91° horizontal FOV provides a massive ground footprint at 100 meters, ensuring the AprilTag remains in view even if the hopper drifts laterally during descent. 
*   The forward camera's narrower 77° FOV provides better pixel density and angular resolution for tracking distant features.
*   Both lenses feature IR-cut filters, which are critical for outdoor daytime flights to prevent infrared sunlight from washing out the sensor and degrading contrast. 

**Factors Affecting Measurements & Placement:**
*   **Vibration:** Both cameras must be mechanically isolated on the hopper structure. High-frequency thrust vibrations will induce rolling shutter distortion or blur, severely degrading optical flow and AprilTag detection.
*   **Plume Interference:** The downward camera must be placed on an outrigger or away from the central engine exhaust to prevent exhaust plumes or kicked-up dust from occluding the lens during terminal descent.

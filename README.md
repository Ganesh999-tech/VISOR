# V.I.S.O.R. (Vision-based Intelligent Sensing for Odometry and Recognition)

This repository contains a vision-based navigation and precision landing prototype developed for a thrust-vectoring hopper. The system is designed to run on a Jetson Orin Nano companion computer. Due to hardware availability during the prototyping phase, a Logitech C270 HD webcam was utilized for initial testing and validation.

## Repository Structure

* `Task_1_April_Tag_Detection.py`: Headless implementation of the precision landing pipeline for SSH environments.
* `Task_1_April_Tag_Detection_GUI.py`: GUI-enabled implementation of the precision landing pipeline.
* `Task_2_Lateral_Drift_Estimation_GUI.py`: GUI-enabled implementation of the optical-flow drift estimation pipeline.
* `visor_output.jpg`: Sample visualization output from the headless Tag Detection script.

## Installation & Reproducibility

To ensure a clean execution of these scripts, please install the following Python dependencies. The system was tested using Python 3.

**Required Libraries:**

```bash
pip install opencv-python numpy apriltag

```

**Execution:**

* To run the precision landing pipeline: `python3 Task_1_April_Tag_Detection_GUI.py`
* To run the lateral drift estimation: `python3 Task_2_Lateral_Drift_Estimation_GUI.py`

*Note: Ensure a webcam is connected and accessible by the operating system before execution. Press 'q' or 'ESC' to terminate the video windows.*

---

## Task 1: Downward Camera - Precision Landing

### Objective

Develop a prototype using a downward-facing camera to detect an AprilTag, identify it, determine its offset relative to the image center, and estimate the 3D relative position of the landing target.

### Approach & Implementation

The system utilizes the `apriltag` Python library for robust marker recognition. Once a tag is detected, the system extracts the 2D pixel coordinates of its four distinct corners and computes the offset relative to the frame center.

To resolve the physical 3D pose, the system solves the **Perspective-n-Point (PnP)** problem using OpenCV's `cv2.solvePnP`.

#### Fundamental Concept: How PnP Works

 PnP is a **geometric optimization method**. It bridges 2D image coordinates with a known 3D world model:
 <img width="600" height="420" alt="image" src="https://github.com/user-attachments/assets/42603b20-1834-4b5b-9df9-25e6597529f4" />
 
 
 Image Ref : https://docs.opencv.org/4.11.0/d5/d1f/calib3d_solvePnP.html


1. **The Intrinsic "Cone of Vision":** The camera intrinsics act as a known mathematical lens model, allowing the algorithm to trace straight rays from 2D pixel coordinates out into 3D space.
2. **Iterative Optimization:** Using the known physical dimensions of the tag (e.g., 15 cm width), `cv2.solvePnP` uses an iterative optimization loop (such as Levenberg-Marquardt). It continually adjusts the camera's position ($X, Y, Z$) and orientation ($Roll, Pitch, Yaw$) until the projection of the 3D model points matches the detected 2D pixel corners.



<iframe width="560" height="315" src="https://www.youtube.com/embed/ZwF-pKAVIJM?si=WsEaPaUM1mxtWl9y" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>



### Engineering Trade-off: 3D Pose Estimation

| Method | Computational Cost | Accuracy on Known Target | Robustness to Vibration | Engineering Verdict |
| --- | --- | --- | --- | --- |
| **PnP (Monocular)** | Very Low | Very High (Sub-centimeter) | High | **Selected.** Provides mathematically stable pose extraction using minimal CPU cycles. |
| **Homography Decomposition** | Low | High | Medium | Rejected. PnP is mathematically more robust against noise and non-coplanar shifts. |

### Assumptions & Limitations

* **Tag Size:** The physical AprilTag size is assumed to be exactly 15 cm ($0.15\,\text{m}$).
* **Camera Intrinsics:** Generic 640x480 intrinsic parameters were used for the prototype ($f_x = f_y = 600\,\text{px}$). For production, the Evetar lens must undergo checkerboard calibration to extract exact focal lengths and distortion coefficients. The Basler AG is providing opton to configure camera using Pylon tool software for vision solutions.

---

## Task 2: Downward Camera - Lateral Drift Estimation

### Objective

Investigate how the downward-facing camera can be used to estimate lateral motion/drift using an optical-flow-based approach.

### Approach & Implementation

To track high-frequency movement, a Sparse Optical Flow architecture is deployed.

* **Feature Detection:** The Shi-Tomasi corner detector (`cv2.goodFeaturesToTrack`) isolates high-contrast corners on the ground plane.
* **Intensity Tracking:** The Lucas-Kanade method (`cv2.calcOpticalFlowPyrLK`) is utilized. Unlike PnP, Lucas-Kanade is an **intensity-based method**: it analyzes local patches of pixel brightness across consecutive frames to determine how patterns of light have shifted.

 <img width="437" height="194" alt="image" src="https://github.com/user-attachments/assets/3bfdc19a-8c31-452a-8f8c-75c353f70616" />
 
 
 It shows a ball moving in 5 consecutive frames. The arrow shows its displacement vector
 
 
 Image Ref : https://docs.opencv.org/3.4.8/d4/dee/tutorial_optical_flow.html


### Mathematical Conversion (Pixels to Physical Motion)

To transform raw pixel displacement into physical velocity, similar-triangle pinhole geometry is applied:

1. **Pixel Displacement:** Let $\Delta p = (\Delta p_x, \Delta p_y)$ be the median pixel shift of tracked features, $Z$ be the altitude in meters, and $f$ be the focal length in pixels. The physical displacement $\Delta D$ in meters is computed as:

$$\Delta D_x = \frac{\Delta p_x \cdot Z}{f}, \quad \Delta D_y = \frac{\Delta p_y \cdot Z}{f}$$


2. **Velocity Vector:** Factoring in the elapsed time $\Delta t$ between frames, the lateral velocity components are derived:

$$V_x = \frac{\Delta D_x}{\Delta t}, \quad V_y = \frac{\Delta D_y}{\Delta t}$$



<iframe width="560" height="315" src="https://www.youtube.com/embed/A1_DGnf8FEs?si=oO4G3J5YHkDBkXne" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>



### Engineering Trade-off: Motion Estimation

| Method | Compute Cost | Output | Handling of Featureless Ground | Engineering Verdict |
| --- | --- | --- | --- | --- |
| **Sparse Flow (Lucas-Kanade)** | Very Low | Sparse global vector | Fails without contrast | **Selected.** Isolates optimal features to deliver necessary velocity vectors at high frame rates. |
| **Dense Flow (Farneback)** | Medium/High | Dense pixel map | Smooths over empty areas | Rejected. Calculating motion for every pixel wastes critical compute resources. |

### Assumptions & Limitations

* **Constant Altitude:** The translation math assumes a static altitude of $Z = 10.0\,\text{m}$. In production, $Z$ must dynamically ingest altitude data from an altimeter or the AprilTag PnP output.
* **Planar Surface Constraint:** Optical flow assumes the tracked surface is parallel to the image plane. If the hopper tilts (pitch/roll), perspective distortion alters pixel velocity across the frame. In a production environment, this is mitigated by fusing IMU orientation data to apply a Homography warp.

---

## Task 3: Forward-Facing Camera - Navigation Architecture

While the downward camera specializes in ground-texture tracking and terminal precision landing, the forward-facing camera serves as a secondary navigation system with distinct advantages:

* **The forward camera:** The forward camera detects the horizon line for absolute attitude estimation, identifies upcoming vertical obstacles, and tracks distant landmarks for global localization.
* **Differences from Downward Camera:** The downward camera suffers from motion blur at high speeds and low altitudes. The forward camera observes objects at a greater distance, meaning angular displacement across the sensor is slower, yielding more stable feature tracking during rapid transit.
* **Visual-Inertial Odometry (VIO) Application:** Yes, the forward camera is highly suited for VIO. By tracking features moving across its field of view, the system calculates relative changes in position.
* **IMU Integration:** A camera alone struggles with scale ambiguity and high-frequency motion. By tightly coupling the camera with an IMU, the IMU bridges the gap between camera frames, while the camera corrects the IMU's inherent drift, providing a complete 6-Degrees-of-Freedom (6-DOF) state estimate ($X, Y, Z$ translation and $Roll, Pitch, Yaw$ rotation).

---

## Task 4: Camera Configuration Analysis

The proposed hardware includes the downward Evetar M118B029528IR (Approx. FOV: 91° H × 75° V) and the forward Evetar M118B0418IR (Approx. FOV: 77° H × 62° V).

**Suitability for 50-100m Altitude:**
This configuration is well-suited for a maximum flight altitude of 50-100 meters.

* The downward camera's wide 91° horizontal FOV provides a massive ground footprint at 100 meters, ensuring the AprilTag remains in view even if the hopper drifts laterally during descent.
* The forward camera's narrower 77° FOV provides better pixel density and angular resolution for tracking distant features.
* Both lenses feature IR-cut filters, which are critical for outdoor daytime flights to prevent infrared sunlight from washing out the sensor and degrading contrast.

**Factors Affecting Measurements & Placement:**

* **Vibration:** Both cameras must be mechanically isolated on the hopper structure. High-frequency thrust vibrations induce rolling shutter distortion or blur, severely degrading optical flow and AprilTag detection.
* **Plume Interference:** The downward camera must be placed on an outrigger or away from the central engine exhaust to prevent exhaust plumes or kicked-up dust from occluding the lens during terminal descent.

---

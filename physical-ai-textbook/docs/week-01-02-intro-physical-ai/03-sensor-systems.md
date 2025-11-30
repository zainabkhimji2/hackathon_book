---
title: "Sensor Systems in Robotics"
sidebar_position: 3
description: "Understand the major sensor types (LIDAR, cameras, IMU) and sensor fusion"
keywords: [sensors, LIDAR, cameras, IMU, sensor fusion, robotics perception]
---

# Sensor Systems in Robotics

**Section Objective:** Understand how robots perceive their environment through diverse sensor types and how sensor fusion creates robust perception.

**Estimated Reading Time:** 12 minutes
**Estimated Word Count:** 2,000 words

---

## Introduction: The Eyes and Ears of Robots

If the perception-action loop is the robot's nervous system, then sensors are its sensory organs. In this section, we examine the major sensor types used in robotics, their principles of operation, and how multiple sensors combine for robust perception.

**Key Question:** *Why does a robot need multiple sensor types when humans get by with just eyes and ears?*

Answer: Robots operate in diverse environments where single sensors fail. LIDAR works in darkness where cameras fail. Cameras see colors where LIDAR cannot. IMU tracks motion when nothing visual changes. Robustness comes from diversity.

:::info Learning Path
This section progresses from individual sensor principles → practical use cases → sensor fusion mechanisms. By end, you'll understand tradeoffs in sensor selection and why ROS 2 systems integrate multiple sensor streams.
:::

---

## 2.1 LIDAR: Light Detection and Ranging

### 2.1.1 Operating Principle

**LIDAR** emits laser light and measures the time it takes for reflections to return.

```
Laser Emitter → Pulse travels distance d →
Reflects off object → Returns → Time-of-flight measured →
Distance calculated: d = (c * t) / 2
where c = speed of light, t = round-trip time
```

**Range Formula:**
```
Distance = (speed of light × time) / 2
Example: Light takes 20 nanoseconds round-trip
Distance = (3×10^8 m/s × 20×10^-9 s) / 2 = 3 meters
```

### 2.1.2 LIDAR Types

**2D LIDAR (Scanning Laser Rangefinder)**

| Characteristic | Value |
|---|---|
| **Field of View** | 270° horizontal only |
| **Range** | 0.5 - 100 meters (depending on model) |
| **Output** | 2D distance profile (like a cross-section) |
| **Applications** | Wheeled robot navigation, indoor mapping |
| **Processing Speed** | 25 Hz typical |

**Example Use Case: Indoor Robot Navigation**
- Robot spins LIDAR and sees 360° distances around it
- Creates a 2D "lidar slice" showing obstacles at each angle
- Used for collision avoidance, SLAM (Simultaneous Localization and Mapping)

:::note Visualization
Imagine looking down at a robot from above. A 2D LIDAR shows what's around it as a circle of distance measurements—like sonar in 2D.
:::

**3D LIDAR (Spinning/Mechanical)**

| Characteristic | Value |
|---|---|
| **Field of View** | 360° horizontal × 20-40° vertical |
| **Range** | 0.5 - 200+ meters |
| **Output** | 3D point cloud (x, y, z coordinates) |
| **Applications** | Autonomous vehicles, autonomous flying robots, complex 3D mapping |
| **Processing Speed** | 10-20 Hz typical; produces 1M+ points per scan |

**Example Use Case: Autonomous Vehicle**
- Spins 64-128 laser beams vertically
- In 100 milliseconds, produces 1-2 million points
- Represents 3D structure of environment (cars, pedestrians, road)

**3D LIDAR (Solid-State)**

| Characteristic | Value |
|---|---|
| **Field of View** | 120-180° horizontal × 20-40° vertical |
| **Range** | 0.1 - 150 meters |
| **Output** | 3D point cloud |
| **Applications** | Emerging; autonomous vehicles, robotics |
| **Advantages** | No moving parts, smaller, more durable |
| **Disadvantages** | Higher cost, less mature |

### 2.1.3 LIDAR Strengths

1. **Darkness-Invariant:** Works equally well in sunlight or darkness (emits its own light)
2. **Metric Accuracy:** Provides precise distance measurements (±5-10 cm typical)
3. **3D Structure:** Captures full 3D geometry (for 3D LIDAR)
4. **Fast Processing:** Geometric structure is easy to process (finding obstacles)
5. **Range:** Detects objects 50+ meters away

### 2.1.4 LIDAR Limitations

1. **No Semantic Information:** Sees "there's something 5 meters away" but not "it's a person"
2. **Sparse:** Especially at range—far objects have few points
3. **Weather Sensitivity:** Rain and fog scatter laser light; range degrades
4. **Computational Load:** 3D LIDAR produces massive data (megabytes per second)
5. **Reflective Surfaces:** Mirrors and glass confuse LIDAR (reflections create false returns)
6. **Cost:** Quality 3D LIDAR systems cost $5,000-$70,000
7. **Low Vertical Resolution:** 3D LIDAR might only have 64 vertical beams

:::warning Practical Limitation
A 3D LIDAR might detect a 1-meter-tall object clearly but miss a 10-cm-tall curb. Vertical resolution at range is limited.
:::

### 2.1.5 LIDAR Use Cases

| Application | Why LIDAR | Complementary Need |
|---|---|---|
| **SLAM (robot mapping)** | Geometric precision | Object recognition (camera) |
| **Autonomous vehicles** | 360° coverage, darkness | Semantic understanding (camera) |
| **Obstacle avoidance** | Metric accuracy | Real-world robustness (multiple LIDAR) |
| **3D reconstruction** | Full geometry | Color/texture (camera) |
| **Terrain analysis** | Detailed elevation | Object type (camera) |

---

## 2.2 Camera Systems

### 2.2.1 Monocular Camera

**Basic Operation:**
```
3D World → Lens projects onto 2D image sensor →
Pixels capture light intensity →
Software interprets pixels
```

**Characteristics:**

| Aspect | Detail |
|---|---|
| **Output** | 2D image array (height × width × 3 for color) |
| **Resolution** | 480p to 4K typical (typically 1080p = 1920×1080 pixels) |
| **Frame Rate** | 30-60 Hz typical |
| **Field of View** | 30-120° typical (depends on lens) |
| **Cost** | $50-500 |

**Critical Limitation: Depth Ambiguity**

A monocular camera loses 3D depth information. Consider:
```
Far small object --projects to--> Small area in image
Near large object --projects to--> Small area in image
```

The camera sees the same pixel pattern but objects are different distances and sizes. This is an ambiguous inverse problem.

**Practical Consequence:** Monocular systems must either:
1. Use motion (moving camera reveals depth through parallax)
2. Use learning (train on data to predict depth)
3. Accept depth uncertainty

### 2.2.2 Stereo Camera

**Operating Principle:**

```
Object in space ↓
Left camera sees it at angle L
Right camera sees it at angle R
Difference in angles (disparity) → depth from stereo triangulation
```

**Triangulation Formula:**
```
Depth = (Baseline × Focal_Length) / Disparity
where:
  Baseline = distance between cameras (typically 60-120 mm)
  Focal_Length = camera lens property (known)
  Disparity = pixel difference between left and right images
```

**Characteristics:**

| Aspect | Detail |
|---|---|
| **Output** | Stereo image pair + depth map (disparity map) |
| **Depth Accuracy** | ±5-10% of measured distance |
| **Range** | 0.3 - 10+ meters (depends on baseline & resolution) |
| **Frame Rate** | 30-60 Hz typical |
| **Computational Cost** | Moderate (must match pixels left-right) |
| **Cost** | $200-1000 |

**Stereo Advantages:**
1. Provides true metric depth (no learning required)
2. Works in any lighting (passive—uses ambient light)
3. Relatively low cost for metric depth
4. Can be computed in real-time on modest hardware

**Stereo Limitations:**
1. Requires texture (homogeneous surfaces fail—can't find matches)
2. Baseline determines range vs. resolution tradeoff
3. Occlusions create depth "holes"
4. Large objects at distance have low depth resolution

:::tip Trade-off
Wider baseline (further apart cameras) = longer range but bulkier system. Close baseline = better nearby resolution but fails at distance.
:::

### 2.2.3 RGB-D Camera (Depth + Color)

**Operating Principle:** Combines monocular color video with depth from various technologies:
- **Infrared Structured Light:** Projector emits IR pattern; camera detects distortion
- **Time-of-Flight (ToF):** Emits IR pulses and measures return time (similar to LIDAR)

**Popular Systems:**
- Intel RealSense (structured light + stereo variants)
- Azure Kinect (ToF)
- iPhone Face ID (ToF)

**Characteristics:**

| Aspect | Detail |
|---|---|
| **Output** | Color image + depth map (per-pixel depth) |
| **Depth Range** | 0.2 - 5 meters typical (much closer than LIDAR) |
| **Color Resolution** | 1080p typical |
| **Depth Resolution** | 480p typical (lower than color) |
| **Frame Rate** | 30 Hz typical |
| **Cost** | $100-300 |

**RGB-D Advantages:**
1. Provides both color and metric depth
2. Works indoors (IR structured light not affected by ambient light)
3. Relatively low cost
4. High quality for near-field objects

**RGB-D Limitations:**
1. Very short range (5 meters max for indoor IR cameras)
2. Fails outdoors (infrared interfered by sunlight)
3. Fails with reflective surfaces
4. Depth is noisy; many missing values
5. Not suitable for autonomous vehicles (too short range)

:::warning Outdoor Failure
If you take an RGB-D camera outdoors, the depth sensor fails because solar infrared noise overwhelms the signal. This is why autonomous vehicles use LIDAR or stereo, not RGB-D.
:::

### 2.2.4 Camera Strengths and Limitations

**Strengths:**
1. **Rich Semantics:** Can identify objects, people, text, colors
2. **Low Cost:** Cameras are mature, inexpensive technology
3. **Light Weight:** Small, power-efficient
4. **Interpretability:** Humans can visually inspect what camera sees

**Limitations:**
1. **Monocular Depth Ambiguity:** Depth from single camera requires learning or motion
2. **Lighting Sensitive:** Fails in darkness; saturates in bright light
3. **Motion Blur:** Fast motion causes blur; limits useful frame rate
4. **Latency from Processing:** Semantic understanding (object detection) adds 50-200 ms
5. **Specular Reflections:** Mirrors and shiny surfaces confuse algorithms

### 2.2.5 Camera Use Cases

| Application | Camera Type | Why Camera |
|---|---|---|
| **Object Identification** | Monocular | Semantic understanding from RGB |
| **Depth Estimation** | Stereo + Monocular learning | Metric depth for manipulation |
| **Visual Navigation** | Monocular + learning | Rich environmental understanding |
| **Face Recognition** | Monocular (or RGB-D outdoors) | Texture and pattern matching |
| **Quality Inspection** | RGB + structured light | Color and fine geometry |
| **Human-Robot Interaction** | Monocular (gesture, gaze) | Interpret human intentions |

---

## 2.3 IMU: Inertial Measurement Unit

### 2.3.1 Sensors Included

An IMU contains:
1. **3-Axis Accelerometer** — Measures acceleration in 3D space
2. **3-Axis Gyroscope** — Measures angular velocity (rotation rate)
3. **3-Axis Magnetometer** — Measures magnetic field (compass heading)

Some IMUs add barometer (altitude) or other sensors.

### 2.3.2 What Does Acceleration Mean?

**Critical Point:** Accelerometer measures apparent acceleration including gravity.

```
At rest on table:
  Accelerometer reads [0, 0, 9.81] m/s^2 (gravity)

In free fall (e.g., jumping):
  Accelerometer reads [0, 0, 0] (no apparent acceleration)

Driving in circle at constant speed:
  Accelerometer reads centripetal acceleration toward center
```

**Key Insight:** From accelerometer alone, you cannot determine if you're being pushed or if gravity changed direction.

### 2.3.3 IMU Strengths

1. **Low Latency:** Produces measurements at 100-1000 Hz (very fast)
2. **Orientation Tracking:** Gyroscope provides angular velocity; integrate over time to get heading
3. **Inertial Sensing:** Detects acceleration from robot's own motion (requires no external reference)
4. **Compact:** Small chip; can be placed anywhere on robot
5. **Low Cost:** $5-50 depending on quality
6. **Works Everywhere:** Works underground, indoors, outdoors (except magnetometer outdoors near metal)

### 2.3.4 IMU Limitations

1. **Bias and Drift:** Gyroscope bias causes integrated heading to drift over time
   - Typical drift: 1-5 degrees per minute
   - After 10 minutes: heading could be off by 50+ degrees

2. **Gravity Coupling:** Cannot distinguish acceleration from gravity change

3. **Noise:** All measurements are noisy; random fluctuations present

4. **Temperature Sensitive:** Bias changes with temperature

5. **No Position:** Accelerometers give velocity by integration, which drifts. Position drifts even faster.

:::warning Critical Limitation
An IMU alone cannot reliably track position. If you integrate accelerometer to get velocity and integrate again to get position, the error grows quadratically with time. After 10 seconds, position error could be meters-scale.
:::

### 2.3.5 IMU Use Cases

| Application | Why IMU | Requirement |
|---|---|---|
| **Orientation Tracking** | Gyroscope + magnetometer | Fused with camera or LIDAR for absolute reference |
| **Shock Detection** | Accelerometer sensitivity | Detects impacts, drops |
| **Inertial Navigation** | Acceleration integration | Fused with odometry (wheels) for drift correction |
| **Activity Recognition** | IMU patterns (walking, running) | Classification from acceleration patterns |
| **Stabilization** | Gyroscope feedback | High-speed balance control (quadrotor drones) |

:::note Example Fusion
A robot tracks position by wheel odometry (drifts slowly) + fuses with IMU (provides short-term accuracy). Combined estimate is more accurate than either alone.
:::

---

## 2.4 Comprehensive Sensor Comparison Table

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                           SENSOR COMPARISON MATRIX                                   │
├──────────────────┬─────────────┬──────────────┬──────────────┬──────────────────────┤
│    PROPERTY      │  2D LIDAR   │  3D LIDAR    │   STEREO     │   RGB-D (Indoor)     │
├──────────────────┼─────────────┼──────────────┼──────────────┼──────────────────────┤
│ Depth Accuracy   │ ±5-10 cm    │ ±5-10 cm     │ ±5-10% range │ ±1-3 cm (near)       │
│ Maximum Range    │ 30m         │ 200m+        │ 30m+         │ 5m                   │
│ Works in Dark    │ YES         │ YES          │ NO           │ YES (indoors)        │
│ Works Outdoors   │ YES (rain?) │ POOR (rain?) │ YES          │ NO (sunlight)        │
│ Semantic Info    │ None        │ None         │ None         │ Color available      │
│ Processing Speed │ Fast        │ Moderate     │ Moderate     │ Moderate             │
│ Cost             │ $500-3k     │ $5k-70k      │ $200-1k      │ $100-300             │
│ Spatial Res.     │ 360×1 2D    │ 360×64 3D    │ Full 2D      │ Full 2D (lower fps)  │
│ Key Strength     │ 360 coverage│ 3D map       │ Metric depth │ Color + depth combo  │
│ Key Limitation   │ 2D only     │ Sparse, cost │ Texture need │ Short range only     │
└──────────────────┴─────────────┴──────────────┴──────────────┴──────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────────────┐
│                      MORE SENSORS: IMU, ODOMETRY, TACTILE                           │
├──────────────────┬──────────────────┬──────────────────┬──────────────────────────┤
│    PROPERTY      │       IMU        │    ODOMETRY      │   FORCE/TACTILE          │
├──────────────────┼──────────────────┼──────────────────┼──────────────────────────┤
│ Measurement      │ Accel/rotation   │ Wheel rotation   │ Contact force, pressure  │
│ Output Rate      │ 100-1000 Hz      │ 10-100 Hz        │ 10-100 Hz                │
│ Latency          │ \<5 ms            │ \<10 ms           │ \<50 ms                   │
│ Key Strength     │ Fast, low drift  │ Odometry for pos │ Manipulation feedback    │
│ Key Limitation   │ Drifts over time │ Wheel slip error │ Limited spatial info     │
│ Typical Cost     │ $5-50            │ Built-in wheels  │ $100-1k                  │
└──────────────────┴──────────────────┴──────────────────┴──────────────────────────┘
```

---

## 2.5 Sensor Fusion: Combining Multiple Sensors

### 2.5.1 Why Sensor Fusion?

**Thesis:** No single sensor is perfect. Sensor fusion combines complementary sensors for robustness.

**Example: Indoor Robot Localization**

| Sensor | Good At | Bad At |
|---|---|---|
| **Wheel Odometry** | Accurate short-term position | Drifts; wheel slip unknown |
| **LIDAR** | Accurate heading; loop closure | Ambiguous positions (many walls look same) |
| **IMU** | Fast orientation; detects motion | Drifts; no position |
| **COMBINED** | Accurate position & heading | — |

By fusing these, the system gets benefits of each and limits weaknesses of each.

### 2.5.2 Fusion Approaches

**1. Complementary Filtering (Simple)**

Combine sensors in post-processing:
```
Estimate = 0.3 * Odometry + 0.5 * LIDAR + 0.2 * IMU
```

**Advantage:** Simple to implement
**Disadvantage:** Fixed weights; doesn't adapt to sensor failure

---

**2. Kalman Filter (Standard)**

Statistical approach that optimizes estimate given sensor uncertainties.

```
Prediction = (Odometry gives expected pose)
           + (IMU predicts motion direction)

Measurement = (LIDAR/Camera sees environment)

Fused Estimate = Weight * Prediction
               + (1-Weight) * Measurement
where Weight depends on each sensor's uncertainty
```

**Advantage:**
- Principled approach
- Automatically weights sensors based on uncertainty
- Adapts if sensor fails (becomes noisy)

**Disadvantage:**
- Assumes linear relationships; fails for non-linear problems
- Requires careful tuning

---

**3. Particle Filter (Flexible)**

Represent uncertainty as ensemble of candidate poses.

```
Maintain 1000 candidate robot poses
Update each based on all sensors
Resample (keep good poses, discard bad ones)
Result: Cloud of likely poses
```

**Advantage:**
- Handles non-linear problems
- Handles multi-modal uncertainty (multiple plausible locations)
- Robust to sensor failures

**Disadvantage:**
- Computationally expensive
- Requires many particles

---

**4. Deep Learning Fusion (Emerging)**

Train neural network to fuse sensors end-to-end.

```
Input: Raw sensor streams
     (camera images, point clouds, accelerometer, etc.)
Network: Learns which sensors to trust when
Output: Fused estimate (position, object locations, etc.)
```

**Advantage:**
- Can learn complex sensor relationships
- Potentially very robust
- End-to-end optimization

**Disadvantage:**
- Requires lots of training data
- Less interpretable (hard to debug)
- Sim-to-real transfer challenges

:::tip Practical Choice
For Week 3-4 ROS 2 projects, you'll likely use Kalman filtering (via existing libraries). Deep learning fusion is an advanced topic for Week 5-6.
:::

### 2.5.3 Classical Fusion Architecture: SLAM

**SLAM = Simultaneous Localization and Mapping**

Fuses odometry, LIDAR, and loop closure to build a map while tracking position.

```
Odometry (wheel rotation) → Rough position estimate
    ↓
LIDAR (geometric structure) → Local map + loop closure detection
    ↓
Loop Closure (revisit earlier location?) → Correct drift
    ↓
Refined pose + global map
```

**Why It Works:**
- Odometry is fast but drifts
- LIDAR is accurate but ambiguous
- Together: fast, accurate, and unambiguous

**You'll implement this in:** Week 4 with ROS 2's nav2 stack

---

## 2.6 Sensor Selection: Decision Framework

### 2.6.1 Sensor Selection Criteria

When designing a robot system, ask:

1. **Environment Characteristics**
   - Indoor or outdoor? (affects camera, RGB-D, LIDAR choice)
   - Static or dynamic? (affects sensor refresh rate needs)
   - Lighting conditions? (LIDAR/IMU advantage in darkness)

2. **Task Requirements**
   - Need semantic understanding? (cameras required)
   - Need precise depth? (stereo or LIDAR required)
   - Need orientation tracking? (IMU required)
   - Need 360° awareness? (LIDAR required)

3. **Real-Time Constraints**
   - Loop frequency needed? (determines sensor bandwidth)
   - Latency budget? (affects sensor + processing choice)

4. **Physical Constraints**
   - Weight budget? (affects sensor selection)
   - Power budget? (LIDAR expensive; cameras cheap)
   - Space constraints? (form factor matters)

5. **Cost Budget**
   - High-end (>$10k): 3D LIDAR + stereo + IMU + camera
   - Mid-range ($2-5k): 2D LIDAR + monocular camera + IMU
   - Budget (\<$1k): Monocular camera + IMU, or RGB-D indoors

### 2.6.2 Sensor Selection Examples

**Example 1: Warehouse Robot (Mobile Bin Picking)**

Requirements:
- Navigate warehouse (indoor)
- Pick objects (depth needed)
- Identify objects (semantics needed)
- Safety-critical (must detect obstacles reliably)

**Sensor Choice:**
- 2D LIDAR (navigation, obstacle avoidance)
- Stereo camera (depth for grasping)
- Monocular camera (object identification)
- IMU (stability feedback)

**Rationale:**
- 2D LIDAR sufficient for warehouse navigation (mostly flat, known structure)
- Stereo cheaper and more reliable than RGB-D (works in variable lighting)
- Separate monocular for high-res object detection
- IMU for stability

---

**Example 2: Autonomous Vehicle**

Requirements:
- Navigate city streets (outdoor)
- Detect pedestrians, cyclists, other vehicles (semantics)
- Precise 3D perception (track many objects)
- All-weather operation (rain, fog, night)

**Sensor Choice:**
- 3D LIDAR (primary perception for safety)
- Stereo camera (redundancy, texture)
- Monocular camera x4 (360° coverage for object detection)
- IMU + GPS + odometry (localization)

**Rationale:**
- 3D LIDAR primary because it works in rain/night and provides metric depth
- Stereo for backup; cameras for semantic detection (pedestrian faces, traffic lights)
- Multiple cameras for full 360° coverage
- GPS for global localization, IMU for fine motion

---

**Example 3: Household Robot Arm**

Requirements:
- Manipulate diverse objects (depth, semantics)
- Work in home environment (variable lighting)
- Low cost (consumer product)
- Real-time manipulation control (fast loops)

**Sensor Choice:**
- RGB-D camera (both color and depth; works indoors)
- Joint encoders (know arm position)
- Force/Torque sensors (manipulation feedback)
- IMU (arm stability)

**Rationale:**
- RGB-D perfect for indoor manipulation (cheap, compact, color + depth)
- Joint encoders essential for kinematics (know where arm is without vision)
- F/T sensors critical for safe grasping (detect when object slips)
- IMU for arm stability during high-speed motion

---

## 2.7 Sensor Noise and Uncertainty

### 2.7.1 Types of Sensor Error

**Systematic Error (Bias):**
- Consistent offset in measurements
- Example: LIDAR consistently reads 5 cm too far
- Can be calibrated out (if repeatable)

**Random Error (Noise):**
- Unpredictable fluctuations
- Example: LIDAR distance varies ±2 cm randomly
- Cannot be eliminated; must be filtered

**Outliers:**
- Rare, extreme errors
- Example: One in 1000 measurements is completely wrong (reflection, interference)
- Robust algorithms must reject outliers

### 2.7.2 Characterizing Sensor Uncertainty

**Standard Approach: Normal Distribution**

Assume sensor readings follow Gaussian distribution:
```
Probability of measurement = Gaussian(true_value, std_dev)
```

Example: LIDAR distance reading
```
True distance to wall = 5.00 meters
Sensor std_dev = 0.05 meters
Reading distribution: Normal(5.00, 0.05)
Meaning: ~68% of readings are 4.95-5.05 m,
         ~95% are 4.90-5.10 m
```

**Why This Matters:**
Sensor fusion algorithms (Kalman filters) assume Gaussian noise. If your sensor has non-Gaussian errors, fusion degrades.

### 2.7.3 Dealing with Uncertainty

**Strategy 1: Redundancy**
Use multiple sensors; fusion down-weights unreliable ones.

**Strategy 2: Filtering**
Smooth noisy measurements over time (exponential smoothing, Kalman filter).

**Strategy 3: Robust Algorithms**
Design algorithms that tolerate outliers (RANSAC, voting).

**Strategy 4: Environmental Design**
Control environment to reduce uncertainty:
- Better lighting for cameras
- Retro-reflectors on walls for LIDAR
- Flat surfaces for odometry

---

## Summary: Sensor Systems Foundation

In this section, you've learned:

1. **LIDAR** provides metric 3D geometry, works in darkness, but requires expensive hardware and provides no semantics
2. **Cameras** provide rich semantic information and color, but monocular systems have depth ambiguity
3. **Stereo cameras** provide metric depth without active emission; RGB-D combines color and depth indoors
4. **IMU** provides fast orientation and motion sensing; useful for control but drifts over time
5. **Sensor fusion** combines complementary sensors for robustness; SLAM is a practical example
6. **Sensor selection** depends on environment, task, real-time requirements, and budget

---

## Key Concepts Reference

- **LIDAR:** Laser-based range sensing; metric accuracy, darkness-invariant
- **Stereo Camera:** Metric depth from baseline + disparity matching
- **RGB-D:** Color + depth (short-range, indoors)
- **IMU:** Inertial sensing; fast but drifts
- **Sensor Fusion:** Combining complementary sensors for robust estimates
- **SLAM:** Simultaneous localization and mapping
- **Bias:** Systematic sensor error
- **Outliers:** Rare extreme errors

---

## Cross-References

- Previous Section: [Embodied Intelligence](./02-embodied-intelligence.md) — How perception-action loop uses sensors
- Next Section: [Physical vs. Digital AI](./04-physical-vs-digital-ai.md) — How sensor limitations create challenges
- Later Weeks: ROS 2 Sensor Integration (Weeks 3-4) — Implementing sensor drivers and fusion

---

## Reflection Questions

:::tip Think About This
1. A robot must navigate a warehouse at night. Which sensor would fail, and why would you choose a different sensor?
2. Why can't we just use a super-high-resolution camera instead of LIDAR?
3. If LIDAR has ±5cm accuracy and camera stereo has ±5% error, when would each be better?
4. How would you detect and reject LIDAR readings from a reflection (false positive)?

(Write 1-2 sentences for each)
:::

---

## Hands-On Thought Experiment

**Exercise:** Design a sensor suite for one of these robots:
1. Delivery robot navigating sidewalks
2. Indoor vacuum cleaner
3. Agricultural robot harvesting crops

For your chosen robot:
- List the three most critical tasks
- For each task, identify which sensor(s) are essential
- Justify your choices using the decision framework from 2.6.1
- Estimate approximate cost of your sensor suite

Write 1-2 paragraphs.

---

**Next:** [Section 1.4 - Physical vs. Digital AI Challenges](./04-physical-vs-digital-ai.md)

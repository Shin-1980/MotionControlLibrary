# Motion Controller Library

The Motion Controller library enables robots to perform dynamic movements by producing sequential motor positions over time. These positions are determined by a velocity profile, representing changes in velocity controlled by input parameters such as:

- Target position
- Target velocity
- Target acceleration
- Target deceleration
- Target jerk

## Features

- Generates smooth motion trajectories for robotic applications.
- Supports S-curve and trapezoidal velocity profiles.
- Provides modular design for easy integration and testing.

[Watch the video](https://youtu.be/Fnmq7zJaVu8)


## Process Overview

### Step 1: Copy Required Files
Copy the following files to your target folder:
- `CommandInfo.cpp/.h`
- `MotionController.cpp/.h`
- `ScurveProfile.cpp/.h`
- `TrajectoryProfile.cpp/.h`
- `TrapezoidalProfile.cpp/.h`

### Step 2: Set Up Parameters
Set up necessary parameters by following the example procedure in `TestSuite.cpp` (e.g., `testCase_ScurveProfile_101`).

#### Example Code:
```cpp
MotionController mc(dof);
mc.setScurveProfile(execProf1);
mc.setCurrentPose(curPose);
mc.setVelocityProfParam(targetPose, targetVelsRad, targetAccsRad, targetJerksRad);


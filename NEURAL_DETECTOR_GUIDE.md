# Neural Detector Pipeline for Fuel/Ball Counting

## Overview
This feature adds a neural detector pipeline to the Limelight camera system that can detect and count fuel balls in the robot's hopper during autonomous operation. The system intelligently controls shooting duration based on ball count to optimize autonomous performance.

## Features

### Pipeline Management
- **AprilTag Pipeline (0)**: Used for aim assist during teleop - provides horizontal alignment and distance control to AprilTag targets (IDs 10 and 26)
- **Neural Detector Pipeline (1)**: Used for ball counting - detects fuel balls (class ID 0) in the hopper

### Teleop Behavior
- **AprilTag Mode (Default)**: Provides full aim assist with distance control when aim assist is enabled (Y button)
- **Neural Mode**: Displays detected ball count on SmartDashboard only - no aim assist
- **Pipeline Toggle**: Press X button on driver controller to switch between pipelines

### Auto Behavior
- During shooting phases, automatically switches to neural pipeline to monitor ball count
- Implements smart shooting logic:
  - If ball count drops to ≤2 during shooting: Continue shooting for 1 more second, then stop and proceed to next action
  - If 5 seconds of shooting elapsed: Stop shooting regardless of ball count and proceed to next action
- Automatically switches back to AprilTag pipeline after shooting phase

## Components

### Constants (LimelightConstants)
```java
// Pipeline indices
kAprilTagPipeline = 0
kNeuralDetectorPipeline = 1

// Neural detector settings
kFuelClassID = 0  // Class ID for fuel balls
kMinDetectionConfidence = 0.5  // Minimum confidence threshold

// Auto shooting logic
kMinBallsForContinuedShooting = 2  // Threshold for low ball count
kMaxAutoShootingTime = 5.0  // Maximum shooting time (seconds)
kExtraShootingTime = 1.0  // Extra time when low on balls (seconds)
```

### LimelightDetectionSubSystem
**New Methods:**
- `switchToAprilTagPipeline()` - Switch to pipeline 0
- `switchToNeuralDetectorPipeline()` - Switch to pipeline 1
- `togglePipeline()` - Toggle between pipelines (bound to X button)
- `getDetectedBallCount()` - Returns number of balls detected
- `isAprilTagPipeline()` / `isNeuralDetectorPipeline()` - Check current pipeline

**Behavior Changes:**
- `aimAssist()` now only activates in teleop mode and when using AprilTag pipeline
- Neural detector data is read from `tclass` NetworkTable entry
- Counts balls with class ID 0 and confidence ≥ 0.5

### SmartShootCommand
A custom command that intelligently manages shooting duration:

1. **Initialize**: Starts timer, switches to neural pipeline
2. **Execute**: Continuously monitors ball count, tracks when count drops below threshold
3. **IsFinished**: Returns true when:
   - Maximum time (5s) reached, OR
   - Low ball count detected AND 1 second elapsed since detection
4. **End**: Stops shooter, switches back to AprilTag pipeline

### Autos Class
Updated to support smart shooting:
- Added limelight subsystem parameter
- Added `smartShoot` binding that can be used in Choreo trajectories
- Updated example auto routines to use `SmartShootCommand`

## Usage

### In Teleop
1. Robot starts in AprilTag pipeline mode (default)
2. Press Y button to toggle aim assist on/off (when in AprilTag mode)
3. Press X button to switch to Neural pipeline to see ball count
4. Press X button again to switch back to AprilTag mode

### In Auto
The smart shooting logic is automatically applied in auto routines that use `SmartShootCommand`:

```java
// Example usage in auto routine
ballsToHub.done().onTrue(
    Commands.sequence(
        shooterSubsystem.doShootCmd(),
        new SmartShootCommand(limelightSubsystem, shooterSubsystem)
    )
);
```

The command will:
1. Start the shooter
2. Monitor ball count using neural pipeline
3. Stop shooting based on ball count or time limit
4. Continue to next auto action

## SmartDashboard Outputs
- `CurrentPipeline` - Current pipeline index (0 or 1)
- `PipelineMode` - Current mode name ("AprilTag" or "Neural")
- `DetectedBallCount` - Number of balls currently detected
- `aimAssistActive` - Whether aim assist is currently active
- All existing limelight data (tx, ty, ta, tid, etc.)

## Configuration

### Limelight Configuration Required
1. **Pipeline 0**: Configure for AprilTag detection
   - Enable AprilTag detector
   - Set valid tag IDs (10, 26)

2. **Pipeline 1**: Configure for Neural Detector
   - Enable Neural Detector
   - Train/load model to detect fuel balls
   - Ensure fuel balls are classified as class ID 0
   - Position camera to view hopper

### Tuning Parameters
Adjust in `Constants.java` → `LimelightConstants`:
- `kMinDetectionConfidence` - Lower for more detections, raise for fewer false positives
- `kMinBallsForContinuedShooting` - Threshold for "low" ball count
- `kMaxAutoShootingTime` - Maximum shooting duration
- `kExtraShootingTime` - Additional time to shoot when low on balls

## Benefits
1. **Optimized Auto Time**: Don't waste time shooting when hopper is empty
2. **Flexible Adaptation**: Adjusts shooting duration based on actual ball availability
3. **Safety**: Maximum time limit prevents stuck shooting
4. **Teleop Visibility**: Drivers can check ball count during teleop
5. **Maintained Aim Assist**: AprilTag aim assist functionality preserved and enhanced

## Future Enhancements
- Add PID control for smoother aim assist corrections
- Implement ball trajectory prediction
- Add multi-target tracking
- Integrate botpose for more accurate distance estimation
- Add confidence filtering for more robust detection

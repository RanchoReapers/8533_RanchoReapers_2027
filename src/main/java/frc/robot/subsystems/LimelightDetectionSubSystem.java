package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import java.util.function.BooleanSupplier;

public class LimelightDetectionSubSystem extends SubsystemBase{

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry pipelineEntry = table.getEntry("pipeline");
    
    double tx = table.getEntry("tx").getDouble(0.0);
    double ty = table.getEntry("ty").getDouble(0.0);
    double ta = table.getEntry("ta").getDouble(0.0);

    long tid = table.getEntry("tid").getInteger(0);

    boolean aimAssistActive = false;
    boolean limelightOverride = false;
    
    // Current pipeline (0 = AprilTag, 1 = Neural Detector)
    int currentPipeline = LimelightConstants.kAprilTagPipeline;
    
    // Neural detector data
    int detectedBallCount = 0;
    double[] neuralDetections = new double[0];

    double[] botpose = table.getEntry("botpose").getDoubleArray(new double[0]);
    double[] testTable = {1,2,3,4,5,6};

    double tagsInView = testTable[3];
    double tagAveDistance = testTable[5];

    double xSpeedLimelight = 0.0;
    double ySpeedLimelight = 0.0;

    double turnAngleLimelight = 0.0;
    
    // Distance to target in inches (calculated from ty and known tag height)
    double distanceToTargetInches = 0.0;

    public LimelightDetectionSubSystem() {
    }

    public Command activateLimelightOverride() {
        limelightOverride = !limelightOverride;
        return new InstantCommand();
    }
    
    /**
     * Switch to AprilTag pipeline (pipeline 0)
     */
    public void switchToAprilTagPipeline() {
        currentPipeline = LimelightConstants.kAprilTagPipeline;
        pipelineEntry.setNumber(currentPipeline);
    }
    
    /**
     * Switch to Neural Detector pipeline (pipeline 1)
     */
    public void switchToNeuralDetectorPipeline() {
        currentPipeline = LimelightConstants.kNeuralDetectorPipeline;
        pipelineEntry.setNumber(currentPipeline);
    }
    
    /**
     * Toggle between AprilTag and Neural Detector pipelines
     */
    public Command togglePipeline() {
        return new InstantCommand(() -> {
            if (currentPipeline == LimelightConstants.kAprilTagPipeline) {
                switchToNeuralDetectorPipeline();
            } else {
                switchToAprilTagPipeline();
            }
        });
    }
    
    /**
     * Get current pipeline index
     */
    public int getCurrentPipeline() {
        return currentPipeline;
    }
    
    /**
     * Check if currently using AprilTag pipeline
     */
    public boolean isAprilTagPipeline() {
        return currentPipeline == LimelightConstants.kAprilTagPipeline;
    }
    
    /**
     * Check if currently using Neural Detector pipeline
     */
    public boolean isNeuralDetectorPipeline() {
        return currentPipeline == LimelightConstants.kNeuralDetectorPipeline;
    }
  
    public void updateLimelightData() {
        tx = -table.getEntry("tx").getDouble(0.0);
        ty = -table.getEntry("ty").getDouble(0.0);
        ta = table.getEntry("ta").getDouble(0.0);
        tid = table.getEntry("tid").getInteger(0);
        botpose = table.getEntry("botpose").getDoubleArray(new double[0]);
        tagsInView = testTable[3];
        tagAveDistance = testTable[5];
        
        // Update neural detector data if on neural pipeline
        if (isNeuralDetectorPipeline()) {
            updateNeuralDetectorData();
        }
    }
    
    /**
     * Update neural detector data and count detected balls
     */
    private void updateNeuralDetectorData() {
        // Neural detector outputs detections in tclass array
        // Format: [classID, confidence, tx, ty, ta, ...]
        neuralDetections = table.getEntry("tclass").getDoubleArray(new double[0]);
        
        // Count balls (class ID 0) with sufficient confidence
        detectedBallCount = 0;
        for (int i = 0; i < neuralDetections.length; i += 6) {  // Each detection has 6 values
            if (i + 1 < neuralDetections.length) {
                double classID = neuralDetections[i];
                double confidence = neuralDetections[i + 1];
                
                if (classID == LimelightConstants.kFuelClassID && 
                    confidence >= LimelightConstants.kMinDetectionConfidence) {
                    detectedBallCount++;
                }
            }
        }
    }
    
    /**
     * Get the number of balls detected by the neural detector
     * @return Number of balls detected in the hopper
     */
    public int getDetectedBallCount() {
        return detectedBallCount;
    }

    public void aimAssist() {
        updateLimelightData();

        // Only do aim assist in teleop mode and with AprilTag pipeline
        if (!DriverStation.isTeleop() || !isAprilTagPipeline()) {
            aimAssistActive = false;
            xSpeedLimelight = 0;
            ySpeedLimelight = 0;
            return;
        }

        // Check if we have a valid target ID (10 or 26)
        boolean validTarget = (tid == 10 || tid == 26);

        if(validTarget && tagsInView <= 3 && limelightOverride == false && tagAveDistance < 10) {
            aimAssistActive = true;
            
            // Horizontal alignment (X-axis correction)
            if(tx > LimelightConstants.kHorizontalDeadbandDegrees) { // if we are too right
                xSpeedLimelight = -LimelightConstants.kHorizontalCorrectionSpeed;
            } else if(tx < -LimelightConstants.kHorizontalDeadbandDegrees) { // if we are too left
                xSpeedLimelight = LimelightConstants.kHorizontalCorrectionSpeed;
            } else {
                xSpeedLimelight = 0.0;
            }
            
            // Distance control (Y-axis correction) - maintain target distance
            // Robot moves forward when too far, backward when too close
            // Calculate approximate distance based on botpose or use a simple ty-based estimation
            // Note: This is a simplified approach. In production, use actual distance from botpose
            distanceToTargetInches = estimateDistanceFromTy(ty);
            
            double distanceError = distanceToTargetInches - LimelightConstants.kTargetDistanceInches;
            
            if (distanceError < -LimelightConstants.kDepthDeadbandInches) {
                // Too far - move forward
                ySpeedLimelight = LimelightConstants.kDepthCorrectionSpeed;
            } else if (distanceError > LimelightConstants.kDepthDeadbandInches) {
                // Too close - move backward
                ySpeedLimelight = -LimelightConstants.kDepthCorrectionSpeed;
            } else {
                ySpeedLimelight = 0.0;
            }
        } else {
            aimAssistActive = false;
            xSpeedLimelight = 0;
            ySpeedLimelight = 0;
        }
    }
    
    /**
     * Estimates distance to target in inches based on vertical angle (ty).
     * This is a simplified estimation. For better accuracy, use botpose data.
     * Assumes camera mounted at a fixed height and angle.
     * 
     * @param ty The vertical angle to the target in degrees (negative when looking up at target)
     * @return Estimated distance to target in inches
     */
    private double estimateDistanceFromTy(double ty) {
        // Handle edge case where ty is zero or very small
        if (Math.abs(ty) < 0.1) {
            return LimelightConstants.kMaxDistanceInches; // Default far distance if no/minimal angle
        }
        
        // Simple estimation using inverse relationship with ty
        // Formula: distance (inches) = kTargetDistanceInches * (kDistanceCalibrationTyReference / ty)
        // This assumes ty is negative when looking up at target (typical Limelight configuration)
        // Calibration: When ty = kDistanceCalibrationTyReference, distance = kTargetDistanceInches
        double estimatedDistance = LimelightConstants.kTargetDistanceInches * 
                                   (LimelightConstants.kDistanceCalibrationTyReference / ty);
        
        // Clamp to reasonable range
        return Math.max(LimelightConstants.kMinDistanceInches, 
                       Math.min(LimelightConstants.kMaxDistanceInches, estimatedDistance));
    }

    public double getXSpeedLimelight() {
        return xSpeedLimelight;
    }

    public double getYSpeedLimelight() {
        return ySpeedLimelight;
    }

    public double getTurnAngleLimelight() {
        return turnAngleLimelight;
    }

    public BooleanSupplier getAimAssistActive() {
        return () -> aimAssistActive;
    }

    @Override
    public void periodic() {
        aimAssist();
        periodicOdometry();
    }

    public void periodicOdometry() {
        SmartDashboard.putNumber("LimelightX", tx);
        SmartDashboard.putNumber("LimelightY", ty);
        SmartDashboard.putNumber("LimelightArea", ta);
        SmartDashboard.putNumber("LimelightID", tid);
        SmartDashboard.putBoolean("aimAssistActive", aimAssistActive);
        SmartDashboard.putNumber("DistanceToTarget", distanceToTargetInches);
        SmartDashboard.putNumber("TargetDistance", LimelightConstants.kTargetDistanceInches);
        SmartDashboard.putNumber("CurrentPipeline", currentPipeline);
        SmartDashboard.putString("PipelineMode", isAprilTagPipeline() ? "AprilTag" : "Neural");
        SmartDashboard.putNumber("DetectedBallCount", detectedBallCount);
    }

}


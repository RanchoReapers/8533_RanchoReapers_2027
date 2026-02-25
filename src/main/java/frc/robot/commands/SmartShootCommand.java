package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.LimelightDetectionSubSystem;
import frc.robot.subsystems.ShooterSubSystem;

/**
 * Command that intelligently manages shooting duration during autonomous based on ball count.
 * 
 * Behavior:
 * - Switches to neural detector pipeline to monitor ball count
 * - If ball count drops to <= 2, shoots for 1 more second then ends
 * - If 5 seconds elapsed, ends regardless of ball count
 * - Switches back to AprilTag pipeline when done
 */
public class SmartShootCommand extends Command {
    private final LimelightDetectionSubSystem limelightSubsystem;
    private final ShooterSubSystem shooterSubsystem;
    private final Timer timer;
    
    private boolean lowBallCountDetected = false;
    private double lowBallCountTime = 0.0;
    private boolean initializedPipeline = false;

    public SmartShootCommand(LimelightDetectionSubSystem limelightSubsystem, ShooterSubSystem shooterSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.timer = new Timer();
        
        // Don't require subsystems since shooting is already active
        // This command just monitors and decides when to stop
    }

    @Override
    public void initialize() {
        timer.restart();
        lowBallCountDetected = false;
        lowBallCountTime = 0.0;
        initializedPipeline = false;
    }

    @Override
    public void execute() {
        // Switch to neural pipeline on first execute
        if (!initializedPipeline) {
            limelightSubsystem.switchToNeuralDetectorPipeline();
            initializedPipeline = true;
        }
        
        // Check ball count
        int ballCount = limelightSubsystem.getDetectedBallCount();
        
        // If we haven't detected low ball count yet, check for it
        if (!lowBallCountDetected && ballCount <= LimelightConstants.kMinBallsForContinuedShooting) {
            lowBallCountDetected = true;
            lowBallCountTime = timer.get();
        }
    }

    @Override
    public boolean isFinished() {
        double elapsedTime = timer.get();
        
        // Case 1: Maximum shooting time reached (5 seconds)
        if (elapsedTime >= LimelightConstants.kMaxAutoShootingTime) {
            return true;
        }
        
        // Case 2: Low ball count detected, and we've shot for 1 more second since detection
        if (lowBallCountDetected && 
            (elapsedTime - lowBallCountTime) >= LimelightConstants.kExtraShootingTime) {
            return true;
        }
        
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        // Switch back to AprilTag pipeline
        limelightSubsystem.switchToAprilTagPipeline();
        // Stop shooting
        shooterSubsystem.shooterTriggerReleased();
    }
}

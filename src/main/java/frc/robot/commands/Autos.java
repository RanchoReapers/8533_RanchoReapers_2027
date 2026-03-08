package frc.robot.commands;

import static frc.robot.generated.ChoreoTraj.*;

import java.util.Set;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.IntakeRetractorSubSystem;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.LimelightDetectionSubSystem;
import frc.robot.subsystems.SwerveSubSystem;
import frc.robot.subsystems.ShooterSubSystem;

public final class Autos {

    private final SwerveSubSystem swerveSubSystem;
    private final IntakeSubSystem intakeSubsystem;
    private final IntakeRetractorSubSystem intakeRetractorSubsystem;
    private final LimelightDetectionSubSystem limelightDetectionSubsystem;
    private final ShooterSubSystem shooterSubsystem;

    private final AutoFactory autoFactory;
    private final AutoChooser autonomousProgramChooser;

    public Autos(SwerveSubSystem swerveSubSystem, IntakeSubSystem intakeSubsystem, IntakeRetractorSubSystem intakeRetractorSubsystem, LimelightDetectionSubSystem limelightDetectionSubsystem, ShooterSubSystem shooterSubsystem) {
        this.swerveSubSystem = swerveSubSystem;
        this.intakeSubsystem = intakeSubsystem;
        this.intakeRetractorSubsystem = intakeRetractorSubsystem;
        this.limelightDetectionSubsystem = limelightDetectionSubsystem;
        this.shooterSubsystem = shooterSubsystem;

        this.autoFactory = swerveSubSystem.createAutoFactory();
        this.autonomousProgramChooser = new AutoChooser();

        autoFactory
                .bind("activateIntake", intakeSubsystem.doIntakeCmd())
                .bind("deactivateIntake", intakeSubsystem.stopIntakeCmd());

    }

    public void configureAutoChooser() {
        autonomousProgramChooser.addRoutine("LEFT TRENCH -> COLLECT SHOOT READYTOCOLLECT", this::startingFrom_LEFT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_Auto);
        autonomousProgramChooser.addRoutine("LEFT BUMP -> SHOOT COLLECT SHOOT COLLECT", this::startingFrom_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_Auto);
        autonomousProgramChooser.addRoutine("LEFT BUMP -> COLLECT SHOOT COLLECT", this::startingFrom_LEFT_BUMP_Performing_COLLECT_SHOOT_COLLECT_Auto);
        autonomousProgramChooser.addRoutine("HUB via LEFT BUMP -> SHOOT COLLECT SHOOT COLLECT", this::startingFrom_HUB_Via_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_Auto);

        autonomousProgramChooser.addRoutine("RIGHT TRENCH -> COLLECT SHOOT READYTOCOLLECT", this::startingFrom_RIGHT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_Auto);
        autonomousProgramChooser.addRoutine("RIGHT BUMP -> SHOOT COLLECT SHOOT COLLECT", this::startingFrom_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_Auto);
        autonomousProgramChooser.addRoutine("RIGHT BUMP -> COLLECT SHOOT COLLECT", this::startingFrom_RIGHT_BUMP_Performing_COLLECT_SHOOT_COLLECT_Auto);
        autonomousProgramChooser.addRoutine("HUB via RIGHT BUMP -> SHOOT COLLECT SHOOT COLLECT", this::startingFrom_HUB_Via_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_Auto);

        SmartDashboard.putData("Auto Chooser", autonomousProgramChooser);

        RobotModeTriggers.autonomous().whileTrue(autonomousProgramChooser.selectedCommandScheduler());
    }

    public void startShootingForPeriodOfTime(AutoTrajectory firstRoutine, AutoTrajectory routineToStartAfterShootingFinishes, int lowBallThreshold, double debounceTimeSeconds, double timeoutSeconds, boolean bypassLimelight) {
        firstRoutine.done().onTrue(
                Commands.defer(() -> {

                    Timer shootTimer = new Timer();
                    Timer debounceTimer = new Timer();

                    shootTimer.start();
                    debounceTimer.start();

                    return Commands.sequence(
                            // Start shooter once
                            shooterSubsystem.doShootCmd(),
                            // Wait until low balls (debounced) OR 5 sec timeout
                            Commands.waitUntil(() -> {

                                boolean debouncedLowBalls;

                                if (!bypassLimelight) {
                                    int count = limelightDetectionSubsystem.getTargetCountLimelight();

                                    if (count > 2) {
                                        debounceTimer.restart();
                                    }

                                    debouncedLowBalls = count <= 2 && debounceTimer.hasElapsed(0.2);

                                } else {
                                    debouncedLowBalls = false;
                                }

                                boolean timeout = shootTimer.hasElapsed(5.0);
                                return debouncedLowBalls || timeout;
                            }),
                            // If ended because of low balls → wait 1 extra second
                            Commands.either(
                                    Commands.waitSeconds(1.0),
                                    Commands.none(),
                                    () -> checkLimelightBypassStatusForCommandSwitcher(bypassLimelight)
                            ),
                            // Stop shooter
                            shooterSubsystem.stopShootCmd(),
                            // Starts next commands
                            routineToStartAfterShootingFinishes.cmd()
                    );

                    // requirements
                }, Set.of(shooterSubsystem))
        );
    }

    private boolean checkLimelightBypassStatusForCommandSwitcher(boolean bypassLimelight) {
        if (!bypassLimelight) {
            return limelightDetectionSubsystem.getTargetCountLimelight() <= 2;
        } else {
            return false;
        }
    }

    // COMPETITION AUTOS
    private AutoRoutine startingFrom_LEFT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_Auto() {
        // declare autoRoutine name
        final AutoRoutine startingFrom_LEFT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine = autoFactory.newRoutine("starting at LEFT TRENCH performing COLLECT SHOOT READYTOCOLLECT");

        // declare autoTrajectories used in this autoRoutine & sequence of commands
        final AutoTrajectory trenchToBalls = BLUELeftTrenchToLeftSideOfBalls.asAutoTraj(startingFrom_LEFT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine);
        final AutoTrajectory ballsToHub = BLUELeftBallsCollectionToHubViaTrench.asAutoTraj(startingFrom_LEFT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine);
        final AutoTrajectory hubToBalls = BLUELeftHubToLeftBallsCollectionViaTrench.asAutoTraj(startingFrom_LEFT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine);

        // When the routine begins, reset odometry and start the first trajectory 
        startingFrom_LEFT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine.active().onTrue(
                Commands.sequence(
                        intakeRetractorSubsystem.doIntakeRetractionCmd(),
                        trenchToBalls.resetOdometry(),
                        trenchToBalls.cmd()
                )
        );

        // start subsequent trajectories when the previous is done
        trenchToBalls.done().onTrue(ballsToHub.cmd());
        startShootingForPeriodOfTime(ballsToHub, hubToBalls, 2, 0.2, 5.0, true);

        return startingFrom_LEFT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine;
    }

    private AutoRoutine startingFrom_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_Auto() {
        final AutoRoutine startingFrom_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine = autoFactory.newRoutine("starting at LEFT BUMP performing SHOOT COLLECT SHOOT COLLECT");

        final AutoTrajectory bumpToHub = BLUELeftBumpToHub.asAutoTraj(startingFrom_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory hubToBump = BLUELeftHubToBump.asAutoTraj(startingFrom_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory bumpToBalls = BLUELeftBumpToLeftSideOfBalls.asAutoTraj(startingFrom_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory ballsToHub = BLUELeftBallsCollectionToHubViaBump.asAutoTraj(startingFrom_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory hubToBalls = BLUELeftHubToLeftBallsCollectionViaBump.asAutoTraj(startingFrom_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);

        startingFrom_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine.active().onTrue(
                Commands.sequence(
                        intakeRetractorSubsystem.doIntakeRetractionCmd(),
                        bumpToHub.resetOdometry(),
                        bumpToHub.cmd()
                )
        );

        startShootingForPeriodOfTime(bumpToHub, hubToBump, 2, 0.2, 2.5, true);
        hubToBump.done().onTrue(bumpToBalls.cmd());
        bumpToBalls.done().onTrue(ballsToHub.cmd());
        startShootingForPeriodOfTime(ballsToHub, hubToBalls, 2, 0.2, 5, true);

        return startingFrom_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine;
    }

    private AutoRoutine startingFrom_LEFT_BUMP_Performing_COLLECT_SHOOT_COLLECT_Auto() {
        final AutoRoutine startingFrom_LEFT_BUMP_Performing_COLLECT_SHOOT_COLLECT_AutoRoutine = autoFactory.newRoutine("starting at LEFT BUMP performing COLLECT SHOOT COLLECT");

        final AutoTrajectory bumpToBalls = BLUELeftBumpToLeftSideOfBalls.asAutoTraj(startingFrom_LEFT_BUMP_Performing_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory ballsToHub = BLUELeftBallsCollectionToHubViaBump.asAutoTraj(startingFrom_LEFT_BUMP_Performing_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory hubToBalls = BLUELeftHubToLeftBallsCollectionViaBump.asAutoTraj(startingFrom_LEFT_BUMP_Performing_COLLECT_SHOOT_COLLECT_AutoRoutine);

        startingFrom_LEFT_BUMP_Performing_COLLECT_SHOOT_COLLECT_AutoRoutine.active().onTrue(
                Commands.sequence(
                        //intakeRetractorSubsystem.doIntakeRetractionCmd(),
                        bumpToBalls.resetOdometry(),
                        bumpToBalls.cmd()
                )
        );

        bumpToBalls.done().onTrue(ballsToHub.cmd());
        startShootingForPeriodOfTime(ballsToHub, hubToBalls, 2, 0.2, 5, true);

        return startingFrom_LEFT_BUMP_Performing_COLLECT_SHOOT_COLLECT_AutoRoutine;
    }

    private AutoRoutine startingFrom_HUB_Via_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_Auto() {
        final AutoRoutine startingFrom_HUB_Via_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine = autoFactory.newRoutine("starting at HUB via LEFT BUMP performing SHOOT COLLECT SHOOT COLLECT");

        final AutoTrajectory hubToBackOfHub = BLUEHubToBackOfHub.asAutoTraj(startingFrom_HUB_Via_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory backOfHubToRightBump = BLUEBackofHubToLeftBump.asAutoTraj(startingFrom_HUB_Via_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory bumpToBalls = BLUELeftBumpToLeftSideOfBalls.asAutoTraj(startingFrom_HUB_Via_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory ballsToHub = BLUELeftBallsCollectionToHubViaBump.asAutoTraj(startingFrom_HUB_Via_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory hubToBalls = BLUELeftHubToLeftBallsCollectionViaBump.asAutoTraj(startingFrom_HUB_Via_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);
        startingFrom_HUB_Via_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine.active().onTrue(
                Commands.sequence(
                        intakeRetractorSubsystem.doIntakeRetractionCmd(),
                        hubToBackOfHub.resetOdometry(),
                        hubToBackOfHub.cmd()
                )
        );

        startShootingForPeriodOfTime(hubToBackOfHub, backOfHubToRightBump, 2, 0.2, 2.5, true);
        backOfHubToRightBump.done().onTrue(bumpToBalls.cmd());
        bumpToBalls.done().onTrue(ballsToHub.cmd());
        startShootingForPeriodOfTime(ballsToHub, hubToBalls, 2, 0.2, 5, true);

        return startingFrom_HUB_Via_LEFT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine;
    }

    private AutoRoutine startingFrom_RIGHT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_Auto() {
        final AutoRoutine startingFrom_RIGHT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine = autoFactory.newRoutine("starting at RIGHT TRENCH performing COLLECT SHOOT READYTOCOLLECT");

        final AutoTrajectory trenchToBalls = BLUERightTrenchToRightSideOfBalls.asAutoTraj(startingFrom_RIGHT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine);
        final AutoTrajectory ballsToHub = BLUERightBallsCollectionToHubViaTrench.asAutoTraj(startingFrom_RIGHT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine);
        final AutoTrajectory hubToBalls = BLUERightHubToRightBallsCollectionViaTrench.asAutoTraj(startingFrom_RIGHT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine);

        startingFrom_RIGHT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine.active().onTrue(
                Commands.sequence(
                        intakeRetractorSubsystem.doIntakeRetractionCmd(),
                        trenchToBalls.resetOdometry(),
                        trenchToBalls.cmd()
                )
        );

        trenchToBalls.done().onTrue(ballsToHub.cmd());
        startShootingForPeriodOfTime(ballsToHub, hubToBalls, 2, 0.2, 5.5, true);

        return startingFrom_RIGHT_TRENCH_Performing_COLLECT_SHOOT_READYTOCOLLECT_AutoRoutine;
    }

    private AutoRoutine startingFrom_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_Auto() {
        final AutoRoutine startingFrom_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine = autoFactory.newRoutine("starting at RIGHT BUMP performing SHOOT COLLECT SHOOT COLLECT");

        final AutoTrajectory bumpToHub = BLUERightBumpToHub.asAutoTraj(startingFrom_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory hubToBump = BLUERightHubToBump.asAutoTraj(startingFrom_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory bumpToBalls = BLUERightBumpToRightSideOfBalls.asAutoTraj(startingFrom_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory ballsToHub = BLUERightBallsCollectionToHubViaBump.asAutoTraj(startingFrom_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory hubToBalls = BLUERightHubToRightBallsCollectionViaBump.asAutoTraj(startingFrom_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);

        startingFrom_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine.active().onTrue(
                Commands.sequence(
                        intakeRetractorSubsystem.doIntakeRetractionCmd(),
                        bumpToHub.resetOdometry(),
                        bumpToHub.cmd()
                )
        );

        startShootingForPeriodOfTime(bumpToHub, hubToBump, 2, 0.2, 2.5, true);
        hubToBump.done().onTrue(bumpToBalls.cmd());
        bumpToBalls.done().onTrue(ballsToHub.cmd());
        startShootingForPeriodOfTime(ballsToHub, hubToBalls, 2, 0.2, 5, true);

        return startingFrom_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine;
    }

    private AutoRoutine startingFrom_RIGHT_BUMP_Performing_COLLECT_SHOOT_COLLECT_Auto() {
        final AutoRoutine startingFrom_RIGHT_BUMP_Performing_COLLECT_SHOOT_COLLECT_AutoRoutine = autoFactory.newRoutine("starting at RIGHT BUMP performing COLLECT SHOOT COLLECT");

        final AutoTrajectory bumpToBalls = BLUERightBumpToRightSideOfBalls.asAutoTraj(startingFrom_RIGHT_BUMP_Performing_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory ballsToHub = BLUERightBallsCollectionToHubViaBump.asAutoTraj(startingFrom_RIGHT_BUMP_Performing_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory hubToBalls = BLUERightHubToRightBallsCollectionViaBump.asAutoTraj(startingFrom_RIGHT_BUMP_Performing_COLLECT_SHOOT_COLLECT_AutoRoutine);

        startingFrom_RIGHT_BUMP_Performing_COLLECT_SHOOT_COLLECT_AutoRoutine.active().onTrue(
                Commands.sequence(
                        intakeRetractorSubsystem.doIntakeRetractionCmd(),
                        bumpToBalls.resetOdometry(),
                        bumpToBalls.cmd()
                )
        );

        bumpToBalls.done().onTrue(ballsToHub.cmd());
        startShootingForPeriodOfTime(ballsToHub, hubToBalls, 2, 0.2, 5, true);

        return startingFrom_RIGHT_BUMP_Performing_COLLECT_SHOOT_COLLECT_AutoRoutine;
    }

    private AutoRoutine startingFrom_HUB_Via_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_Auto() {
        final AutoRoutine startingFrom_HUB_Via_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine = autoFactory.newRoutine("starting at HUB via RIGHT BUMP performing SHOOT COLLECT SHOOT COLLECT");

        final AutoTrajectory hubToBackOfHub = BLUEHubToBackOfHub.asAutoTraj(startingFrom_HUB_Via_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory backOfHubToRightBump = BLUEBackOfHubToRightBump.asAutoTraj(startingFrom_HUB_Via_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory bumpToBalls = BLUERightBumpToRightSideOfBalls.asAutoTraj(startingFrom_HUB_Via_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory ballsToHub = BLUERightBallsCollectionToHubViaBump.asAutoTraj(startingFrom_HUB_Via_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);
        final AutoTrajectory hubToBalls = BLUERightHubToRightBallsCollectionViaBump.asAutoTraj(startingFrom_HUB_Via_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine);
        startingFrom_HUB_Via_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine.active().onTrue(
                Commands.sequence(
                        intakeRetractorSubsystem.doIntakeRetractionCmd(),
                        hubToBackOfHub.resetOdometry(),
                        hubToBackOfHub.cmd()
                )
        );

        startShootingForPeriodOfTime(hubToBackOfHub, backOfHubToRightBump, 2, 0.2, 2.5, true);
        backOfHubToRightBump.done().onTrue(bumpToBalls.cmd());
        bumpToBalls.done().onTrue(ballsToHub.cmd());
        startShootingForPeriodOfTime(ballsToHub, hubToBalls, 2, 0.2, 5, true);

        return startingFrom_HUB_Via_RIGHT_BUMP_Performing_SHOOT_COLLECT_SHOOT_COLLECT_AutoRoutine;
    }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Dimensions;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.LinesVertical;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerOperatingMode;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeSystemMode;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterOperatingMode;
import frc.robot.util.HubShiftUtil;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// FILE GUIDE: High-level robot coordinator: translates whole-robot intentions into separate commands for intake, indexer, and shooter.
public class Superstructure extends SubsystemBase {

    // STUDENT MAP: COORDINATED ROBOT ACTIONS
    // PURPOSE: Combines intake, shooter, and indexer commands for whole-robot scoring, passing, and idling.
    // LOOK FOR: Select a requested robot mode first; scheduled commands decide what each subsystem actually does.
    private final Shooter shooter;
    private final Intake intake;
    private final Indexer indexer;

    private final Supplier<Pose2d> robotPoseSupplier;

    @Getter
    @AutoLogOutput
    private RobotOperatingMode requestedRobotMode = RobotOperatingMode.SCORING;

    @AutoLogOutput
    public final Trigger inAllianceZoneTrigger = new Trigger(this::isRobotInAllianceZone);

    @AutoLogOutput
    private boolean isHubShiftOverrideEnabled = false;

    /**
     * True when the timing utility currently reports an active hub.
     * This condition does not include a pre-shoot countdown.
     */
    @AutoLogOutput
    public final Trigger isHubActive =
        new Trigger(() -> HubShiftUtil.getShiftedShiftInfo(isHubShiftOverrideEnabled).active());

    @AutoLogOutput
    public final Trigger isHubActiveInAllianceZone =
        inAllianceZoneTrigger.and(DriverStation::isTeleop).and(isHubActive);

    @AutoLogOutput
    public final Trigger isHubInactiveInAllianceZone =
        inAllianceZoneTrigger.and(DriverStation::isTeleop).and(isHubActive.negate());

    @AutoLogOutput
    public final Trigger hasLeftAllianceZone = inAllianceZoneTrigger.negate()
        .and(DriverStation::isTeleop);

    private final Map<RobotOperatingMode, Supplier<Command>> commandsForRobotMode;

    /**
     * Creates a new Superstructure.
     */
    // STUDENT NOTE: Build the recipe of individual subsystem commands for scoring and passing.
    public Superstructure(Shooter shooter, Intake intake, Indexer indexer,
        Supplier<Pose2d> robotPoseSupplier) {
        this.shooter = shooter;
        this.intake = intake;
        this.indexer = indexer;
        this.robotPoseSupplier = robotPoseSupplier;

        // Each entry is a recipe for a WHOLE-ROBOT mode.
        // Commands.parallel runs its parts together; Commands.sequence runs them in order.
        // A Supplier<Command> constructs the recipe when needed, not at robot startup.
        // Map each whole-robot goal to commands for its component subsystems.
        // Wait durations and command composition are preserved exactly.
        commandsForRobotMode = Map.of(
            RobotOperatingMode.SCORING,
            () -> Commands.parallel(
                    runOnce(() -> this.shooter.setShooterEnabled(true)),
                    Commands.waitSeconds(0.65).andThen(this.indexer.setOperatingModeCommand(IndexerOperatingMode.FEEDING)),
                    Commands.runOnce(() -> System.out.println("Shoot intake"), intake).andThen(Commands.waitSeconds(1.25)).andThen(this.intake.setSystemModeCommand(IntakeSystemMode.STOWING))
                )
                .withName("Start scoring"),
            RobotOperatingMode.PASSING,
            () -> Commands.sequence(
                    runOnce(() -> this.shooter.setShooterEnabled(true)),
                    Commands.waitSeconds(0.2).andThen(this.indexer.setOperatingModeCommand(IndexerOperatingMode.FEEDING))

//                    Commands.waitSeconds(1.25).andThen(this.intake.setGoalCommand(IntakeGoal.STOW))
                )
                .withName("Start passing"),
            RobotOperatingMode.IDLING,
            () -> Commands.sequence(
                    this.shooter.setOperatingModeCommand(ShooterOperatingMode.IDLING)

//                    this.intake.setGoal(IntakeGoal.STOW)
//                    this.indexer.setGoal(IndexerGoal.IDLE)
                )
                .withName("Idle"));

        inAllianceZoneTrigger.onTrue(runOnce(() -> {
                requestedRobotMode = RobotOperatingMode.SCORING;
                this.shooter.setOperatingMode(ShooterOperatingMode.SCORING);
            }));
        hasLeftAllianceZone.onTrue(
            runOnce(() -> {
                requestedRobotMode = RobotOperatingMode.PASSING;
                this.shooter.setOperatingMode(ShooterOperatingMode.PASSING);
            }));

    }

    // STUDENT NOTE: Test the robot position against the alliance-specific field boundary.
    private boolean isRobotInAllianceZone() {
        Pose2d pose = robotPoseSupplier.get();
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        return isBlue && pose.getMeasureX()
            .lt(Units.Meters.of(LinesVertical.allianceZone).plus(Dimensions.FULL_WIDTH.div(2)))
            || !isBlue
            && pose.getMeasureX()
            .gt(Units.Meters.of(FieldConstants.fieldLength).minus(
                Units.Meters.of(LinesVertical.allianceZone).plus(Dimensions.FULL_WIDTH.div(2))));
    }

    // STUDENT NOTE: Create a command that records a chosen whole-robot mode.
    public Command selectRobotMode(RobotOperatingMode requestedMode) {
        return this.runOnce(() -> this.requestedRobotMode = requestedMode)

//            .andThen(goalCommands.get(newGoal).get())
            .andThen(runOnce(() -> this.shooter.setOperatingMode(switch (requestedMode) {
                case SCORING -> ShooterOperatingMode.SCORING;
                case PASSING -> ShooterOperatingMode.PASSING;
                case IDLING -> ShooterOperatingMode.IDLING;
            })))
            .withName("Set goal");
    }

    // Commands.defer chooses the operating mode when the command is scheduled, not
    // when this Superstructure object is first constructed.
    // STUDENT NOTE: Choose the command recipe when scheduled, then run it until ended.
    public Command runSelectedRobotMode() {
        return Commands.defer(() -> commandsForRobotMode.get(requestedRobotMode).get(), Set.of(intake, indexer, shooter)).andThen(Commands.none().repeatedly()).finallyDo(
            this::idleSubsystems);
    }

    // Stop shooter/indexer requests. The existing code also requests COLLECTING,
    // which keeps the intake deployed rather than retracting it.
    // That may sound surprising: the original code deliberately does this.
    // STUDENT NOTE: Stop shooter and indexer requests; current code separately requests intake collecting.
    public void idleSubsystems() {
        shooter.setShooterEnabled(false);
        intake.setSystemMode(IntakeSystemMode.COLLECTING);

        indexer.setRequestedIndexerMode(IndexerOperatingMode.IDLING);
    }

    // STUDENT NOTE: Build a command that temporarily overrides the usual hub shift logic.
    public Command enableShiftOverride() {
        return Commands.startEnd(() -> isHubShiftOverrideEnabled = true, () -> isHubShiftOverrideEnabled = false)
            .withName("Override active first");
    }

    // READ THIS: Update/log high-level robot status and state triggers.
    @Override
    public void periodic() {
        Logger.recordOutput(
            "Superstructure/Shift Time", HubShiftUtil.getOfficialShiftInfo().remainingTime());
    }

    /**
     * The requested operating mode of the coordinated scoring mechanisms, which determines the behavior of the turret,
     * intake, and indexer. The requestedRobotMode is automatically set based on the robot's position on the field
     * and the hub's active/inactive state, but can also be manually selected by the driver using
     * the selectRobotMode command as well as the various related commands.
     */
    public static enum RobotOperatingMode {
        /**
         * Score fuel in the hub. Automatically activated when entering the alliance zone if the hub
         * is active.
         */
        SCORING,
        /**
         * Pass fuel to our alliance zone. Automatically activated when leaving the alliance zone,
         * unless currently collecting.
         */
        PASSING,
        /**
         * Request an idle operating mode. This enum alone does not stow the intake;
         * inspect the command recipe and idleSubsystems() for the actions actually requested.
         */
        IDLING
    }
}

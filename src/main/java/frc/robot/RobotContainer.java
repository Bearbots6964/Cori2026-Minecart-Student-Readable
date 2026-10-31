// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;


import static frc.robot.subsystems.vision.VisionConstants.*;
import static frc.robot.subsystems.vision.VisionConstants.robotToRightCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerOperatingMode;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.Intake.IntakeSystemMode;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.shooter.Shooter.ShooterOperatingMode;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.*;
import frc.robot.util.ControllerRumbleManager;
import frc.robot.util.FuelSim;
import frc.robot.util.Identifiable;
import java.util.function.BooleanSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * command-based pattern, most button decisions belong here rather than in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // STUDENT MAP: ROBOT ASSEMBLY AND CONTROLLER BINDINGS
    // PURPOSE: Builds the Java objects for the physical robot, chooses real hardware or simulation, and connects gamepad buttons to commands.
    // LOOK FOR: Start at the constructor, then configureButtonBindings. A Command is created here but runs only when scheduled.


    // One Java object per physical robot subsystem. These objects do not
    // represent motors directly; each subsystem uses its own IO class
    // to communicate with real, simulated, or replayed hardware.
    private final Drive swerveDrive;
    private final Vision visionSystem;
    private final Intake intakeSystem;

    //    private final Climber climber;
    private final Shooter shooterSystem;
    private final Indexer indexerSystem;

    private final Superstructure robotCoordinator;

    private SwerveDriveSimulation swerveDriveSimulation = null;

    // Controller
    private final CommandXboxController driverGamepad = new CommandXboxController(0);
    private final CommandXboxController operatorGamepad = new CommandXboxController(1);

    // Buttons are converted to Triggers here. A Trigger is a condition
    // that the scheduler checks; configureButtonBindings() gives each
    // condition its command.
    // Driver
    private final Trigger crossBumpButtonPressed = driverGamepad.a();
    private final Trigger lockWheelsButtonPressed = driverGamepad.x();
    private final Trigger alignToTowerButtonPressed = driverGamepad.start();
    private final Trigger resetHeadingButtonPressed = driverGamepad.back();
    private final Trigger cancelAutonomousButtonPressed = driverGamepad.b();

    // Operator
    private final Trigger spinShooterButtonPressed = operatorGamepad.leftTrigger();
    private final Trigger shootButtonPressed = operatorGamepad.rightTrigger();
    private final Trigger stopIntakeButtonPressed = operatorGamepad.y();
    private final Trigger collectFuelButtonPressed = operatorGamepad.b();
    private final Trigger retractIntakeButtonPressed = operatorGamepad.b()
        .and(operatorGamepad.rightBumper());
    private final Trigger ejectFuelButtonPressed = operatorGamepad.leftBumper();
    private final Trigger manualShooterControlButtonPressed = operatorGamepad.a();

    // The dashboard chooser holds autonomous routines and test commands.
    private final LoggedDashboardChooser<Command> autoChooser;

    private final ControllerRumbleManager rumbleManager;

    public FuelSim fuelSim;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    // STUDENT NOTE: Choose the real, simulated, or replay setup; create the subsystem objects.
    public RobotContainer() {

        // Exactly one branch runs: REAL connects hardware, SIM uses simulated
        // or placeholder devices, and REPLAY reads recorded inputs.
        switch (Constants.currentMode) {
            case REAL:

                // Real robot, instantiate hardware IO implementations
                // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
                // a CANcoder
                swerveDrive =
                    new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight),
                        (pose) -> {
                        });
                intakeSystem = new Intake(new IntakeIOTalonFX());

//                climber = new Climber(new ClimberIOTalonFX());
                shooterSystem = new Shooter(new ShooterIOTalonFX(), swerveDrive::getPose,
                    swerveDrive::getChassisSpeeds);
                indexerSystem = new Indexer(new IndexerIOTalonFX(), shooterSystem::getRequestedIndexerSpeedRps);

                this.visionSystem = new Vision(
                    swerveDrive,
                    new VisionIOPhotonVision(leftCameraName, robotToLeftCamera),
                    new VisionIOPhotonVision(rightCameraName, robotToRightCamera),
                    new VisionIOPhotonVision(rearCameraName, robotToRearCamera)
                );
                break;
            case SIM:

                // Sim robot, instantiate physics sim IO implementations
                configureFuelSim();

                swerveDriveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig,
                    new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
                swerveDrive =
                    new Drive(
                        new GyroIO() {
                        },
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight),
                        swerveDriveSimulation::setSimulationWorldPose);
                visionSystem = new Vision(
                    swerveDrive,
                    new VisionIOPhotonVisionSim(
                        leftCameraName, robotToLeftCamera, swerveDriveSimulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonVisionSim(
                        rightCameraName, robotToRightCamera, swerveDriveSimulation::getSimulatedDriveTrainPose));
                intakeSystem = new Intake(new IntakeIO() {
                });

//                climber = new Climber(new ClimberIO() {});
                shooterSystem = new Shooter(new ShooterIO() {
                }, swerveDrive::getPose, swerveDrive::getChassisSpeeds);
                indexerSystem = new Indexer(new IndexerIO() {
                }, shooterSystem::getRequestedIndexerSpeedRps);

                break;

            default:

                // Replayed robot, disable IO implementations
                swerveDrive = new Drive(
                    new GyroIO() {
                    },
                    new ModuleIO() {
                    },
                    new ModuleIO() {
                    },
                    new ModuleIO() {
                    },
                    new ModuleIO() {
                    },
                    (pose) -> {
                    });
                visionSystem = new Vision(swerveDrive, new VisionIO() {
                }, new VisionIO() {
                });
                intakeSystem = new Intake(new IntakeIO() {
                });

//                climber = new Climber(new ClimberIO() {});
                shooterSystem = new Shooter(new ShooterIO() {
                }, swerveDrive::getPose, swerveDrive::getChassisSpeeds);

                indexerSystem = new Indexer(new IndexerIO() {
                }, shooterSystem::getRequestedIndexerSpeedRps);
                break;
        }

        // The superstructure coordinates the shooter, intake, and indexer.
        robotCoordinator = new Superstructure(shooterSystem, intakeSystem, indexerSystem, swerveDrive::getPose);

        // STUDENT NOTE: Register autonomous behavior after the subsystems are ready.
        configureAutonomous();

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization",
            DriveCommands.wheelRadiusCharacterization(swerveDrive));
        autoChooser.addOption("Drive Simple FF Characterization",
            DriveCommands.feedforwardCharacterization(swerveDrive));
        addSysIdOptions("Drive", swerveDrive);
        addSysIdOptions("Shooter", shooterSystem);
        addSysIdOptions("Intake", intakeSystem);

        // Configure the button bindings
        configureButtonBindings();

        rumbleManager = new ControllerRumbleManager(
            (c) -> driverGamepad.setRumble(RumbleType.kBothRumble, c));
    }

    // STUDENT NOTE: Add motor-characterization commands to the autonomous chooser.
    private void addSysIdOptions(String name, Identifiable subsystem) {
        autoChooser.addOption(name + " SysId (Quasistatic Forward)", subsystem.sysIdQuasistatic(
            Direction.kForward));
        autoChooser.addOption(name + " SysId (Quasistatic Reverse)", subsystem.sysIdQuasistatic(
            Direction.kReverse));
        autoChooser.addOption(name + " SysId (Dynamic Forward)", subsystem.sysIdDynamic(
            Direction.kForward));
        autoChooser.addOption(name + " SysId (Dynamic Reverse)", subsystem.sysIdDynamic(
            Direction.kReverse));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    // STUDENT NOTE: Associate each controller Trigger with the Command it should schedule.
    private void configureButtonBindings() {

        // A default command runs whenever no other scheduled command owns
        // the drive subsystem. The joystick values are negated to match
        // the team's driving directions.
        // Default command, normal field-relative drive
        swerveDrive.setDefaultCommand(
            DriveCommands.joystickDrive(swerveDrive, () -> -driverGamepad.getLeftY(),
                () -> -driverGamepad.getLeftX(), () -> -driverGamepad.getRightX()));

//        slowMovementTrigger.whileTrue(DriveCommands.joystickDriveSlowly(drive, () -> -driverController.getLeftY(),
//            () -> -driverController.getLeftX(), () -> -driverController.getRightX()));
//        fineTurningTrigger.whileTrue(DriveCommands.joystickDriveFollowingVelocity(drive, () -> -driverController.getLeftY(), () -> -driverController.getLeftX()));
//        drive.nearBumpTrigger.whileTrue(
//            DriveCommands.joystickDriveOverBump(drive, () -> -driverController.getLeftY(),
//                () -> -driverController.getLeftX()));

        // Switch to X pattern when X button is pressed
        lockWheelsButtonPressed.onTrue(Commands.runOnce(swerveDrive::stopWithX, swerveDrive));

// Historical driver-controller experiment is disabled in this project.
        // The operator B button requests the collecting/deployed intake state.
        // Note: the existing trigger order and button combinations are
        // deliberately unchanged in this readability-only refactor.
        // When the operator presses B, request that the arm deploy and rollers collect.
        collectFuelButtonPressed.onTrue(intakeSystem.setSystemModeCommand(IntakeSystemMode.COLLECTING));

        // Holding B plus Right Bumper requests the arm to retract.
        // NOTE: These combinations overlap; verify which command wins on hardware.
        retractIntakeButtonPressed.and(intakeSystem.isArmRetracted.negate())
            .whileTrue(intakeSystem.setSystemModeCommand(IntakeSystemMode.STOWING));

        // Holding Y requests rollers to stop; the arm retains its previous target.
        stopIntakeButtonPressed.whileTrue(intakeSystem.setSystemModeCommand(IntakeSystemMode.IDLING));

        // Holding Left Bumper reverses both the intake rollers and indexer.
        ejectFuelButtonPressed.whileTrue(
            intakeSystem.setSystemModeCommand(IntakeSystemMode.EJECTING).alongWith(indexerSystem.setOperatingModeCommand(
                IndexerOperatingMode.REVERSING))).onFalse(indexerSystem.setOperatingModeCommand(IndexerOperatingMode.IDLING)
            .alongWith(intakeSystem.setSystemModeCommand(IntakeSystemMode.COLLECTING)));

        shootButtonPressed.whileTrue(DriveCommands.joystickDriveAtAngle(
            swerveDrive, () -> -driverGamepad.getLeftY(), () -> -driverGamepad.getLeftX(),
            shooterSystem::getCurrentFieldTarget, () -> -operatorGamepad.getRightX(), () -> operatorGamepad.x().getAsBoolean()).alongWith(
            robotCoordinator.runSelectedRobotMode()).finallyDo(robotCoordinator::idleSubsystems));
        spinShooterButtonPressed.whileTrue(
            DriveCommands.joystickDrive(swerveDrive, () -> -driverGamepad.getLeftY() / 4,
                    () -> -driverGamepad.getLeftX() / 4, () -> -operatorGamepad.getRightX())
                .alongWith(robotCoordinator.runSelectedRobotMode()).finallyDo(robotCoordinator::idleSubsystems));

//        shootTrigger.onTrue(indexer.setGoal(IndexerGoal.ACTIVE));
        manualShooterControlButtonPressed.onTrue(shooterSystem.setOperatingModeCommand(ShooterOperatingMode.TUNING).repeatedly());

//        operatorController.povUp().onTrue(intake.setGoal(IntakeGoal.TILT));
//        operatorController.povUp().whileTrue(turret.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
//        operatorController.povRight().whileTrue(turret.sysIdQuasistatic(Direction.kReverse));
//        operatorController.povDown().whileTrue(turret.sysIdDynamic(SysIdRoutine.Direction.kForward));
//        operatorController.povLeft().whileTrue(turret.sysIdDynamic(Direction.kReverse));
        operatorGamepad.start()
            .whileTrue(indexerSystem.feedFuelUntilInterrupted().alongWith(shooterSystem.runSlowlyCommand(() -> 2.5)));

        // Reset gyro / odometry
        final Runnable resetGyro =
            Constants.currentMode == Constants.Mode.SIM ? () -> swerveDrive.setPose(
                swerveDriveSimulation.getSimulatedDriveTrainPose())

                // reset odometry to actual robot pose during
                // simulation
                : () -> swerveDrive.setPose(

                    // zero gyro
                    new Pose2d(swerveDrive.getPose().getTranslation(), new Rotation2d()));
        resetHeadingButtonPressed.onTrue(Commands.runOnce(resetGyro, swerveDrive).ignoringDisable(true));

        operatorGamepad.povDown().onTrue(intakeSystem.nudgeArmUp());
        operatorGamepad.povUp().onTrue(intakeSystem.nudgeArmDown());
    }

    /**
     * Register command names referenced by PathPlanner autonomous files.
     * DO NOT rename the quoted strings without updating those .auto files.
     */
    // STUDENT NOTE: Register names used by PathPlanner autonomous routines.
    private void configureAutonomous() {
        NamedCommands.registerCommand("Shoot", robotCoordinator.runSelectedRobotMode().alongWith(
            DriveCommands.joystickDriveAtAngle(swerveDrive, () -> 0.0, () -> 0.0,
                shooterSystem::getCurrentFieldTarget, () -> 0.0, () -> false)).finallyDo(() -> intakeSystem.setSystemMode(IntakeSystemMode.COLLECTING)));
        NamedCommands.registerCommand("Shoot Only", robotCoordinator.runSelectedRobotMode()
            .alongWith(Commands.runOnce(() -> System.out.println("Superstructure running")))
            .finallyDo(() -> System.out.println("Superstructure stopped")));
        NamedCommands.registerCommand("Intake", intakeSystem.setSystemModeCommand(IntakeSystemMode.COLLECTING));
    }

    // STUDENT NOTE: Create virtual fuel and scoring behavior for simulation.
    private void configureFuelSim() {
        fuelSim = new FuelSim();
        fuelSim.spawnStartingFuel();

        fuelSim.start();
        SmartDashboard.putData(Commands.runOnce(() -> {
                fuelSim.clearFuel();
                fuelSim.spawnStartingFuel();
            })
            .withName("Reset Fuel")
            .ignoringDisable(true));
    }

    // READ THIS: Tell the simulator when the robot can collect a simulated game piece.
    private void configureFuelSimRobot(BooleanSupplier ableToIntake, Runnable intakeCallback) {

        // TODO: implement method to link fuel sim with simulated intake
    }

    /** Re-select the shooter target using the alliance reported by Driver Station. */
    public void refreshShooterAllianceHubTarget() {
        shooterSystem.refreshAllianceHubTarget();
    }

    /**
     * Starts the controller rumble manager to provide rumble feedback during the match. Should be
     * called at beginning of teleop.
     */
    public void startControllerRumbleManager() {
        if (rumbleManager.isInitialized()) {

            // already started
            return;
        }
        CommandScheduler.getInstance().schedule(rumbleManager.fullMatchSequence());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    // STUDENT NOTE: Give Robot.java whichever autonomous command the driver selected.
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    // READ THIS: Reset game pieces/field state for another simulation run.
    public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM) {
            return;
        }

        swerveDriveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    // STUDENT NOTE: Advance simulated driving and game-piece physics.
    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) {
            return;
        }

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition",
            swerveDriveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput("FieldSimulation/Coral",
            SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput("FieldSimulation/Algae",
            SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }
}

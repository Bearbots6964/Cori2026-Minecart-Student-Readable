package frc.robot.subsystems.shooter;


import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.ShooterConstants.TargetingConstants.FLYWHEEL_RADIUS;
import static frc.robot.subsystems.shooter.ShooterConstants.TargetingConstants.PASSING_SPOT_LEFT;
import static frc.robot.subsystems.shooter.ShooterConstants.TargetingConstants.PASSING_SPOT_RIGHT;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Hub;
import frc.robot.subsystems.shooter.ShooterCalculator.ShotData;
import frc.robot.subsystems.shooter.ShooterConstants.TalonFXConstants.FlywheelMotorConstants;
import frc.robot.subsystems.shooter.ShooterConstants.TalonFXConstants.HoodMotorConstants;
import frc.robot.subsystems.shooter.ShooterConstants.TargetingConstants;
import frc.robot.util.Identifiable;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// FILE GUIDE: Shooter subsystem: chooses shooting mode, computes hood/flywheel targets, and records what hardware reports.
public class Shooter extends SubsystemBase implements Identifiable {

    // STUDENT MAP: SHOOTER SYSTEM DECISION-MAKING
    // PURPOSE: Chooses scoring, passing, tuning, or idle behavior and calculates desired flywheel and hood settings.
    // LOOK FOR: Read setOperatingMode and periodic before the angle-conversion math at the bottom.
    private final ShooterIO shooterHardware;
    private ShooterIOInputsAutoLogged shooterSensorReadings = new ShooterIOInputsAutoLogged();
    private final Supplier<Pose2d> currentRobotPoseSupplier;
    private final Supplier<ChassisSpeeds> fieldRelativeChassisSpeedsSupplier;

    public Rotation2d requestedHoodAngle = new Rotation2d();

    @Getter
    private double requestedIndexerSpeedRps = 0.0;

    @Getter
    @AutoLogOutput
    Translation3d currentFieldTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ? Hub.topCenterPoint
        : Hub.opposingTopCenterPoint;

    @AutoLogOutput
    private ShooterOperatingMode operatingMode = ShooterOperatingMode.SCORING;


    @Setter
    @AutoLogOutput
    private boolean shooterEnabled = false;


    private final Alert flywheelMotorDisconnected;
    private final Alert flywheelFollowerMotor1Disconnected;
    private final Alert flywheelFollowerMotor2Disconnected;
    private final Alert flywheelFollowerMotor3Disconnected;

    private final LoggedTunableNumber hoodKP = new LoggedTunableNumber("Turret/Hood/kP",
        HoodMotorConstants.HOOD_GAINS.kP);
    private final LoggedTunableNumber hoodKD = new LoggedTunableNumber("Turret/Hood/kD",
        HoodMotorConstants.HOOD_GAINS.kD);
    private final LoggedTunableNumber hoodKS = new LoggedTunableNumber("Turret/Hood/kS",
        HoodMotorConstants.HOOD_GAINS.kS);
    private final LoggedTunableNumber hoodKV = new LoggedTunableNumber("Turret/Hood/kV",
        HoodMotorConstants.HOOD_GAINS.kV);

    private final LoggedTunableNumber flywheelKP = new LoggedTunableNumber("Turret/Flywheel/kP",
        FlywheelMotorConstants.FLYWHEEL_GAINS.kP);
    private final LoggedTunableNumber flywheelKD = new LoggedTunableNumber("Turret/Flywheel/kD",
        FlywheelMotorConstants.FLYWHEEL_GAINS.kD);
    private final LoggedTunableNumber flywheelKS = new LoggedTunableNumber("Turret/Flywheel/kS",
        FlywheelMotorConstants.FLYWHEEL_GAINS.kS);
    private final LoggedTunableNumber flywheelKV = new LoggedTunableNumber("Turret/Flywheel/kV",
        FlywheelMotorConstants.FLYWHEEL_GAINS.kV);

    private final LoggedTunableNumber tuningFlywheelSpeed = new LoggedTunableNumber(
        "Turret/Tuning/FlywheelSpeedRPS", 0.0);
    private final LoggedTunableNumber tuningHoodPosition = new LoggedTunableNumber(
        "Turret/Tuning/HoodPosition", 0.0);

    private final SysIdRoutine flywheelRoutine;
    private final SysIdRoutine hoodRoutine;

    /** Refresh the hub target from the alliance reported by Driver Station. */
    public void refreshAllianceHubTarget() {
        currentFieldTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? Hub.topCenterPoint
            : Hub.opposingTopCenterPoint;
    }


    // STUDENT NOTE: Connect shooter hardware with robot position and movement information.
    public Shooter(ShooterIO shooterHardware, Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> fieldRelativeChassisSpeedsSupplier) {
        this.shooterHardware = shooterHardware;
        this.currentRobotPoseSupplier = poseSupplier;
        this.fieldRelativeChassisSpeedsSupplier = fieldRelativeChassisSpeedsSupplier;

        flywheelMotorDisconnected = new Alert("Turret flywheel motor disconnected.",
            AlertType.kError);
        flywheelFollowerMotor1Disconnected = new Alert(
            "Turret flywheel follower motor 1 disconnected.", AlertType.kError);

        flywheelFollowerMotor2Disconnected = new Alert(
            "Turret flywheel follower motor 2 disconnected.", AlertType.kError);
        flywheelFollowerMotor3Disconnected = new Alert(
            "Turret flywheel follower motor 3 disconnected.", AlertType.kError);

        SmartDashboard.putData(
            Commands.runOnce(() -> setOperatingModeCommand(ShooterOperatingMode.TUNING)).withName("Turret Tuning Mode").ignoringDisable(true));

        flywheelRoutine = new SysIdRoutine(
            new Config(null, null, null,
                (state) -> SignalLogger.writeString("state", state.toString())),
            new Mechanism(shooterHardware::setFlywheelVoltage, null, this)
        );

        hoodRoutine = new SysIdRoutine(
            new Config(Volts.per(Second).of(0.1), Volts.of(1), null,
                (state) -> SignalLogger.writeString("state", state.toString())),
            new Mechanism(shooterHardware::setHoodVoltage, null, this)
        );
    }

    /**
     * Read sensor data, calculate a shot for scoring/passing, apply tuning
     * requests, and report motor connection status once per scheduler cycle.
     * This method intentionally preserves the original operation order.
     */
    @Override

    // STUDENT NOTE: Read the shooter sensors, compute scoring targets, apply tuning requests, and show alerts.
    public void periodic() {

        // STUDENT NOTE: First refresh measured hardware state; calculated targets are not sensor measurements.
        shooterHardware.updateInputs(shooterSensorReadings);
        Logger.processInputs("Turret", shooterSensorReadings);

        Pose2d pose = currentRobotPoseSupplier.get();

        // STUDENT NOTE: Only scoring/passing modes use pose-based shot calculations.
        if (operatingMode == ShooterOperatingMode.SCORING || operatingMode == ShooterOperatingMode.PASSING) {
            calculateShot(pose);
        }

        if (operatingMode == ShooterOperatingMode.PASSING) {
            setTarget(getPassingTarget(pose));
        }

        if (operatingMode == ShooterOperatingMode.TUNING) {
            shooterHardware.setFlywheelSpeed(RotationsPerSecond.of(tuningFlywheelSpeed.get()));
            shooterHardware.setHoodPosition(Degrees.of(tuningHoodPosition.get()));
        }

        Logger.recordOutput("Turret/Distance To Target",
            ShooterCalculator.getDistanceToTarget(pose, currentFieldTarget));

        updateTunables();

        flywheelMotorDisconnected.set(!shooterSensorReadings.flywheelLeaderConnected);
        flywheelFollowerMotor1Disconnected.set(!shooterSensorReadings.flywheelFollower1Connected);
        flywheelFollowerMotor2Disconnected.set(!shooterSensorReadings.flywheelFollower2Connected);
        flywheelFollowerMotor3Disconnected.set(!shooterSensorReadings.flywheelFollower3Connected);
    }

    // READ THIS: Wrap a requested operating-mode change in a one-shot WPILib command.
    public Command setOperatingModeCommand(ShooterOperatingMode operatingMode) {
        return runOnce(() -> setOperatingMode(operatingMode));
    }

    // STUDENT NOTE: Select scoring, passing, tuning, or off/idle behavior.
    public void setOperatingMode(ShooterOperatingMode operatingMode) {
        this.operatingMode = operatingMode;
        System.out.println("Shooter goal set to " + operatingMode);
        switch (operatingMode) {
            case SCORING, TUNING:
                setTarget(Hub.topCenterPoint);
                break;
            case PASSING:
                setTarget(getPassingTarget(currentRobotPoseSupplier.get()));
                break;
            case IDLING, OFF:
                stop();
                break;
        }

    }


    // STUDENT NOTE: Set the field point that the shooter should aim at.
    public void setTarget(Translation3d target) {
        this.currentFieldTarget = target;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            Translation2d flipped = FlippingUtil.flipFieldPosition(target.toTranslation2d());
            currentFieldTarget = new Translation3d(flipped.getX(), flipped.getY(), target.getZ());
        }
    }

    // STUDENT NOTE: Choose the target point used when passing fuel instead of scoring.
    private Translation3d getPassingTarget(Pose2d pose) {
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        boolean onBlueLeftSide = currentRobotPoseSupplier.get().getMeasureY()
            .gt(Units.Meters.of(FieldConstants.fieldWidth / 2.0));

        return isBlue == onBlueLeftSide ? PASSING_SPOT_LEFT : PASSING_SPOT_RIGHT;
    }

    /**
     * Use the robot pose, movement, and current target to request a hood
     * angle and flywheel speed. The indexer speed comes from the target
     * distance map, not from a separate encoder on the indexer.
     */
    // STUDENT NOTE: Determine which flywheel speed and hood setting are needed for the target.
    private void calculateShot(Pose2d robotPose) {
        ChassisSpeeds fieldSpeeds = fieldRelativeChassisSpeedsSupplier.get();

        ShotData calculatedShot = ShooterCalculator.iterativeMovingShotFromMap(robotPose,
            fieldSpeeds, currentFieldTarget,
            TargetingConstants.LOOKAHEAD_ITERATIONS);
        Angle azimuthAngle = ShooterCalculator.calculateAzimuthAngle(robotPose,
            calculatedShot.target());
        double dist = robotPose.getTranslation().getDistance(currentFieldTarget.toTranslation2d());
        requestedIndexerSpeedRps = TargetingConstants.INDEXER_MAP.get(dist);
        requestedHoodAngle = new Rotation2d(azimuthAngle);
        if (shooterEnabled) {
            shooterHardware.setHoodPosition(calculatedShot.getHoodAngle());
            shooterHardware.setFlywheelSpeed(
                ShooterCalculator.linearToAngularVelocity(calculatedShot.getExitVelocity(),
                    FLYWHEEL_RADIUS));
        }
        else {
            shooterHardware.setHoodPosition(Degrees.of(0.0));
            shooterHardware.stopFlywheel();
        }

//        io.setFlywheelSpeed(calculatedShot.getExitAngularVelocity());
        Logger.recordOutput("Turret/CalculatedShot", calculatedShot);
    }

    // READ THIS: Create a command that applies a supplied flywheel voltage until interrupted, then stops.
    public Command runSlowlyCommand(DoubleSupplier speed) {
        return runEnd(() -> shooterHardware.setFlywheelVoltage(Volts.of(speed.getAsDouble())), shooterHardware::stopFlywheel);
    }

    // STUDENT NOTE: Apply calibration changes from dashboard-adjustable numbers.
    private void updateTunables() {
        if (hoodKP.hasChanged(hashCode())
            || hoodKD.hasChanged(hashCode())
            || hoodKV.hasChanged(hashCode())
            || hoodKS.hasChanged(hashCode())) {
            shooterHardware.setHoodPIDConstants(hoodKP.get(), hoodKD.get(), hoodKV.get(), hoodKS.get());
        }

        if (flywheelKP.hasChanged(hashCode())
            || flywheelKD.hasChanged(hashCode())
            || flywheelKV.hasChanged(hashCode())
            || flywheelKS.hasChanged(hashCode())) {
            shooterHardware.setFlywheelPIDConstants(flywheelKP.get(), flywheelKD.get(), flywheelKV.get(),
                flywheelKS.get());
        }

    }


    // READ THIS: Send the stop request through the hardware interface.
    private void stop() {
        operatingMode = ShooterOperatingMode.IDLING;
        shooterHardware.stopFlywheel();
        shooterHardware.setHoodPosition(Degrees.of(0.0));
    }

    // STUDENT NOTE: Convert a physical hood angle into the motor-position representation.
    public static double hoodAngleToPosition(Angle hoodAngle) {
        return hoodAngle.in(

            // TODO: Implement this conversion based on the specific hardware setup
            Radians);
    }

    // READ THIS: Use the linkage geometry to convert a servo stroke length into a hood angle.
    public static Angle actualHoodPositionToAngle(double hoodPosition) {

        // precondition: hoodPosition is in millimeters and is between 0 and 100 mm. it represents the current stroke length of the servo
        // Point AB is the point of pivot of the hood, side A is the distance between the pivot and the point where the servo connects to the shooter assembly, and side B is the distance between the pivot and the point where the servo connects to the hood.
        // Point AC is the point where the servo connects to the shooter assembly, and point BC is the point where the servo connects to the hood.
        // The angle between A and the horizontal plane intersecting AB is 16.137492 degrees from horizontal. This is static.
        Distance sideA = Inches.of(9.888568);
        Distance sideB = Inches.of(6.25);
        Distance sideC =

            // add the offset from the servo stroke to the actual length of the side
            Millimeters.of(hoodPosition).plus(Millimeters.of(168.0));

        Angle angle = Radians.of(Math.acos(
            (Math.pow(sideA.in(Inches), 2) + Math.pow(sideB.in(Inches), 2) - Math.pow(sideC.in(Inches), 2))
                / (2 * sideA.in(Inches) * sideB.in(Inches))
        ));

        // subtract the static angle to get the actual hood angle from horizontal
        return angle.minus(Degrees.of(16.1374915)).minus(Degrees.of(4.666254));
    }

    // READ THIS: Use the linkage geometry to convert a hood angle into servo stroke length.
    public static double actualHoodAngleToPosition(Angle hoodAngle) {

        // precondition: hoodAngle is the angle between the hood and the horizontal plane, in degrees. 0 degrees means the hood is parallel to the ground, and positive angles mean the hood is angled upwards.
        // Point AB is the point of pivot of the hood, side A is the distance between the pivot and the point where the servo connects to the shooter assembly, and side B is the distance between the pivot and the point where the servo connects to the hood.
        // Point AC is the point where the servo connects to the shooter assembly, and point BC is the point where the servo connects to the hood.
        // The angle between A and the horizontal plane intersecting AB is 16.137492 degrees from horizontal. This is static.
        Distance sideA = Inches.of(9.888568);
        Distance sideB = Inches.of(6.25);

        // add a static angle to account for the fact that 0 degrees on our hood does not correspond to 0 mm of servo stroke
        Angle staticAngle = Degrees.of(16.1374915).plus(Degrees.of(4.666254));

        Angle angleC = hoodAngle.plus(staticAngle);

        double sideCLength = Math.sqrt(
            Math.pow(sideA.in(Inches), 2) + Math.pow(sideB.in(Inches), 2)
                - 2 * sideA.in(Inches) * sideB.in(Inches) * Math.cos(angleC.in(Radians))
        );

        // subtract the offset from the actual length of the side to get the servo stroke length
        double servoStroke = sideCLength - Millimeters.of(168.0).in(Inches);

        return Inches.of(servoStroke).in(Millimeters);
    }


    // STUDENT NOTE: Convert the motor-position representation into a hood angle; see existing TODO.
    public static Angle hoodPositionToAngle(double hoodPosition) {
        return Radians.of(

            // TODO: Implement this conversion based on the specific hardware setup
            hoodPosition);
    }

    // READ THIS: Return the gradual flywheel characterization command.
    public Command sysIdQuasistatic(SysIdRoutine.Direction dir) {
        return flywheelRoutine.quasistatic(dir);
    }

    // READ THIS: Return the sudden-step flywheel characterization command.
    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return flywheelRoutine.dynamic(dir);
    }


    public enum ShooterOperatingMode {
        SCORING,
        PASSING,
        IDLING,
        TUNING,
        OFF
    }

    public enum HoodGoal {
        ACTIVE,
        IDLE
    }
}

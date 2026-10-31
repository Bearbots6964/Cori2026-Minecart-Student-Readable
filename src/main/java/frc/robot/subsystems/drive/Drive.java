// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;


import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.ChassisAccelerations;
import frc.robot.util.Identifiable;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.Zones;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// FILE GUIDE: Swerve drivetrain: turns chassis movement requests into four module targets and estimates field position from sensor readings.
public class Drive extends SubsystemBase implements Vision.VisionConsumer, Identifiable {

    // STUDENT MAP: FOUR-MODULE SWERVE DRIVETRAIN
    // PURPOSE: Reads gyro/wheel states, estimates robot position, and sends motion targets to four modules.
    // LOOK FOR: Joystick requests arrive from DriveCommands; sensor data returns through ModuleIO and GyroIO.


    // TunerConstants doesn't include these constants, so they are declared locally
    static final double ODOMETRY_FREQUENCY = TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
    public static final double DRIVE_BASE_RADIUS = Math.max(
        Math.max(
            Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
        Math.max(
            Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

    // PathPlanner config constants
    private static final double ROBOT_MASS_KG = 61.68856232;
    private static final double ROBOT_MOI = 6.2639058241;
    private static final double WHEEL_COF = 1.2;
    private static final RobotConfig PP_CONFIG = new RobotConfig(
        ROBOT_MASS_KG,
        ROBOT_MOI,
        new ModuleConfig(
            TunerConstants.FrontLeft.WheelRadius,
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
            WHEEL_COF,
            DCMotor.getKrakenX60Foc(1).withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
            TunerConstants.FrontLeft.SlipCurrent,
            1),
        getModuleTranslations());

    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
        .withBumperSize(Units.Inches.of(35.0), Units.Inches.of(35.0))
        .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
        .withCustomModuleTranslations(getModuleTranslations())
        .withGyro(COTS.ofPigeon2())
        .withSwerveModule(new SwerveModuleSimulationConfig(
            DCMotor.getKrakenX60(1),
            DCMotor.getKrakenX60(1),
            TunerConstants.FrontLeft.DriveMotorGearRatio,
            TunerConstants.FrontLeft.SteerMotorGearRatio,
            Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
            Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
            Meters.of(TunerConstants.FrontLeft.WheelRadius),
            KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
            WHEEL_COF));

    static final Lock odometryLock = new ReentrantLock();

    // Gyro reports measured heading; it does not command the chassis to turn.
    private final GyroIO gyroHardware;
    private final GyroIOInputsAutoLogged gyroSensorReadings = new GyroIOInputsAutoLogged();

    // FL, FR, BL, BR
    private final Module[] swerveModules = new Module[4];
    private final SysIdRoutine driveIdentificationRoutine;
    private final Alert gyroDisconnectedAlert =
        new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    private SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawRobotHeading = Rotation2d.kZero;

    // For delta tracking
    private SwerveModulePosition[] previousModulePositions =
        new SwerveModulePosition[]{
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };

    // Pose is the estimated location and heading of the robot on the field.
    // Estimation uses gyro/module motion and can be corrected using vision.
    private SwerveDrivePoseEstimator robotPoseEstimator =
        new SwerveDrivePoseEstimator(swerveKinematics, rawRobotHeading, previousModulePositions,
            Pose2d.kZero);

    private final Consumer<Pose2d> resetSimulationPoseCallback;

    private ChassisSpeeds previousChassisSpeeds = new ChassisSpeeds();

    @AutoLogOutput
    public final Trigger isRobotNearBump = Zones.BUMP_ZONES.willContain(this::getPose,
        this::getChassisSpeeds, Seconds.of(0.3)).or(Zones.BUMP_ZONES.contains(this::getPose));

    // STUDENT NOTE: Create four module objects, set up gyro readings, and start position estimation.
    public Drive(
        GyroIO gyroHardware,
        ModuleIO flModuleIO,
        ModuleIO frModuleIO,
        ModuleIO blModuleIO,
        ModuleIO brModuleIO,
        Consumer<Pose2d> resetSimulationPoseCallback) {
        this.gyroHardware = gyroHardware;
        this.resetSimulationPoseCallback = resetSimulationPoseCallback;
        swerveModules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
        swerveModules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
        swerveModules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
        swerveModules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive,
            tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
            this::getPose,
            this::setPose,
            this::getChassisSpeeds,
            this::runVelocity,
            new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(5.0, 0.0, 0.0)),
            PP_CONFIG,
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
            (activePath) -> Logger.recordOutput("Odometry/Trajectory",
                activePath.toArray(new Pose2d[0])));
        PathPlannerLogging.setLogTargetPoseCallback(
            (targetPose) -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

        // Configure SysId
        driveIdentificationRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null,
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null,
                this));
    }

    /**
     * Each cycle: read all four wheel swerveModules and the gyro, stop motors if
     * disabled, then update the estimated robot position from sensor samples.
     */
    @Override

    // STUDENT NOTE: Read wheel/gyro sensors, update robot position, and check connections.
    public void periodic() {

        // Prevents odometry updates while reading data
        odometryLock.lock();

        // STUDENT NOTE: Collect actual gyro measurements before updating the pose estimate.
        gyroHardware.updateInputs(gyroSensorReadings);
        Logger.processInputs("Drive/Gyro", gyroSensorReadings);
        for (var module : swerveModules) {
            module.periodic();
        }
        odometryLock.unlock();
        previousChassisSpeeds = getChassisSpeeds();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : swerveModules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[]{});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[]{});
        }

        // ODOMETRY: estimate how far the robot moved using wheel distances,
        // wheel directions, and gyroscope rotation. Timestamps keep readings
        // from different wheels aligned.
        // Update odometry
        // All signals are sampled together
        double[] sampleTimestamps = swerveModules[0].getOdometryTimestamps();
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {

            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = swerveModules[moduleIndex].getSampledModulePositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                    modulePositions[moduleIndex].distanceMeters
                        - previousModulePositions[moduleIndex].distanceMeters,
                    modulePositions[moduleIndex].angle);
                previousModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroSensorReadings.connected) {

                // Use the real gyro angle
                rawRobotHeading = gyroSensorReadings.odometryYawPositions[i];
            } else {

                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = swerveKinematics.toTwist2d(moduleDeltas);
                rawRobotHeading = rawRobotHeading.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            robotPoseEstimator.updateWithTime(sampleTimestamps[i], rawRobotHeading, modulePositions);
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroSensorReadings.connected && Constants.currentMode != Mode.SIM);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    // ChassisSpeeds describes forward, sideways, and turning speed for the
  // WHOLE robot. Kinematics translates it to four wheel speeds and angles.
  // STUDENT NOTE: Convert the requested chassis movement into desired speed and angle for four modules.
  public void runVelocity(ChassisSpeeds speeds) {

        // Convert one requested chassis velocity (forward, sideways, turn)
        // into four wheel speed/angle requests; do not change this math.
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = swerveKinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            swerveModules[i].applyRequestedWheelState(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /**
     * Runs the drive in a straight line with the specified drive output.
     */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].runCharacterization(output);
        }
    }

    /**
     * Stops the drive.
     */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the swerveModules to an X arrangement to resist movement. The swerveModules
     * will return to their normal orientations the next time a nonzero velocity is requested.
     */
    // STUDENT NOTE: Point wheels into an X pattern to resist movement.
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        swerveKinematics.resetHeadings(headings);
        stop();
    }

    /**
     * Returns a command to run a quasistatic test in the specified direction.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0)
            .andThen(driveIdentificationRoutine.quasistatic(direction));
    }

    /**
     * Returns a command to run a dynamic test in the specified direction.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0)
            .andThen(driveIdentificationRoutine.dynamic(direction));
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the swerveModules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = swerveModules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the swerveModules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = swerveModules[i].getPosition();
        }
        return states;
    }

    /**
     * Returns the measured chassis speeds of the robot.
     */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    // READ THIS: Estimate acceleration from the current and previous chassis speeds.
    public ChassisAccelerations getChassisAccelerations() {
        return new ChassisAccelerations(getChassisSpeeds(), previousChassisSpeeds, 0.020);
    }

    /**
     * Returns the position of each module in radians.
     */
    public double[] getWheelRotationsForRadiusCalibration() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = swerveModules[i].getWheelRotationForRadiusCalibration();
        }
        return values;
    }

    /**
     * Returns the average velocity of the swerveModules in rotations/sec (Phoenix native units).
     */
    public double getFeedforwardTestSpeed() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += swerveModules[i].getFeedforwardTestSpeed() / 4.0;
        }
        return output;
    }

    /**
     * Returns the current odometry pose.
     */
    @AutoLogOutput(key = "Odometry/Robot")

    // STUDENT NOTE: Return the current estimated field position and orientation.
    public Pose2d getPose() {
        return robotPoseEstimator.getEstimatedPosition();
    }

    /**
     * Returns the current odometry rotation.
     */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    // READ THIS: Return the measured or estimated robot turning rate.
    public AngularVelocity getAngularVelocity() {
        return RadiansPerSecond.of(getChassisSpeeds().omegaRadiansPerSecond);
    }

    /**
     * Resets the current odometry pose.
     */
    public void setPose(Pose2d pose) {
        resetSimulationPoseCallback.accept(pose);
        robotPoseEstimator.resetPosition(rawRobotHeading, getModulePositions(), pose);
    }

    /**
     * Adds a new timestamped vision measurement.
     */
    // STUDENT NOTE: Use an accepted camera pose estimate to improve the robot position estimate.
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs) {
        robotPoseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds,
            visionMeasurementStdDevs);
    }

    /**
     * Returns the maximum linear speed in meters per sec.
     */
    public double getMaxLinearSpeedMetersPerSec() {
        return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    /**
     * Returns the maximum angular speed in radians per sec.
     */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    /**
     * Returns an array of module translations.
     */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[]{
            new Translation2d(TunerConstants.FrontLeft.LocationX,
                TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX,
                TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX,
                TunerConstants.BackRight.LocationY)
        };
    }
}

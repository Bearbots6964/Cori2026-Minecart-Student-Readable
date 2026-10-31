// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.drive.Drive;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Arrays;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

// FILE GUIDE: Reusable drive commands: convert joystick or automated movement requests into chassis motion.
public class DriveCommands {

    // STUDENT MAP: DRIVER-CONTROL MOVEMENT COMMANDS
    // PURPOSE: Turns joystick and requested heading information into motion commands for Drive.
    // LOOK FOR: A method returning Command describes an action for WPILib to schedule; it does not drive immediately.
    private static final double JOYSTICK_DEADBAND = 0.1;
    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;

    // Secs
    private static final double FF_START_DELAY = 2.0;

    // Volts/Sec
    private static final double FF_RAMP_RATE = 0.1;

    // Rad/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25;

    // Rad/Sec^2
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05;

    // READ THIS: Prevent creating a DriveCommands object; this class only contains static command factories.
    private DriveCommands() {
    }

    // STUDENT NOTE: Convert gamepad stick X/Y into a travel-direction vector.
    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {

        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), JOYSTICK_DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(Translation2d.kZero, linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
            .getTranslation();
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular
     * velocities).
     */
    // STUDENT NOTE: Build a command that controls chassis translation and rotation from gamepad input.
    public static Command joystickDrive(
        Drive drive,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        DoubleSupplier omegaSupplier) {
        return Commands.run(
            () -> {

                // Get linear velocity
                Translation2d linearVelocity =
                    getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
                        ySupplier.getAsDouble());

                // Apply rotation deadband
                double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), JOYSTICK_DEADBAND);

                // Square rotation value for more precise control
                omega = Math.copySign(omega * omega, omega);

                // Convert to field relative speeds & send command
                ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                        omega * drive.getMaxAngularSpeedRadPerSec());
                boolean isFlipped =
                    DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red;
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        isFlipped
                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                            : drive.getRotation()));
            },
            drive);
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities)
     * with limited speed.
     */
    // STUDENT NOTE: Build a reduced-speed driving command.
    public static Command joystickDriveSlowly(
        Drive drive,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        DoubleSupplier omegaSupplier) {
        return Commands.run(
            () -> {

                // Get linear velocity
                Translation2d linearVelocity =
                    getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
                        ySupplier.getAsDouble());

                // Apply rotation deadband
                double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), JOYSTICK_DEADBAND);

                // Square rotation value for more precise control
                omega = Math.copySign(omega * omega, omega);

                // Convert to field relative speeds & send command
                ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        linearVelocity.getX() * AutoConstants.SLOW_SPEED,
                        linearVelocity.getY() * AutoConstants.SLOW_SPEED,
                        omega * drive.getMaxAngularSpeedRadPerSec());
                boolean isFlipped =
                    DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red;
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        isFlipped
                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                            : drive.getRotation()));
            },
            drive);
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */
    // STUDENT NOTE: Build a command that drives and attempts to face a chosen angle.
    public static Command joystickDriveAtAngle(
        Drive drive,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        Supplier<Translation3d> currentFieldTarget,
        DoubleSupplier omegaSupplier,
        BooleanSupplier xTheWheels) {

        // Create PID controller
        ProfiledPIDController angleController =
            new ProfiledPIDController(
                ANGLE_KP,
                0.0,
                ANGLE_KD,
                new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.run(
                () -> {
                    Logger.recordOutput("Odometry/AngleTarget", currentFieldTarget.get());

                    // Get linear velocity
                    Translation2d linearVelocity =
                        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
                            ySupplier.getAsDouble());

                    // Apply rotation deadband
                    double controllerOmega =
                        MathUtil.applyDeadband(omegaSupplier.getAsDouble(), JOYSTICK_DEADBAND) / 4.0;

                    // Square rotation value for more precise control
                    controllerOmega = Math.copySign(controllerOmega * controllerOmega, controllerOmega);

                    // Calculate angular speed
                    double omega =
                        angleController.calculate(
                            drive.getRotation().getRadians(), new Rotation2d(
                                currentFieldTarget.get().getX() - drive.getPose().getX(),
                                currentFieldTarget.get().getY() - drive.getPose().getY()).plus(
                                Rotation2d.k180deg).getRadians());

                    omega += controllerOmega;
                    if (Math.abs(xSupplier.getAsDouble()) < JOYSTICK_DEADBAND
                        && Math.abs(ySupplier.getAsDouble()) < JOYSTICK_DEADBAND && xTheWheels.getAsBoolean()) {
                        drive.stopWithX();
                    } else {

                        // Convert to field relative speeds & send command
                        ChassisSpeeds speeds =
                            new ChassisSpeeds(
                                linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() / 4,
                                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() / 4,
                                omega);
                        boolean isFlipped =
                            DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get() == Alliance.Red;
                        drive.runVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                speeds,
                                isFlipped
                                    ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                    : drive.getRotation()));
                    }
                },
                drive)

            // Reset PID controller when command starts
            .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
    }

    // READ THIS: Follow a velocity request through the drive command framework.
    public static Command joystickDriveFollowingVelocity(
        Drive drive,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier) {

        // Create PID controller
        ProfiledPIDController angleController =
            new ProfiledPIDController(
                ANGLE_KP,
                0.0,
                ANGLE_KD,
                new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.run(
                () -> {

                    // Get linear velocity
                    Translation2d linearVelocity =
                        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
                            ySupplier.getAsDouble());

                    // Get target rotation
                    Rotation2d targetRotation = new Rotation2d(
                        drive.getChassisSpeeds().vxMetersPerSecond,
                        drive.getChassisSpeeds().vyMetersPerSecond);

                    // Calculate angular speed
                    double omega =
                        angleController.calculate(
                            drive.getRotation().getRadians(), targetRotation.getRadians());

                    // Convert to field relative speeds & send command
                    ChassisSpeeds speeds =
                        new ChassisSpeeds(
                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                            omega);
                    boolean isFlipped =
                        DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red;
                    drive.runVelocity(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                            speeds,
                            isFlipped
                                ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                : drive.getRotation()));
                }
            )

            // Reset PID controller when command starts
            .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
    }

    // STUDENT NOTE: Build a command designed to cross a field bump.
    public static Command joystickDriveOverBump(
        Drive drive,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier
    ) {

        // Create PID controller
        ProfiledPIDController angleController =
            new ProfiledPIDController(
                ANGLE_KP,
                0.0,
                ANGLE_KD,
                new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Calculate nearest angle to snap to
        Rotation2d rot = Arrays.stream(AutoConstants.BUMP_TRAVERSAL_HEADINGS).min(
                Comparator.comparingDouble(a -> Math.abs(a.minus(drive.getRotation()).getRadians())))
            .orElse(Rotation2d.kZero);

        return Commands.run(() -> {

                    // Get linear velocity
                    Translation2d linearVelocity =
                        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
                            ySupplier.getAsDouble());

                    // Calculate angular speed
                    double omega =
                        angleController.calculate(
                            drive.getRotation().getRadians(), rot.getRadians());

                    // Convert to field relative speeds & send command
                    ChassisSpeeds speeds =
                        new ChassisSpeeds(
                            linearVelocity.getX() * AutoConstants.BUMP_TRAVERSAL_SPEED,
                            linearVelocity.getY() * AutoConstants.BUMP_TRAVERSAL_SPEED,
                            omega);
                    boolean isFlipped =
                        DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red;
                    drive.runVelocity(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                            speeds,
                            isFlipped
                                ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                : drive.getRotation()));
                },
                drive)

            // Reset PID controller when command starts
            .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
    }


    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>This command should only be used in voltage control mode.
     */
    // STUDENT NOTE: Build a motor test used to estimate drive feedforward settings.
    public static Command feedforwardCharacterization(Drive drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(

            // Reset data
            Commands.runOnce(
                () -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }),

            // Allow modules to orient
            Commands.run(
                    () -> drive.runCharacterization(0.0),
                    drive)
                .withTimeout(FF_START_DELAY),

            // Start timer
            Commands.runOnce(timer::restart),

            // Accelerate and gather data
            Commands.run(
                    () -> {
                        double voltage = timer.get() * FF_RAMP_RATE;
                        drive.runCharacterization(voltage);
                        velocitySamples.add(drive.getFeedforwardTestSpeed());
                        voltageSamples.add(voltage);
                    },
                    drive)

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                        int n = velocitySamples.size();
                        double sumX = 0.0;
                        double sumY = 0.0;
                        double sumXY = 0.0;
                        double sumX2 = 0.0;
                        for (int i = 0; i < n; i++) {
                            sumX += velocitySamples.get(i);
                            sumY += voltageSamples.get(i);
                            sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                            sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                        }
                        double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                        double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                        NumberFormat formatter = new DecimalFormat("#0.00000");
                        System.out.println(
                            "********** Drive FF Characterization Results **********");
                        System.out.println("\tkS: " + formatter.format(kS));
                        System.out.println("\tkV: " + formatter.format(kV));
                    }));
    }

    /**
     * Measures the robot's wheel radius by spinning in a circle.
     */
    // STUDENT NOTE: Build a motion test used to estimate effective wheel radius.
    public static Command wheelRadiusCharacterization(Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(

            // Drive control sequence
            Commands.sequence(

                // Reset acceleration limiter
                Commands.runOnce(
                    () -> limiter.reset(0.0)),

                // Turn in place, accelerating up to full speed
                Commands.run(
                    () -> {
                        double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                        drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                    },
                    drive)),

            // Measurement sequence
            Commands.sequence(

                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),

                // Record starting measurement
                Commands.runOnce(
                    () -> {
                        state.positions = drive.getWheelRotationsForRadiusCalibration();
                        state.lastAngle = drive.getRotation();
                        state.gyroDelta = 0.0;
                    }),

                // Update gyro delta
                Commands.run(
                        () -> {
                            var rotation = drive.getRotation();
                            state.gyroDelta += Math.abs(
                                rotation.minus(state.lastAngle).getRadians());
                            state.lastAngle = rotation;
                        })

                    // When cancelled, calculate and print results
                    .finallyDo(
                        () -> {
                            double[] positions = drive.getWheelRotationsForRadiusCalibration();
                            double wheelDelta = 0.0;
                            for (int i = 0; i < 4; i++) {
                                wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                            }
                            double wheelRadius =
                                (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                            NumberFormat formatter = new DecimalFormat("#0.000");
                            System.out.println(
                                "********** Wheel Radius Characterization Results **********");
                            System.out.println(
                                "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                            System.out.println(
                                "\tGyro Delta: " + formatter.format(state.gyroDelta)
                                    + " radians");
                            System.out.println(
                                "\tWheel Radius: "
                                    + formatter.format(wheelRadius)
                                    + " meters, "
                                    + formatter.format(Units.metersToInches(wheelRadius))
                                    + " inches");
                        })));
    }

    private static class WheelRadiusCharacterizationState {

        double[] positions = new double[4];
        Rotation2d lastAngle = Rotation2d.kZero;
        double gyroDelta = 0.0;
    }
}

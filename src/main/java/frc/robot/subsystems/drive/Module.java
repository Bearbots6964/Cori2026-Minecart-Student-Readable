// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

// FILE GUIDE: One swerve wheel assembly: translates wheel speed/angle requests to a ModuleIO and records measured feedback.
public class Module {

    // STUDENT MAP: ONE SWERVE-DRIVE WHEEL ASSEMBLY
    // PURPOSE: Coordinates one wheel drive motor and one steering motor through ModuleIO.
    // LOOK FOR: The desired wheel state contains speed and angle; measured wheel state comes from sensors.
  private final ModuleIO swerveModuleHardware;
  private final ModuleIOInputsAutoLogged moduleSensorReadings = new ModuleIOInputsAutoLogged();
  private final int moduleIndex;
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  private final Alert driveMotorDisconnectedAlert;
  private final Alert steeringMotorDisconnectedAlert;
  private final Alert steeringEncoderDisconnectedAlert;
    /**
     * -- GETTER --
     * Returns the module positions received this cycle.
     */
    @Getter
    private SwerveModulePosition[] sampledModulePositions = new SwerveModulePosition[] {};

  // READ THIS: Associate a physical/simulated module with its configuration and sensor data.
  public Module(
      ModuleIO swerveModuleHardware,
      int moduleIndex,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.swerveModuleHardware = swerveModuleHardware;
    this.moduleIndex = moduleIndex;
    this.constants = constants;
    driveMotorDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(moduleIndex) + ".",
            AlertType.kError);
    steeringMotorDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(moduleIndex) + ".", AlertType.kError);
    steeringEncoderDisconnectedAlert =
        new Alert(
            "Disconnected turn encoder on module " + Integer.toString(moduleIndex) + ".",
            AlertType.kError);
  }

  // STUDENT NOTE: Read real or simulated data for this one wheel module.
  public void periodic() {
    swerveModuleHardware.updateInputs(moduleSensorReadings);
    Logger.processInputs("Drive/Module" + Integer.toString(moduleIndex), moduleSensorReadings);

    // Calculate positions for odometry
    // All signals are sampled together
    int sampleCount = moduleSensorReadings.odometryTimestamps.length;
    sampledModulePositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = moduleSensorReadings.odometryDriveWheelPositionRadians[i] * constants.WheelRadius;
      Rotation2d angle = moduleSensorReadings.odometrySteeringAngles[i];
      sampledModulePositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveMotorDisconnectedAlert.set(!moduleSensorReadings.driveMotorConnected);
    steeringMotorDisconnectedAlert.set(!moduleSensorReadings.steeringMotorConnected);
    steeringEncoderDisconnectedAlert.set(!moduleSensorReadings.steeringEncoderConnected);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  // STUDENT NOTE: Choose wheel motor speed and steering direction for the requested state.
  public void applyRequestedWheelState(SwerveModuleState state) {

    // Optimize velocity setpoint
    state.optimize(getAngle());
    state.cosineScale(moduleSensorReadings.steeringAngle);

    // Apply setpoints
    swerveModuleHardware.setDriveWheelSpeed(state.speedMetersPerSecond / constants.WheelRadius);
    swerveModuleHardware.setSteeringAngleTarget(state.angle);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    swerveModuleHardware.setDriveMotorOpenLoopOutput(output);
    swerveModuleHardware.setSteeringAngleTarget(Rotation2d.kZero);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    swerveModuleHardware.setDriveMotorOpenLoopOutput(0.0);
    swerveModuleHardware.setSteeringMotorOpenLoopOutput(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return moduleSensorReadings.steeringAngle;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return moduleSensorReadings.driveWheelPositionRadians * constants.WheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return moduleSensorReadings.driveWheelVelocityRadiansPerSecond * constants.WheelRadius;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

    /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return moduleSensorReadings.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRotationForRadiusCalibration() {
    return moduleSensorReadings.driveWheelPositionRadians;
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFeedforwardTestSpeed() {
    return Units.radiansToRotations(moduleSensorReadings.driveWheelVelocityRadiansPerSecond);
  }
}

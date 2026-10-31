// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;


import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

// FILE GUIDE: Swerve module hardware contract: reports drive/steer sensor values and accepts wheel speed and angle requests.
public interface ModuleIO {

    // STUDENT MAP: SWERVE MODULE HARDWARE INTERFACE
    // PURPOSE: Defines the measurements and motor instructions used for one steering-and-drive wheel module.
    // LOOK FOR: The same interface lets Drive.java use real hardware or simulation.
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveMotorConnected = false;
    public double driveWheelPositionRadians = 0.0;
    public double driveWheelVelocityRadiansPerSecond = 0.0;
    public double driveMotorAppliedVolts = 0.0;
    public double driveMotorCurrentAmps = 0.0;

    public boolean steeringMotorConnected = false;
    public boolean steeringEncoderConnected = false;
    public Rotation2d absoluteSteeringAngle = Rotation2d.kZero;
    public Rotation2d steeringAngle = Rotation2d.kZero;
    public double steeringAngularVelocityRadiansPerSecond = 0.0;
    public double steeringMotorAppliedVolts = 0.0;
    public double steeringMotorCurrentAmps = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDriveWheelPositionRadians = new double[] {};
    public Rotation2d[] odometrySteeringAngles = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs measuredInputs) {}

  /** Request an unregulated drive-motor output (not a target wheel speed). */
  public default void setDriveMotorOpenLoopOutput(double output) {}

  /** Request an unregulated steering-motor output (not a target angle). */
  public default void setSteeringMotorOpenLoopOutput(double output) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveWheelSpeed(double velocityRadPerSec) {}

  /** Run the turn motor to the specified rotation. */
  public default void setSteeringAngleTarget(Rotation2d rotation) {}
}

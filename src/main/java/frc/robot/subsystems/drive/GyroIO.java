// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;


import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

// FILE GUIDE: Gyro hardware contract: measured heading, rate, and connection status flow toward the drive estimator.
public interface GyroIO {

    // STUDENT MAP: GYRO SENSOR INTERFACE
    // PURPOSE: Defines the rotation readings Drive.java expects from a gyro.
    // LOOK FOR: The real and simulated gyro implementations fill the same fields.
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = Rotation2d.kZero;
    public double yawVelocityRadPerSec = 0.0;
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  // READ THIS: Read available sensor signals into the measured-inputs object.
  public default void updateInputs(GyroIOInputs measuredInputs) {}
}

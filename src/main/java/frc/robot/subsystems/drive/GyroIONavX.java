// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;


import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {

    // STUDENT MAP: ALTERNATIVE REAL GYRO
    // PURPOSE: Reads heading and rotation from a NavX gyro if this implementation is used.
    // LOOK FOR: RobotContainer currently chooses Pigeon2 for the real robot.
  private final AHRS navXGyro = new AHRS(NavXComType.kMXP_SPI, (byte) Drive.ODOMETRY_FREQUENCY);
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  // READ THIS: Construct the NavX sensor connection used to read heading.
  public GyroIONavX() {
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(navXGyro::getYaw);
  }

  // READ THIS: Read available sensor signals into the measured-inputs object.
  @Override
  public void updateInputs(GyroIOInputs measuredInputs) {
    measuredInputs.connected = navXGyro.isConnected();
    measuredInputs.yawPosition = Rotation2d.fromDegrees(-navXGyro.getYaw());
    measuredInputs.yawVelocityRadPerSec = Units.degreesToRadians(-navXGyro.getRawGyroZ());

    measuredInputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    measuredInputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(-value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}

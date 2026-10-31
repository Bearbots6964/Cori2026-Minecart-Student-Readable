// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.generated.TunerConstants;
import java.util.Objects;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {

    // STUDENT MAP: REAL PIGEON2 DIRECTION SENSOR
    // PURPOSE: Obtains robot heading and rotation information from the CTRE Pigeon2.
    // LOOK FOR: The numbers reported here are measurements, not target angles.
  private final Pigeon2 pigeonGyro =
      new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);
  private final StatusSignal<Angle> yaw = pigeonGyro.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeonGyro.getAngularVelocityZWorld();

  // READ THIS: Configure the CTRE Pigeon2 gyro connection.
  public GyroIOPigeon2() {
      pigeonGyro.getConfigurator().apply(
          Objects.requireNonNullElseGet(TunerConstants.DrivetrainConstants.Pigeon2Configs,
              Pigeon2Configuration::new));

    pigeonGyro.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(50.0);
    pigeonGyro.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone());
  }

  // READ THIS: Read available sensor signals into the measured-inputs object.
  @Override
  public void updateInputs(GyroIOInputs measuredInputs) {
    measuredInputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    measuredInputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    measuredInputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    measuredInputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    measuredInputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map(Rotation2d::fromDegrees)
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}

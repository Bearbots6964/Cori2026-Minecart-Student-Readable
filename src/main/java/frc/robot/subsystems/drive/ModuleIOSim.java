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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ModuleIOSim implements ModuleIO {

    // STUDENT MAP: SWERVE MODULE SIMULATION
    // PURPOSE: Simulates a wheel module and reports virtual wheel speed, angle, current, and position.
    // LOOK FOR: Results are predictions from the simulation model, not measurements from real hardware.

  // TunerConstants doesn't support separate sim constants, so they are declared
  // locally
  private static final double DRIVE_KP = 0.05;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_KS = 0.0;
  private static final double DRIVE_KV_ROT =

      // Same units as TunerConstants: (volt * secs) / rotation
      0.91035;
  private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
  private static final double TURN_KP = 8.0;
  private static final double TURN_KD = 0.0;
  private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
  private PIDController turnController = new PIDController(TURN_KP, 0, TURN_KD);
  private double driveFFVolts = 0.0;
  private double driveMotorAppliedVolts = 0.0;
  private double steeringMotorAppliedVolts = 0.0;

  // READ THIS: Connect this wheel to its simulated drive and steering motors.
  public ModuleIOSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {

    // Create drive and turn sim models
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DRIVE_GEARBOX, constants.DriveInertia, constants.DriveMotorGearRatio),
            DRIVE_GEARBOX);
    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                TURN_GEARBOX, constants.SteerInertia, constants.SteerMotorGearRatio),
            TURN_GEARBOX);

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // READ THIS: Read available sensor signals into the measured-inputs object.
  @Override
  public void updateInputs(ModuleIOInputs measuredInputs) {

    // Run closed-loop control
    if (driveClosedLoop) {
      driveMotorAppliedVolts =
          driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      steeringMotorAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
    } else {
      turnController.reset();
    }

    // Update simulation state
    driveSim.setInputVoltage(MathUtil.clamp(driveMotorAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(MathUtil.clamp(steeringMotorAppliedVolts, -12.0, 12.0));
    driveSim.update(0.02);
    turnSim.update(0.02);

    // Update drive inputs
    measuredInputs.driveMotorConnected = true;
    measuredInputs.driveWheelPositionRadians = driveSim.getAngularPositionRad();
    measuredInputs.driveWheelVelocityRadiansPerSecond = driveSim.getAngularVelocityRadPerSec();
    measuredInputs.driveMotorAppliedVolts = driveMotorAppliedVolts;
    measuredInputs.driveMotorCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    // Update turn inputs
    measuredInputs.steeringMotorConnected = true;
    measuredInputs.steeringEncoderConnected = true;
    measuredInputs.absoluteSteeringAngle = new Rotation2d(turnSim.getAngularPositionRad());
    measuredInputs.steeringAngle = new Rotation2d(turnSim.getAngularPositionRad());
    measuredInputs.steeringAngularVelocityRadiansPerSecond = turnSim.getAngularVelocityRadPerSec();
    measuredInputs.steeringMotorAppliedVolts = steeringMotorAppliedVolts;
    measuredInputs.steeringMotorCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
    // matter)
    measuredInputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    measuredInputs.odometryDriveWheelPositionRadians = new double[] {measuredInputs.driveWheelPositionRadians};
    measuredInputs.odometrySteeringAngles = new Rotation2d[] {measuredInputs.steeringAngle};
  }

  // READ THIS: Update the drive motor open loop output request or setting used below.
  @Override
  public void setDriveMotorOpenLoopOutput(double output) {
    driveClosedLoop = false;
    driveMotorAppliedVolts = output;
  }

  // READ THIS: Update the steering motor open loop output request or setting used below.
  @Override
  public void setSteeringMotorOpenLoopOutput(double output) {
    turnClosedLoop = false;
    steeringMotorAppliedVolts = output;
  }

  // READ THIS: Update the drive wheel speed request or setting used below.
  @Override
  public void setDriveWheelSpeed(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  // READ THIS: Update the steering angle target request or setting used below.
  @Override
  public void setSteeringAngleTarget(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}

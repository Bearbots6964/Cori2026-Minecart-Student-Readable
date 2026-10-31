// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;


import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/**
 * Boundary between intake logic and hardware. Intake reads measured values
 * from IntakeIOInputs and sends motor requests through the methods below.
 * The real implementation is IntakeIOTalonFX; simulation may provide a
 * no-op implementation. Do not rename input fields casually: they are
 * used by generated AdvantageKit logging code.
 */
public interface IntakeIO {

    // STUDENT MAP: INTAKE HARDWARE INTERFACE
    // PURPOSE: Defines the measured inputs and instructions that the intake subsystem expects from its hardware.
    // LOOK FOR: An interface method such as setArmTargetAngle is a request; the real implementation talks to motor controllers.

    // These fields are measurements populated by updateInputs, not target values.
    @AutoLog
    public static class IntakeIOInputs {
        public boolean rollerLeaderMotorConnected = false;
        public boolean rollerFollowerMotorConnected = false;
        public boolean armMotorConnected = false;

        public double rollerLeaderPositionDegrees = 0.0;
        public double rollerLeaderVelocityDegreesPerSecond = 0.0;
        public double rollerLeaderAppliedVolts = 0.0;
        public double rollerLeaderCurrentAmps = 0.0;

        public double armPositionDegrees = 0.0;
        public double armVelocityDegreesPerSecond = 0.0;
        public double armAppliedVolts = 0.0;
        public double armCurrentAmps = 0.0;

        public double armAbsoluteEncoderPositionRotations = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(IntakeIOInputs measuredInputs) {}

    /** Ask the roller motor controller to apply this voltage. */
    public default void setRollerVoltage(Voltage volts) {}

    /** Ask the pivot-arm motor controller to apply this voltage. */
    public default void setArmVoltage(Voltage volts) {}

    /** Request an arm angle using the primary control gains. */
    public default void setArmTargetAngle(Angle angle) {}
    /** Request an arm angle using a second set of motor-control gains. */
    public default void setArmTargetAngleWithAlternateGains(Angle angle) {}

    /** Stop the rollers; this does not change the arm target. */
    public default void stopRollerMotors() {}

    /** Stop the arm motor; this does not change roller speed. */
    public default void stopArmMotor() {}

    /** Request a roller speed. Velocity is rotations per second. */
    public default void setRollerSpeed(AngularVelocity velocity) {}

    /** Recalibrate the arm motor sensor by adding this many degrees. */
    public default void adjustArmSensorPositionDegrees(double degrees) {}

}

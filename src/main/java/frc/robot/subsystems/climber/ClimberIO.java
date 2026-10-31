// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;


import org.littletonrobotics.junction.AutoLog;

/** Hardware boundary for climber feedback and motor requests. */
public interface ClimberIO {

    // STUDENT MAP: CLIMBER HARDWARE INTERFACE
    // PURPOSE: Lists sensor readings and motor instructions for the climbing mechanism.
    // LOOK FOR: Climber.java uses this interface without needing hardware-vendor details.
    @AutoLog
        public static class ClimberIOInputs{
            public boolean climberMotorConnected = false;
            public boolean climberEncoderConnected = false;
            public double climberMotorPositionDegrees = 0.0;
            public double climberMotorVelocityDegreesPerSec = 0.0;
            public double climberMotorAppliedVolts = 0.0;
            public double climberMotorCurrentAmps = 0.0;
            public double climberEncoderAbsolutePositionDegrees = 0.0;
        }
     /** Updates the set of loggable inputs. */
    public default void updateInputs(ClimberIOInputs measuredInputs) {}
    /** Run the climber motor at the specified voltage. */
    public default void setClimberMotorVoltage(double volts) {}
    /** Stop the climber motor. */
    public default void stopClimber() {}

    }

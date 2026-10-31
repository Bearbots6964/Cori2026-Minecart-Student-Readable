// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.Degrees;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;

/** Physical climber motor, CANcoder, motion targets, and CTRE controller settings. */
public class ClimberConstants {

    // STUDENT MAP: CLIMBER MOTOR AND POSITION VALUES
    // PURPOSE: Defines climber targets, tolerances, and motor settings.
    // LOOK FOR: These numbers belong to the physical mechanism and should be verified on hardware.
    public static final class ClimberMotorConstants {
        public static final int climberMotorCanId = 5;

        // Encoder and rotor rotations must be converted into climber mechanism rotations.
        public static final double encoderToMechanismRatio = 2.0;
        public static final double rotorToEncoderRatio = 45.0/2.0;

        public static final double climberCommandVoltage = 6.0;
        public static final Angle climbedPositionDegrees = Degrees.of(90.0);
        public static final Angle loweredPositionDegrees = Degrees.of(0.0);
        public static final Angle climberPositionTolerance = Degrees.of(0.5);

        //safety timeout — tune based on how long a full climb takes
        public static final double climberMotionTimeoutSeconds = 10.0;

        public static final Slot0Configs climberMotorPidGains = new Slot0Configs()
                .withKP(0.0)
                .withKD(0.0)
                .withKS(0.0)
                .withKV(0.0);

        public static final CurrentLimitsConfigs climberMotorCurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(20.0);

        public static final MotorOutputConfigs climberMotorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
    }

    //climber encoder constants, including CAN ID, magnet offset, and sensor direction
    public static final class ClimberEncoderConstants {
        public static final double absoluteSensorDiscontinuityPoint = 0.5;
        public static final SensorDirectionValue sensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        public static final double magnetOffset = 0.40380859375;

        public static final int climberEncoderCanId = 6;
    }


}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;


import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import java.util.EnumMap;

    /**
     * Competition intake targets, roller velocities, motor IDs, and CTRE
     * configuration. This class contains robot-specific calibration.
     */
public class IntakeConstants {

    // STUDENT MAP: INTAKE TARGETS AND MOTOR SETTINGS
    // PURPOSE: Maps a named intake-arm position to an angle and a named roller action to speed.
    // LOOK FOR: RETRACTED and DEPLOYED are requested positions, while measured arm position comes from IntakeIO.


    // Arm positions use the angular reference configured for the physical
    // arm encoder. These numbers are calibration, not teaching examples.
    // Confirm the current mechanical zero before changing them.
    public static final Angle DEPLOYED_ANGLE = Degrees.of(-25.2232050549);
    public static final Angle RETRACTED_ANGLE = Degrees.of(103.966125);
    public static final Angle TILT_ANGLE = Degrees.of(60.0);
    public static final Angle ARM_POSITION_TOLERANCE = Degrees.of(5.0);

    public static final Angle START_ANGLE = Degrees.of(91.758);

    /** Requested arm target, not a measurement of where the arm is now. */
    public enum IntakeArmPosition {
        RETRACTED,
        DEPLOYED,
        TILTED
    }

    /** Requested speed/direction for the intake rollers only. */
    public enum RollerAction {
        RUNNING,
        RUNNING_SLOWLY,
        STOPPED,
        REVERSING,
        REVERSING_SLOWLY
    }

    // Maps state names to actual targets. Values are intentionally identical
    // to the competition code; changing these values changes robot motion.
    public static EnumMap<IntakeArmPosition, Double> armTargetDegreesByPosition = new EnumMap<>(IntakeArmPosition.class);
    public static EnumMap<RollerAction, Double> rollerSpeedRpsByAction = new EnumMap<>(RollerAction.class);
    static {
        armTargetDegreesByPosition.put(IntakeArmPosition.RETRACTED, RETRACTED_ANGLE.in(Degrees));
        armTargetDegreesByPosition.put(IntakeArmPosition.DEPLOYED, DEPLOYED_ANGLE.in(Degrees));
        armTargetDegreesByPosition.put(IntakeArmPosition.TILTED, TILT_ANGLE.in(Degrees));

        rollerSpeedRpsByAction.put(RollerAction.RUNNING, 21.0);
        rollerSpeedRpsByAction.put(RollerAction.RUNNING_SLOWLY, 5.0);
        rollerSpeedRpsByAction.put(RollerAction.STOPPED, 0.0);
        rollerSpeedRpsByAction.put(RollerAction.REVERSING, -21.0);
        rollerSpeedRpsByAction.put(RollerAction.REVERSING_SLOWLY, -5.0);
    }

    public static final double tiltCycleDurationSeconds = 1.0;


    public static final class RollerMotorConstants {

        public static final int rollerLeaderCanId = 2;
        public static final int rollerFollowerCanId = 3;

        // Software setting is 3.0; old mechanical note said 5:1. Verify before changing.
        public static final double gearRatio = 3.0;
        public static final Voltage rollerVoltageLimit = Volts.of(11.0);
        public static final AngularVelocity rollerTargetSpeedRps = RotationsPerSecond.of(21.0);

        public static final Slot0Configs rollerMotorPidGains = new Slot0Configs()
            .withKP(0.0024374)
            .withKD(0.0)
            .withKS(0.26554)
            .withKV(0.3734)
            .withKA(0.0089656);

        public static final CurrentLimitsConfigs rollerMotorCurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(70.0);

        public static final MotorOutputConfigs rollerMotorOutputConfiguration = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
    }


    public static final class ArmMotorConstants {
        public static final int armMotorCanId = 1;

        // set
        public static final int armEncoderCanId = 11;

        // 48:1 software value; verify physical stages before tuning.
        public static final double gearRatio = 4.0 * 4.0 * 3.0;
        public static final double armVoltageLimit = 6.0;
        public static final double armDeployTimeoutSeconds = 5.0;
        public static final double armRetractTimeoutSeconds = 5.0;

        // https://www.reca.lc/arm?armMass=%7B%22s%22%3A9.781%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A10.7756464614%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A80%2C%22u%22%3A%22A%22%7D&efficiency=100&endAngle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22Kraken%20X44%20%28FOC%29%22%7D&ratio=%7B%22magnitude%22%3A48%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D
        public static final Slot0Configs armMotorPidGains = new Slot0Configs()
            .withKP(69.211)
            .withKD(10)
            .withKS(1.0728)
            .withKV(2.5138)
            .withKA(1.2276)
            .withKG(0.303)
            .withGravityArmPositionOffset(-0.156)
            .withGravityType(GravityTypeValue.Arm_Cosine);
        public static final Slot1Configs armRetractingPidGains = new Slot1Configs()
            .withKP(60.0)
            .withKD(2.0)
            .withKS(0.5)
            .withKV(4.69)
            .withKA(0.10)
            .withKG(10.0)
            .withGravityArmPositionOffset(RETRACTED_ANGLE.minus(Degrees.of(81.123)))
            .withGravityType(GravityTypeValue.Arm_Cosine);


        public static final CurrentLimitsConfigs armMotorCurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(120.0);

        public static final MotorOutputConfigs armMotorOutputConfiguration = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

        public static final FeedbackConfigs armEncoderFeedbackConfiguration = new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
            .withFeedbackRemoteSensorID(armEncoderCanId)
            .withRotorToSensorRatio(gearRatio)
            .withSensorToMechanismRatio(1);

        public static final SoftwareLimitSwitchConfigs armSoftwareLimitConfiguration = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(RETRACTED_ANGLE)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(DEPLOYED_ANGLE);

        public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(0.5)
            .withMotionMagicAcceleration(1.0);

        public static final CANcoderConfiguration CANCODER_CONFIGS = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withMagnetOffset(0.3956434462)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            );
    }

}

package frc.robot.subsystems.shooter;


import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.ShooterCalculator.ShotData;

// FILE GUIDE: Shooter motor gains, target ranges, and lookup settings. Numeric values are preserved from the source project.
public class ShooterConstants {

    // STUDENT MAP: SHOOTER CALIBRATION AND SHOT TABLES
    // PURPOSE: Stores physical configuration and lookup points for desired speed and angle at different distances.
    // LOOK FOR: Do not edit numbers to improve readability: these points should come from hardware testing.
    public static final double turretToleranceDegrees = 2.0;

    public static final class TargetingConstants {
        public static final Translation3d PASSING_SPOT_LEFT = new Translation3d(
            Inches.of(90), Units.Meters.of(FieldConstants.fieldWidth / 2).plus(Inches.of(85)), Inches.zero());
        public static final Translation3d PASSING_SPOT_CENTER =
            new Translation3d(Inches.of(90), Units.Meters.of(FieldConstants.fieldWidth / 2), Inches.zero());
        public static final Translation3d PASSING_SPOT_RIGHT = new Translation3d(
            Inches.of(90), Units.Meters.of(FieldConstants.fieldWidth / 2).minus(Inches.of(85)), Inches.zero());

        public static final Distance DISTANCE_ABOVE_FUNNEL = Inches.of(20.0);

        public static final Distance FLYWHEEL_RADIUS = Inches.of(2.0);

        public static final InterpolatingTreeMap<Double, ShotData> SHOT_MAP = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), ShotData::interpolate);
        public static final InterpolatingDoubleTreeMap TOF_MAP = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap INDEXER_MAP = new InterpolatingDoubleTreeMap();

        // One lookup-table point: distance (meters), flywheel speed (rotations/second), hood angle (degrees).
        private static void addShotSpeedAndHoodCalibration(double distanceMeters, double flywheelSpeedRps, double hoodAngleDegrees) {
            SHOT_MAP.put(distanceMeters, new ShotData(flywheelSpeedRps, hoodAngleDegrees));
        }

        // One lookup-table point: distance (meters) and estimated flight time (seconds).
        private static void addFlightTimeCalibration(double distanceMeters, double flightTimeSeconds) {
            TOF_MAP.put(distanceMeters, flightTimeSeconds);
        }

        // One lookup-table point: distance (meters) and indexer speed (rotations/second).
        private static void addIndexerFeedSpeedCalibration(double distanceMeters, double indexerSpeedRps) {
            INDEXER_MAP.put(distanceMeters, indexerSpeedRps);
        }

        static {

            // Existing calibration points are below. TODO: verify/fill gaps using real robot testing.
            addShotSpeedAndHoodCalibration(1.5, 32, 0);
            addShotSpeedAndHoodCalibration(2, 32, 6);
            addShotSpeedAndHoodCalibration(2.5, 35, 9);
            addShotSpeedAndHoodCalibration(3, 35, 12);
            addShotSpeedAndHoodCalibration(3.3, 40, 13);

//            addShotSpeedAndHoodCalibration(3.5, 37, 13);
            addShotSpeedAndHoodCalibration(4, 40, 17);
            addShotSpeedAndHoodCalibration(4.5, 44, 17);
            addShotSpeedAndHoodCalibration(5, 50, 17);
            addShotSpeedAndHoodCalibration(10, 85, 30);

            addFlightTimeCalibration(2.5, 0.94);

            addIndexerFeedSpeedCalibration(2, 10);
            addIndexerFeedSpeedCalibration(2.5, 11);
            addIndexerFeedSpeedCalibration(3, 15);
            addIndexerFeedSpeedCalibration(3.5, 19);
            addIndexerFeedSpeedCalibration(5, 22);
            addIndexerFeedSpeedCalibration(10, 50);


//
        }

        public static final int LOOKAHEAD_ITERATIONS = 2;
    }

    public static final class TalonFXConstants {

        public static final class TurnMotorConstants {
            public static final int TURN_MOTOR_ID = 8;
            public static final int TURN_ENCODER_ID = 9;

            public static final Angle MAX_TURN_ANGLE = Degrees.of(180.0);
            public static final Angle MIN_TURN_ANGLE = Degrees.of(-190.0);

            public static final double MOTOR_TO_TURRET_RATIO = 4.8

                // 4.8:1 gearing from motor to encoder, 10:1 gearing from encoder to turret
                * 10.0;
            // 10:1 gearing from encoder to turret
            public static final double ENCODER_TO_TURRET_RATIO = -10.0;

            public static final Slot0Configs TURN_GAINS = new Slot0Configs()
                .withKP(59.757)
                .withKD(10.987)
                .withKS(0.355)
                .withKV(5.2411)
                .withKA(1.6317);

            public static final CurrentLimitsConfigs TURN_CURRENT_LIMITS = new CurrentLimitsConfigs()
                .withSupplyCurrentLowerLimit(30.0);

            public static final MotorOutputConfigs TURN_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

            public static final double MAX_OUTPUT_VOLTS = 7.0;
        }

        public static final class FlywheelMotorConstants {
            /*  ^ front/top
            M        F2
            ----------- -> right
            F1       F3
             */
            public static final int FLYWHEEL_MOTOR_ID = 6;
            public static final int FLYWHEEL_FOLLOWER_1_MOTOR_ID = 7;
            public static final int FLYWHEEL_FOLLOWER_2_MOTOR_ID = 8;
            public static final int FLYWHEEL_FOLLOWER_3_MOTOR_ID = 9;

            // https://www.reca.lc/flywheel?currentLimit=%7B%22s%22%3A80%2C%22u%22%3A%22A%22%7D&efficiency=100&flywheelMomentOfInertia=%7B%22s%22%3A0%2C%22u%22%3A%22in2%2Albs%22%7D&flywheelRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&flywheelRatio=%7B%22magnitude%22%3A1%2C%22ratioType%22%3A%22Reduction%22%7D&flywheelWeight=%7B%22s%22%3A1.5%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22Kraken%20X60%20%28FOC%29%22%7D&motorRatio=%7B%22magnitude%22%3A1%2C%22ratioType%22%3A%22Step-up%22%7D&projectileRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&projectileWeight=%7B%22s%22%3A0.5%2C%22u%22%3A%22lbs%22%7D&shooterMomentOfInertia=%7B%22s%22%3A4.124133%2C%22u%22%3A%22in2%2Albs%22%7D&shooterRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&shooterTargetSpeed=%7B%22s%22%3A4800%2C%22u%22%3A%22rpm%22%7D&shooterWeight=%7B%22s%22%3A1%2C%22u%22%3A%22lbs%22%7D&useCustomFlywheelMoi=1&useCustomShooterMoi=1
            public static final Slot0Configs FLYWHEEL_GAINS = new Slot0Configs()
                .withKP(0.15662)
                .withKD(0.0)
                .withKS(.5)
                .withKV(0.1358851675)
                .withKA(0.23979);

            public static final CurrentLimitsConfigs FLYWHEEL_CURRENT_LIMITS = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(120);

            public static final MotorOutputConfigs LEFT_FLYWHEEL_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast);
            public static final MotorOutputConfigs RIGHT_FLYWHEEL_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast);

            public static final double GEAR_RATIO = (34.0 / 30.0) * (26.0 / 28.0);
            public static final FeedbackConfigs FLYWHEEL_FEEDBACK_CONFIGS = new FeedbackConfigs()
                .withVelocityFilterTimeConstant(Seconds.of(0.0))
                .withSensorToMechanismRatio(GEAR_RATIO);

        }


        public static final Transform3d ROBOT_TO_TURRET_TRANSFORM = new Transform3d(
            Inches.of(0.0),
            Inches.of(0.0),
            Inches.of(21.875),
            new Rotation3d()
        );

        public static final class HoodMotorConstants {
            public static final int HOOD_MOTOR_ID = 10;
            public static final Angle MAX_HOOD_ANGLE = Degrees.of(15.0);

            public static final double MOTOR_TO_HOOD_RATIO = (50.0 / 14.0) * (360.0 / 50.0);

            public static final Slot0Configs HOOD_GAINS = new Slot0Configs()
                .withKP(90.0)
                .withKD(0.0)
                .withKS(0.339)
                .withKV(0.0)
                .withKA(0.0)
                .withKG(0.589)
                .withGravityType(GravityTypeValue.Elevator_Static);

            public static final MotionMagicConfigs HOOD_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(RotationsPerSecond.of(1.0))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(3.0));

            public static final CurrentLimitsConfigs HOOD_CURRENT_LIMITS = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40.0);

            public static final MotorOutputConfigs HOOD_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);


        }
    }

}

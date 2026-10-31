package frc.robot.subsystems.indexer;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// FILE GUIDE: Indexer motor constants, including the speed request and gearing used by the hardware adapter.
public class IndexerConstants {

    // STUDENT MAP: INDEXER HARDWARE SETTINGS
    // PURPOSE: Defines the CAN IDs, motor gains, and requested speeds for the indexer.
    // LOOK FOR: Motor gain values were tuned for this robot; they are not arbitrary sample numbers.
    public static final double MOTOR_SPEED_PERCENTAGE = 0.75;

    public static final double AGITATE_SPEED_PERCENTAGE = 0.2;
    public static final double AGITATE_DURATION_SECONDS = 0.1875;

    public static final class TalonFXConstants {
        public static final int MOTOR_ID = 4;
        public static final int FOLLOWER_MOTOR_ID = 5;

        // 5:1 gearbox with a 16:64 belt reduction
        public static final double GEAR_RATIO = 50.0 / 30;
        public static final Slot0Configs MOTOR_GAINS = new Slot0Configs()
                .withKP(0.0)
                .withKD(0.0)
                .withKS(.5)
                .withKV(0.353);

        public static final CurrentLimitsConfigs MOTOR_CURRENT_LIMITS = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(120)
            .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLowerLimit(30.0);

        public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast);

        public static final OpenLoopRampsConfigs MOTOR_OPEN_LOOP_RAMPS = new OpenLoopRampsConfigs()
            .withDutyCycleOpenLoopRampPeriod(0.25);

        public static final FeedbackConfigs  MOTOR_FEEDBACK_CONFIGS = new FeedbackConfigs()
            .withSensorToMechanismRatio(GEAR_RATIO);

    }

}

package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.shooter.ShooterConstants.TalonFXConstants.FlywheelMotorConstants;
import frc.robot.subsystems.shooter.ShooterConstants.TalonFXConstants.HoodMotorConstants;
import frc.robot.util.PhoenixUtil;

// FILE GUIDE: CTRE implementation of ShooterIO: configures and controls the flywheel and hood motors.
public class ShooterIOTalonFX implements ShooterIO {

    // STUDENT MAP: REAL SHOOTER MOTORS
    // PURPOSE: Controls the flywheel leader/followers and hood motor, and reads speeds, angles, currents, and voltages.
    // LOOK FOR: A control request is a desired value. StatusSignal values are reported measurements from the electronics.

    // TalonFX objects represent actual CAN-connected motor controllers.
    // One leader sets speed; three follower controllers copy its output.
    private final TalonFX flywheelLeaderMotor;
    private final TalonFX hoodAngleMotor;

    private final TalonFX flywheelFollowerMotor1;
    private final TalonFX flywheelFollowerMotor2;
    private final TalonFX flywheelFollowerMotor3;

    private final TalonFXConfiguration flywheelLeaderMotorConfig;
    private final TalonFXConfiguration hoodMotorConfig;

    private final TalonFXConfiguration flywheelFollower1Config;
    private final TalonFXConfiguration flywheelFollower2Config;
    private final TalonFXConfiguration flywheelFollower3Config;

    // StatusSignal objects represent measurements read FROM a controller.
    // They are different from the VelocityVoltage requests sent TO a motor.
    private final StatusSignal<AngularVelocity> measuredFlywheelSpeed;

    private final StatusSignal<Voltage> shooterFollower1Voltage;
    private final StatusSignal<Voltage> shooterFollower2Voltage;
    private final StatusSignal<Voltage> shooterFollower3Voltage;

    private final StatusSignal<Current> measuredFlywheelCurrent;
    private final StatusSignal<Voltage> measuredFlywheelVoltage;
    private final StatusSignal<AngularAcceleration> measuredFlywheelAcceleration;

    private final StatusSignal<Angle> measuredHoodMotorPosition;
    private final StatusSignal<Current> measuredHoodMotorCurrent;
    private final StatusSignal<Voltage> measuredHoodMotorVoltage;
    private final StatusSignal<AngularVelocity> measuredHoodMotorVelocity;

    // Prepared for an alternate motor-control method, but not used by live code.
    private final MotionMagicVoltage motionMagicHoodPositionRequest = new MotionMagicVoltage(0);
    private final PositionVoltage hoodPositionVoltageRequest = new PositionVoltage(0);

    private final VelocityVoltage flywheelSpeedControlRequest = new VelocityVoltage(0).withEnableFOC(false);
    private final VoltageOut flywheelVoltageControlRequest = new VoltageOut(0);

    private final Follower flywheelFollower1ControlRequest =
        new Follower(
            FlywheelMotorConstants.FLYWHEEL_MOTOR_ID,
            MotorAlignmentValue.Aligned);

    private final Follower flywheelFollower2ControlRequest =
        new Follower(
            FlywheelMotorConstants.FLYWHEEL_MOTOR_ID,
            MotorAlignmentValue.Opposed);

    private final Follower flywheelFollower3ControlRequest =
        new Follower(
            FlywheelMotorConstants.FLYWHEEL_MOTOR_ID,
            MotorAlignmentValue.Opposed);

    private final NeutralOut neutralOut = new NeutralOut();

    // STUDENT NOTE: Create and configure the real hood motor and all flywheel controllers.
    public ShooterIOTalonFX() {

        flywheelLeaderMotor = new TalonFX(FlywheelMotorConstants.FLYWHEEL_MOTOR_ID);
        flywheelFollowerMotor1 = new TalonFX(FlywheelMotorConstants.FLYWHEEL_FOLLOWER_1_MOTOR_ID);
        flywheelFollowerMotor2 = new TalonFX(FlywheelMotorConstants.FLYWHEEL_FOLLOWER_2_MOTOR_ID);
        flywheelFollowerMotor3 = new TalonFX(FlywheelMotorConstants.FLYWHEEL_FOLLOWER_3_MOTOR_ID);

        hoodAngleMotor = new TalonFX(HoodMotorConstants.HOOD_MOTOR_ID);

        flywheelLeaderMotorConfig =

            // Verify the motor direction against the actual wiring; settings are unchanged.
            new TalonFXConfiguration()
                .withSlot0(FlywheelMotorConstants.FLYWHEEL_GAINS)
                .withCurrentLimits(FlywheelMotorConstants.FLYWHEEL_CURRENT_LIMITS)
                .withMotorOutput(FlywheelMotorConstants.LEFT_FLYWHEEL_OUTPUT_CONFIGS);

        hoodMotorConfig =

            // Verify the motor direction against the actual wiring; settings are unchanged.
            new TalonFXConfiguration()
                .withSlot0(HoodMotorConstants.HOOD_GAINS)
                .withCurrentLimits(HoodMotorConstants.HOOD_CURRENT_LIMITS)
                .withMotorOutput(HoodMotorConstants.HOOD_OUTPUT_CONFIGS);
        hoodMotorConfig.Feedback.SensorToMechanismRatio = HoodMotorConstants.MOTOR_TO_HOOD_RATIO;

        flywheelFollower1Config =

            // Verify the motor direction against the actual wiring; settings are unchanged.
            new TalonFXConfiguration()
                .withCurrentLimits(FlywheelMotorConstants.FLYWHEEL_CURRENT_LIMITS)
                .withMotorOutput(FlywheelMotorConstants.LEFT_FLYWHEEL_OUTPUT_CONFIGS);

        flywheelFollower2Config =

            // Verify the motor direction against the actual wiring; settings are unchanged.
            new TalonFXConfiguration()
                .withCurrentLimits(FlywheelMotorConstants.FLYWHEEL_CURRENT_LIMITS)
                .withMotorOutput(FlywheelMotorConstants.RIGHT_FLYWHEEL_OUTPUT_CONFIGS);

        flywheelFollower3Config =

            // Verify the motor direction against the actual wiring; settings are unchanged.
            new TalonFXConfiguration()
                .withCurrentLimits(FlywheelMotorConstants.FLYWHEEL_CURRENT_LIMITS)
                .withMotorOutput(FlywheelMotorConstants.RIGHT_FLYWHEEL_OUTPUT_CONFIGS);

        // Send the chosen settings to each physical controller. tryUntilOk retries
        // configuration if the device does not confirm success immediately.
        PhoenixUtil.tryUntilOk(5, () -> flywheelLeaderMotor.getConfigurator().apply(flywheelLeaderMotorConfig));
        PhoenixUtil.tryUntilOk(
            5, () -> flywheelFollowerMotor1.getConfigurator().apply(flywheelFollower1Config));
        PhoenixUtil.tryUntilOk(
            5, () -> flywheelFollowerMotor2.getConfigurator().apply(flywheelFollower2Config));
        PhoenixUtil.tryUntilOk(
            5, () -> flywheelFollowerMotor3.getConfigurator().apply(flywheelFollower3Config));
        PhoenixUtil.tryUntilOk(5, () -> hoodAngleMotor.getConfigurator().apply(hoodMotorConfig));

        measuredFlywheelSpeed = flywheelLeaderMotor.getVelocity();
        measuredFlywheelVoltage = flywheelLeaderMotor.getMotorVoltage();
        measuredFlywheelCurrent = flywheelLeaderMotor.getStatorCurrent();
        measuredFlywheelAcceleration = flywheelLeaderMotor.getAcceleration();

        measuredHoodMotorCurrent = hoodAngleMotor.getStatorCurrent();
        measuredHoodMotorVoltage = hoodAngleMotor.getMotorVoltage();
        measuredHoodMotorPosition = hoodAngleMotor.getPosition();
        measuredHoodMotorVelocity = hoodAngleMotor.getVelocity();

        hoodMotorConfig.MotionMagic.MotionMagicCruiseVelocity = measuredHoodMotorVelocity.getValueAsDouble();

        shooterFollower1Voltage = flywheelFollowerMotor1.getMotorVoltage();
        shooterFollower2Voltage = flywheelFollowerMotor2.getMotorVoltage();
        shooterFollower3Voltage = flywheelFollowerMotor3.getMotorVoltage();

        // Choose how frequently the CAN bus reports these measurements.
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            measuredFlywheelSpeed,
            measuredFlywheelVoltage,
            measuredFlywheelCurrent,
            measuredFlywheelAcceleration,
            measuredHoodMotorPosition,
            measuredHoodMotorVelocity,
            measuredHoodMotorVoltage,
            measuredHoodMotorCurrent);

        BaseStatusSignal.setUpdateFrequencyForAll(
            4, shooterFollower1Voltage, shooterFollower2Voltage, shooterFollower3Voltage);

        flywheelLeaderMotor.optimizeBusUtilization();
        flywheelFollowerMotor1.optimizeBusUtilization();
        flywheelFollowerMotor2.optimizeBusUtilization();
        flywheelFollowerMotor3.optimizeBusUtilization();

        hoodAngleMotor.optimizeBusUtilization();

        // Tell followers to mirror or oppose the leader as configured.
        flywheelFollowerMotor1.setControl(flywheelFollower1ControlRequest);
        flywheelFollowerMotor2.setControl(flywheelFollower2ControlRequest);
        flywheelFollowerMotor3.setControl(flywheelFollower3ControlRequest);
    }

    @Override

    // STUDENT NOTE: Read electronic status signals and store measured voltages, speed, current, and angles.
    public void updateInputs(ShooterIOInputs measuredInputs) {
        measuredInputs.flywheelLeaderConnected =
            BaseStatusSignal.refreshAll(measuredFlywheelSpeed, measuredFlywheelVoltage, measuredFlywheelCurrent, measuredFlywheelAcceleration).isOK();

        measuredInputs.flywheelLeaderMeasuredSpeed = measuredFlywheelSpeed.getValue();
        measuredInputs.flywheelLeaderAppliedVoltage = measuredFlywheelVoltage.getValue();
        measuredInputs.flywheelLeaderMeasuredCurrent = measuredFlywheelCurrent.getValue();
        measuredInputs.flywheelLeaderMeasuredAcceleration = measuredFlywheelAcceleration.getValue();

        measuredInputs.flywheelFollower1Connected = BaseStatusSignal.refreshAll(shooterFollower1Voltage).isOK();
        measuredInputs.flywheelFollower2Connected = BaseStatusSignal.refreshAll(shooterFollower2Voltage).isOK();
        measuredInputs.flywheelFollower3Connected = BaseStatusSignal.refreshAll(shooterFollower3Voltage).isOK();

        measuredInputs.hoodMotorConnected =
            BaseStatusSignal.refreshAll(measuredHoodMotorPosition, measuredHoodMotorVelocity, measuredHoodMotorCurrent, measuredHoodMotorVoltage).isOK();

        measuredInputs.hoodMotorAngularVelocity = measuredHoodMotorVelocity.getValue();
        measuredInputs.hoodMotorVoltage = measuredHoodMotorVoltage.getValue();
        measuredInputs.hoodMotorCurrent = measuredHoodMotorCurrent.getValue();
        measuredInputs.hoodPosition = measuredHoodMotorPosition.getValue();
    }

    @Override

    // STUDENT NOTE: Ask the flywheel leader to reach a speed; followers follow it.
    public void setFlywheelSpeed(AngularVelocity speed) {
        flywheelLeaderMotor.setControl(flywheelSpeedControlRequest.withVelocity(speed));
        flywheelFollowerMotor1.setControl(flywheelFollower1ControlRequest);
        flywheelFollowerMotor2.setControl(flywheelFollower2ControlRequest);
        flywheelFollowerMotor3.setControl(flywheelFollower3ControlRequest);
    }

    @Override

    // STUDENT NOTE: Ask the hood motor to reach the desired angular position.
    public void setHoodPosition(Angle position) {
        hoodAngleMotor.setControl(hoodPositionVoltageRequest.withPosition(position));
    }

    @Override

    // STUDENT NOTE: Update hood control gains used by CTRE software.
    public void setHoodPIDConstants(double kP, double kD, double kV, double kS) {
        hoodMotorConfig.Slot0.kP = kP;
        hoodMotorConfig.Slot0.kD = kD;
        hoodMotorConfig.Slot0.kV = kV;
        hoodMotorConfig.Slot0.kS = kS;
        PhoenixUtil.tryUntilOk(5, () -> hoodAngleMotor.getConfigurator().apply(hoodMotorConfig));
    }

    @Override

    // STUDENT NOTE: Update flywheel control gains used by CTRE software.
    public void setFlywheelPIDConstants(double kP, double kD, double kV, double kS) {
        flywheelLeaderMotorConfig.Slot0.kP = kP;
        flywheelLeaderMotorConfig.Slot0.kD = kD;
        flywheelLeaderMotorConfig.Slot0.kV = kV;
        flywheelLeaderMotorConfig.Slot0.kS = kS;

        // Send the chosen settings to each physical controller. tryUntilOk retries
        // configuration if the device does not confirm success immediately.
        PhoenixUtil.tryUntilOk(5, () -> flywheelLeaderMotor.getConfigurator().apply(flywheelLeaderMotorConfig));
    }

    @Override

    // STUDENT NOTE: Request that the hood motor stop.
    public void stopHood() {
        hoodAngleMotor.setControl(neutralOut);
    }

    @Override

    // STUDENT NOTE: Request that all flywheel motors stop.
    public void stopFlywheel() {
        flywheelLeaderMotor.setControl(neutralOut);
        flywheelFollowerMotor1.setControl(neutralOut);
        flywheelFollowerMotor2.setControl(neutralOut);
        flywheelFollowerMotor3.setControl(neutralOut);
    }

    // READ THIS: Send a flywheel voltage request to its motor controller.
    public void setFlywheelVoltage(Voltage volts) {
        flywheelLeaderMotor.setControl(flywheelVoltageControlRequest.withOutput(volts));
        flywheelFollowerMotor1.setVoltage(volts.in(Units.Volts));
        flywheelFollowerMotor2.setVoltage(volts.unaryMinus().in(Units.Volts));
        flywheelFollowerMotor3.setVoltage(volts.unaryMinus().in(Units.Volts));
    }

    // READ THIS: Send a hood voltage request to its motor controller.
    public void setHoodVoltage(Voltage volts) {
        hoodAngleMotor.setVoltage(volts.in(Units.Volts));
    }
}

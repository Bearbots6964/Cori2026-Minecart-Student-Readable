package frc.robot.subsystems.indexer;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.indexer.IndexerConstants.TalonFXConstants;
import frc.robot.util.PhoenixUtil;

// FILE GUIDE: CTRE implementation of IndexerIO: controls leader and follower motors and records sensor feedback.
public class IndexerIOTalonFX implements IndexerIO {

    // STUDENT MAP: REAL INDEXER MOTOR
    // PURPOSE: Configures the motor moving fuel toward or away from the shooter and reads its status.
    // LOOK FOR: Speed requests are in revolutions per second; duty cycle is a fraction of available motor output.


    // Motor
    private final TalonFX indexerLeaderMotor;
    private final TalonFX indexerFollowerMotor;

    // Config
    private final TalonFXConfiguration indexerMotorConfiguration;

    // Status Signals
    StatusSignal<Current> indexerAmpsActive;
    StatusSignal<AngularVelocity> requestedIndexerSpeedRps;
    StatusSignal<Voltage> indexerAppliedVoltage;
    StatusSignal<Voltage> indexerFollowerVoltage;

    // Control Requests
    private final VoltageOut indexerVoltageRequest = new VoltageOut(0);
    private final DutyCycleOut indexerDutyCycleRequest = new DutyCycleOut(0.0);
    private final DutyCycleOut dutyCycleRequest2 = new DutyCycleOut(0.0);
    private final VelocityVoltage indexerVelocityRequest = new VelocityVoltage(0.0);
    private final NeutralOut indexerNeutralRequest = new NeutralOut();
    private final Follower indexerFollowerRequest = new Follower(TalonFXConstants.MOTOR_ID, MotorAlignmentValue.Opposed);

    // READ THIS: Configure the indexer leader and follower TalonFX controllers.
    public IndexerIOTalonFX() {
        indexerLeaderMotor = new TalonFX(TalonFXConstants.MOTOR_ID);
        indexerFollowerMotor = new TalonFX(TalonFXConstants.FOLLOWER_MOTOR_ID);

        indexerMotorConfiguration = new TalonFXConfiguration()
            .withSlot0(TalonFXConstants.MOTOR_GAINS)
            .withCurrentLimits(TalonFXConstants.MOTOR_CURRENT_LIMITS)
            .withOpenLoopRamps(TalonFXConstants.MOTOR_OPEN_LOOP_RAMPS)
            .withFeedback(TalonFXConstants.MOTOR_FEEDBACK_CONFIGS)
            .withMotorOutput(TalonFXConstants.MOTOR_OUTPUT_CONFIGS);

        PhoenixUtil.tryUntilOk(5, () -> indexerLeaderMotor.getConfigurator().apply(indexerMotorConfiguration));
        PhoenixUtil.tryUntilOk(5, () -> indexerFollowerMotor.getConfigurator().apply(indexerMotorConfiguration));

        indexerAmpsActive = indexerLeaderMotor.getStatorCurrent();
        requestedIndexerSpeedRps = indexerLeaderMotor.getVelocity();
        indexerAppliedVoltage = indexerLeaderMotor.getMotorVoltage();
        indexerFollowerVoltage = indexerFollowerMotor.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(50, indexerAmpsActive, requestedIndexerSpeedRps,
            indexerAppliedVoltage);

        indexerFollowerMotor.setControl(new Follower(TalonFXConstants.MOTOR_ID, MotorAlignmentValue.Opposed));

        indexerLeaderMotor.optimizeBusUtilization();
        indexerFollowerMotor.optimizeBusUtilization();

    }

    // READ THIS: Read available sensor signals into the measured-inputs object.
    @Override
    public void updateInputs(IndexerIOInputs measuredInputs) {
        measuredInputs.indexerLeaderMotorConnected = BaseStatusSignal.refreshAll(indexerAmpsActive,
                requestedIndexerSpeedRps, indexerAppliedVoltage)
            .isOK();
        measuredInputs.followerMotorConnected = BaseStatusSignal.refreshAll(indexerFollowerVoltage).isOK();

        measuredInputs.indexerSpeedRps = requestedIndexerSpeedRps.getValue().in(Units.RotationsPerSecond)
            / TalonFXConstants.GEAR_RATIO;

        measuredInputs.indexerCurrentAmps = indexerAmpsActive.getValueAsDouble();

        measuredInputs.indexerAppliedVoltage = indexerAppliedVoltage.getValueAsDouble();
    }

    // READ THIS: Send an indexer voltage request to its motor controller.
    @Override
    public void setIndexerVoltage(double volts) {
        indexerLeaderMotor.setControl(indexerVoltageRequest.withOutput(volts));
        indexerFollowerMotor.setControl(indexerFollowerRequest);
    }

    // READ THIS: Send an open-loop indexer percentage to its motor controller.
    @Override
    public void setIndexerDutyCycle(double input) {
        indexerLeaderMotor.setControl(indexerDutyCycleRequest.withOutput(input));
        indexerFollowerMotor.setControl(indexerFollowerRequest);
    }

    // READ THIS: Request a target indexer wheel speed in rotations per second.
    @Override
    public void setIndexerSpeedRps(double velocity) {
        indexerLeaderMotor.setControl(indexerVelocityRequest
            .withVelocity(Units.RotationsPerSecond.of(
                velocity)));
        indexerFollowerMotor.setControl(indexerFollowerRequest);

    }

    // READ THIS: Send the stop request through the hardware interface.
    @Override
    public void stop() {
        indexerLeaderMotor.stopMotor();
        indexerFollowerMotor.stopMotor();
    }

}

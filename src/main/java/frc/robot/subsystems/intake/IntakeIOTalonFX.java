// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;


import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.intake.IntakeConstants.ArmMotorConstants;
import frc.robot.subsystems.intake.IntakeConstants.RollerMotorConstants;

/** Physical TalonFX motors and CANcoder for the intake rollers and arm. */
public class IntakeIOTalonFX implements IntakeIO {

    // STUDENT MAP: REAL INTAKE MOTORS AND ENCODER
    // PURPOSE: Configures the roller motor controllers, arm motor, and position feedback, then sends motor instructions.
    // LOOK FOR: TalonFX is a motor controller; CANcoder supplies a position measurement. Configuration numbers must match actual wiring.


    // These TalonFX objects represent the two roller motors and arm motor.
    private final TalonFX rollerLeaderMotor;
    private final TalonFX rollerFollowerMotor;
    private final TalonFX armMotor;
    private final CANcoder armAbsoluteEncoder;

    // Observed roller motor values: measured feedback, not requested outputs.
    StatusSignal<Angle> rollerLeaderPositionRotationsSignal;
    StatusSignal<AngularVelocity> rollerLeaderSpeedRotationsPerSecondSignal;
    StatusSignal<Voltage> rollerLeaderAppliedVolts;
    StatusSignal<Current> rollerLeaderCurrentAmps;

    StatusSignal<Voltage> rollerFollowerAppliedVoltageSignal;

    // Observed arm position, velocity, voltage, and current.
    StatusSignal<Angle> armPositionRotationsSignal;
    StatusSignal<AngularVelocity> armSpeedRotationsPerSecondSignal;
    StatusSignal<Voltage> armAppliedVolts;
    StatusSignal<Current> armCurrentAmps;
    StatusSignal<Angle> armAbsoluteEncoderPositionSignal;

    //create a new request to reuse for setting voltages and using commands
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0);
    private final VelocityVoltage  rollerVelocityRequest = new VelocityVoltage(0);
    private final PositionVoltage armPositionRequest = new PositionVoltage(0);
    private final MotionMagicVoltage armMotionMagicRequest = new MotionMagicVoltage(0);
    private final NeutralOut neutralMotorRequest = new NeutralOut();

    // STUDENT NOTE: Create the TalonFX controllers and configure their motor and encoder behavior.
    public IntakeIOTalonFX() {

        // Setup sequence (all CAN IDs and settings come from constants):
        //1. config
        //2. create motor objects
        //3. apply config to motors, retrying up to 5 times with 0.25s delay
        //4. create status signals
        //5. set update frequency for all status signals
        //6. optimize bus utilization

        //configure the TalonFX for the intake motor,
        var rollerMotorConfiguration = new TalonFXConfiguration()
            .withMotorOutput(RollerMotorConstants.rollerMotorOutputConfiguration)
            .withCurrentLimits(RollerMotorConstants.rollerMotorCurrentLimits)
            .withOpenLoopRamps(new OpenLoopRampsConfigs()
                .withVoltageOpenLoopRampPeriod(0.5))
            .withSlot0(RollerMotorConstants.rollerMotorPidGains);
        rollerMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rollerMotorConfiguration.Feedback.SensorToMechanismRatio = 3.0;


        var armMotorConfiguration = new TalonFXConfiguration()
            .withMotorOutput(ArmMotorConstants.armMotorOutputConfiguration)
            .withCurrentLimits(ArmMotorConstants.armMotorCurrentLimits)
            .withSlot0(ArmMotorConstants.armMotorPidGains)
            .withSoftwareLimitSwitch(ArmMotorConstants.armSoftwareLimitConfiguration)
            .withMotionMagic(ArmMotorConstants.MOTION_MAGIC_CONFIGS)
            .withFeedback(ArmMotorConstants.armEncoderFeedbackConfiguration);
        armMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        armMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        armAbsoluteEncoder = new CANcoder(ArmMotorConstants.armEncoderCanId);
        tryUntilOk(5, () -> armAbsoluteEncoder.getConfigurator().apply(ArmMotorConstants.CANCODER_CONFIGS, 0.25));

        //create motor object and apply configuration to intake motor, retrying up to 5 times with 0.25s delay
        rollerLeaderMotor = new TalonFX(RollerMotorConstants.rollerLeaderCanId);
        tryUntilOk(5, () -> rollerLeaderMotor.getConfigurator().apply(rollerMotorConfiguration, 0.25));

        rollerFollowerMotor = new TalonFX(RollerMotorConstants.rollerFollowerCanId);
        tryUntilOk(5, () -> rollerFollowerMotor.getConfigurator().apply(rollerMotorConfiguration, 0.25));

        rollerFollowerAppliedVoltageSignal = rollerFollowerMotor.getMotorVoltage();

        //create motor object and apply configuration to deploy motor, retrying up to 5 times with 0.25s delay
        armMotor = new TalonFX(ArmMotorConstants.armMotorCanId);
        tryUntilOk(5, () -> armMotor.getConfigurator().apply(armMotorConfiguration, 0.25));

        // Original startup calibration: request this sensor position once.
        // This is not a physical homing movement or a limit-switch check;
        // leave it unchanged until encoder reference has been verified on the robot.
        armMotor.setPosition(IntakeConstants.START_ANGLE);

        //create status signals for intake motor
        rollerLeaderPositionRotationsSignal = rollerLeaderMotor.getPosition();
        rollerLeaderSpeedRotationsPerSecondSignal = rollerLeaderMotor.getVelocity();
        rollerLeaderAppliedVolts = rollerLeaderMotor.getMotorVoltage();
        rollerLeaderCurrentAmps = rollerLeaderMotor.getSupplyCurrent();

        //create status signals for deploy motor
        armPositionRotationsSignal = armMotor.getPosition();
        armSpeedRotationsPerSecondSignal = armMotor.getVelocity();
        armAppliedVolts = armMotor.getMotorVoltage();
        armCurrentAmps = armMotor.getSupplyCurrent();

        armAbsoluteEncoderPositionSignal = armAbsoluteEncoder.getPosition();

        //set update frequency for all status signals to 50Hz,
        BaseStatusSignal.setUpdateFrequencyForAll(50.0,
            rollerLeaderPositionRotationsSignal,
            rollerLeaderSpeedRotationsPerSecondSignal,
            rollerLeaderAppliedVolts,
            rollerLeaderCurrentAmps,
            armPositionRotationsSignal,
            armSpeedRotationsPerSecondSignal,
            armAppliedVolts,
            armCurrentAmps,
            armAbsoluteEncoderPositionSignal);

        BaseStatusSignal.setUpdateFrequencyForAll(4.0, rollerFollowerAppliedVoltageSignal);

        ParentDevice.optimizeBusUtilizationForAll(rollerLeaderMotor);
        ParentDevice.optimizeBusUtilizationForAll(armMotor);
        ParentDevice.optimizeBusUtilizationForAll(rollerFollowerMotor);


        rollerFollowerMotor.setControl(new Follower(RollerMotorConstants.rollerLeaderCanId, MotorAlignmentValue.Opposed));

        //end of constructor
    }

    @Override

    // STUDENT NOTE: Copy current device feedback into the shared sensor-reading structure.
    public void updateInputs(IntakeIOInputs measuredInputs) {
        measuredInputs.rollerLeaderMotorConnected = BaseStatusSignal.refreshAll(
            rollerLeaderPositionRotationsSignal,
            rollerLeaderSpeedRotationsPerSecondSignal,
            rollerLeaderAppliedVolts,
            rollerLeaderCurrentAmps
        ).isOK();

        measuredInputs.armMotorConnected = BaseStatusSignal.refreshAll(
            armPositionRotationsSignal,
            armSpeedRotationsPerSecondSignal,
            armAppliedVolts,
            armCurrentAmps
        ).isOK();
        measuredInputs.rollerFollowerMotorConnected = BaseStatusSignal.refreshAll(rollerFollowerAppliedVoltageSignal).isOK();

        //update the logged inputs with the latest values from the status signals
        measuredInputs.rollerLeaderPositionDegrees =
            rollerLeaderPositionRotationsSignal.getValueAsDouble() * 360.0;
        measuredInputs.rollerLeaderVelocityDegreesPerSecond =
            rollerLeaderSpeedRotationsPerSecondSignal.getValueAsDouble() * 360.0;
        measuredInputs.rollerLeaderAppliedVolts = rollerLeaderAppliedVolts.getValueAsDouble();
        measuredInputs.rollerLeaderCurrentAmps = rollerLeaderCurrentAmps.getValueAsDouble();
        measuredInputs.armPositionDegrees =
            armPositionRotationsSignal.getValueAsDouble() * 360.0;
        measuredInputs.armVelocityDegreesPerSecond =
            armSpeedRotationsPerSecondSignal.getValueAsDouble() * 360.0;
        measuredInputs.armAppliedVolts = armAppliedVolts.getValueAsDouble();
        measuredInputs.armCurrentAmps = armCurrentAmps.getValueAsDouble();
        measuredInputs.armAbsoluteEncoderPositionRotations = armAbsoluteEncoderPositionSignal.getValueAsDouble(); 

    }

    // READ THIS: Send a direct voltage request to the intake roller motor controller.
    @Override
    public void setRollerVoltage(Voltage volts) {
        rollerLeaderMotor.setControl(rollerVoltageRequest.withOutput(volts));
    }

    @Override

    // STUDENT NOTE: Ask the roller controller to maintain a requested rotation speed.
    public void setRollerSpeed(AngularVelocity velocity) {
        rollerLeaderMotor.setControl(rollerVelocityRequest.withVelocity(velocity));
    }

    @Override

    // STUDENT NOTE: Apply a raw voltage, primarily for motor characterization.
    public void setArmVoltage(Voltage volts) {
        armMotor.setControl(rollerVoltageRequest.withOutput(volts));
    }

    @Override

    // STUDENT NOTE: Ask the arm controller to move toward a desired angle.
    public void setArmTargetAngle(Angle angle) {
        armMotor.setControl(armMotionMagicRequest.withPosition(angle).withSlot(0));
    }

    // READ THIS: Move the arm using the alternative configured feedback gains.
    @Override
    public void setArmTargetAngleWithAlternateGains(Angle angle) {
        armMotor.setControl(armPositionRequest.withPosition(angle).withSlot(1));
    }

    // READ THIS: Stop the intake roller motors.
    @Override
    public void stopRollerMotors() {
        rollerLeaderMotor.setControl(neutralMotorRequest);
    }

    // READ THIS: Stop the intake pivot-arm motor.
    @Override
    public void stopArmMotor() {
        armMotor.setControl(neutralMotorRequest);
    }

    // READ THIS: Adjust the arm sensor position reading; this affects calibration and must be checked on hardware.
    @Override
    public void adjustArmSensorPositionDegrees(double degrees) {
        var pos = armPositionRotationsSignal.getValueAsDouble() * 360.0;
        armMotor.setPosition((pos + degrees) / 360.0);
    }

}

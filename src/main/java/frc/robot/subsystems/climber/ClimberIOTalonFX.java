// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;


import static frc.robot.util.PhoenixUtil.tryUntilOk;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.climber.ClimberConstants.ClimberEncoderConstants;
import frc.robot.subsystems.climber.ClimberConstants.ClimberMotorConstants;

/** Add your docs here. */
public class ClimberIOTalonFX implements ClimberIO{

    // STUDENT MAP: REAL CLIMBER MOTOR AND ENCODER
    // PURPOSE: Reads the climber position sensor and commands the climber motor through its TalonFX.
    // LOOK FOR: A measured encoder angle is different from a requested climber target.


    //instantiate climber motor
    private final TalonFX climberDriveMotor;

    //instantiate climber encoder
    private final CANcoder climberAbsoluteEncoder;

    //climber motor status signals
    //use rotations because getPosition returns rotations
    StatusSignal<Angle> climberMotorPositionRot;
    StatusSignal<AngularVelocity> climberMotorVelocityRadPerSec;
    StatusSignal<Voltage> climberMotorAppliedVolts;
    StatusSignal<Current> climberMotorCurrentAmps;

    //climber encoder status signals
    StatusSignal<Angle> climberEncoderAbsolutePositionRot;

    //create a new request to reuse for setting voltages and using commands
    private final VoltageOut climberVoltageRequest = new VoltageOut(0);
    private final NeutralOut neutralMotorRequest = new NeutralOut();


    // READ THIS: Configure the real climber controller and its sensor inputs.
    public ClimberIOTalonFX() {

      //structure for constructor is as follows:
      //1. config
      //2. create motor objects
      //3. apply config to motors, retrying up to 5 times with 0.25s delay
      //4. create status signals
      //5. set update frequency for all status signals
      //6. optimize bus utilization
       TalonFXConfiguration climberConfig = new TalonFXConfiguration()
            .withMotorOutput(ClimberMotorConstants.climberMotorOutputConfigs)
            .withCurrentLimits(ClimberMotorConstants.climberMotorCurrentLimits)
            .withSlot0(ClimberMotorConstants.climberMotorPidGains)
            .withFeedback(new FeedbackConfigs()
                .withFeedbackRemoteSensorID(ClimberEncoderConstants.climberEncoderCanId)
                .withSensorToMechanismRatio(ClimberMotorConstants.encoderToMechanismRatio)
                .withRotorToSensorRatio(ClimberMotorConstants.rotorToEncoderRatio)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder));
        climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = ClimberEncoderConstants.absoluteSensorDiscontinuityPoint;
        canCoderConfig.MagnetSensor.SensorDirection = ClimberEncoderConstants.sensorDirection; 
        canCoderConfig.MagnetSensor.MagnetOffset = ClimberEncoderConstants.magnetOffset;


        //create motor object and apply config
        climberDriveMotor = new TalonFX(ClimberMotorConstants.climberMotorCanId);
        tryUntilOk(5, () -> climberDriveMotor.getConfigurator().apply(climberConfig, 0.25));

        //create encoder object and apply config
        climberAbsoluteEncoder = new CANcoder(ClimberEncoderConstants.climberEncoderCanId);
        tryUntilOk(5, () -> climberAbsoluteEncoder.getConfigurator().apply(canCoderConfig, 0.25));

        //create status signals
        climberMotorPositionRot = climberDriveMotor.getPosition();
        climberMotorVelocityRadPerSec = climberDriveMotor.getVelocity();
        climberMotorAppliedVolts = climberDriveMotor.getMotorVoltage();
        climberMotorCurrentAmps = climberDriveMotor.getSupplyCurrent();
        climberEncoderAbsolutePositionRot = climberAbsoluteEncoder.getAbsolutePosition();

        //set update frequency for all status signals
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, 
            climberMotorPositionRot,
            climberMotorVelocityRadPerSec,
            climberMotorAppliedVolts,
            climberMotorCurrentAmps,
            climberEncoderAbsolutePositionRot);

            //optimize bus utilization
        climberDriveMotor.optimizeBusUtilization();
        climberAbsoluteEncoder.optimizeBusUtilization();

        //end of constructor
    }

    // READ THIS: Read available sensor signals into the measured-inputs object.
    @Override
    public void updateInputs(ClimberIOInputs measuredInputs) {

        //refresh the status signals to get the latest data from the motor
        measuredInputs.climberMotorConnected = BaseStatusSignal.refreshAll(
            climberMotorPositionRot,
            climberMotorVelocityRadPerSec,
            climberMotorAppliedVolts,
            climberMotorCurrentAmps
        ).isOK();

        //refresh climber encoder status signals to get the latest data from the encoder
        measuredInputs.climberEncoderConnected = BaseStatusSignal.refreshAll(
            climberEncoderAbsolutePositionRot
        ).isOK();
        measuredInputs.climberEncoderAbsolutePositionDegrees = climberEncoderAbsolutePositionRot.getValueAsDouble() * 360.0;

        //update the logged inputs with the latest values from the status signals
        measuredInputs.climberMotorPositionDegrees = climberMotorPositionRot.getValueAsDouble() * 360.0;
        measuredInputs.climberMotorVelocityDegreesPerSec = climberMotorVelocityRadPerSec.getValueAsDouble() * 360.0;
        measuredInputs.climberMotorAppliedVolts = climberMotorAppliedVolts.getValueAsDouble();
        measuredInputs.climberMotorCurrentAmps = climberMotorCurrentAmps.getValueAsDouble();
    }

    // READ THIS: Send the requested climber voltage to its motor.
    @Override
    public void setClimberMotorVoltage(double volts) {
        climberDriveMotor.setControl(climberVoltageRequest.withOutput(volts));
    }

    // READ THIS: Stop climber output through the hardware interface.
    @Override
    public void stopClimber() {
        climberDriveMotor.setControl(neutralMotorRequest);
    }


}

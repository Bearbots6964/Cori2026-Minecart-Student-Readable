// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;


import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.climber.ClimberConstants.ClimberMotorConstants;

// FILE GUIDE: Climber subsystem: records requested climb direction and sends voltage requests through the hardware interface.
public class Climber extends SubsystemBase {

    // STUDENT MAP: CLIMBER COMMANDS AND SAFETY STOPS
    // PURPOSE: Requests climbing up or down and stops at the measured target or a timeout.
    // LOOK FOR: This class is currently commented out in RobotContainer, so the actual robot does not instantiate it.
    private final ClimberIO climberHardware;
    private final ClimberIOInputsAutoLogged climberSensorReadings = new ClimberIOInputsAutoLogged();

    //Alert for climber disconnection
    private final Alert climberMotorDisconnectedAlert;
    private final Alert climberEncoderDisconnectedAlert;

    // Each Trigger evaluates a sensor-based yes/no condition; it does not move the climber.
    //trigger that is true when the climber has fully climbed
    public final Trigger isClimberAtTop = new Trigger(
        () -> Math.abs(climberSensorReadings.climberEncoderAbsolutePositionDegrees - 
        ClimberMotorConstants.climbedPositionDegrees.in(Degrees)) 
        <= ClimberMotorConstants.climberPositionTolerance.in(Degrees));

    //trigger that is true when the climber has fully descended
    public final Trigger isClimberAtBottom = new Trigger(
        () -> Math.abs(climberSensorReadings.climberEncoderAbsolutePositionDegrees - 
        ClimberMotorConstants.loweredPositionDegrees.in(Degrees)) 
        <= ClimberMotorConstants.climberPositionTolerance.in(Degrees));

    public final Trigger isClimberMotorDisconnected = new Trigger(
        () -> !climberSensorReadings.climberMotorConnected);

    public final Trigger isClimberEncoderDisconnected = new Trigger(
        () -> !climberSensorReadings.climberEncoderConnected);

  /** Creates a new climber. */
  public Climber(ClimberIO climberHardware) {
    this.climberHardware = climberHardware;
    climberMotorDisconnectedAlert =
       new Alert("Climber motor disconnected.", Alert.AlertType.kError);

         //critical alert for motor disconnection
    climberEncoderDisconnectedAlert = 
       new Alert("Climber encoder disconnected.", Alert.AlertType.kError);

         //critical alert for encoder disconnection
      }

  @Override

  // STUDENT NOTE: Read climb motor and encoder feedback and show connection warnings.
  public void periodic() {

    // This method will be called once per scheduler run
    climberHardware.updateInputs(climberSensorReadings);
    Logger.processInputs("Climber", climberSensorReadings);

    //check for climber motor disconnection and display if true
    climberMotorDisconnectedAlert.set(!climberSensorReadings.climberMotorConnected);

    //check for climber encoder disconnection and display if true
    climberEncoderDisconnectedAlert.set(!climberSensorReadings.climberEncoderConnected);
  }

  // STUDENT NOTE: Choose an upward, downward, or stop command.
  public Command setGoal(ClimberGoal goal){
    return switch(goal){
      case CLIMB -> climb();
      case DESCEND -> descend();
      case IDLE -> Commands.runOnce(this::stopClimber, this);
    };
  }

  //command to climb
  // STUDENT NOTE: Run the motor upward until the top position or timeout.
  public Command climb(){
    return runEnd(

      //climb at 6 volts when this command is scheduled
      () -> climberHardware.setClimberMotorVoltage(ClimberMotorConstants.climberCommandVoltage),

      //stop the climber motor when the command ends
      () -> climberHardware.setClimberMotorVoltage(0.0)

    //use absolute position from CANcoder
    ).until(isClimberAtTop)

     //safety timeout in case encoder fails or mechanism stalls
     .withTimeout(ClimberMotorConstants.climberMotionTimeoutSeconds);
  }

  //command to descend
  // STUDENT NOTE: Run the motor downward until the bottom position or timeout.
  public Command descend(){
    return runEnd(

      //descend at 6 volts when this command is scheduled
      () -> climberHardware.setClimberMotorVoltage(-ClimberMotorConstants.climberCommandVoltage),

      //stop the climber motor when the command ends
      () -> climberHardware.setClimberMotorVoltage(0.0)

    //use absolute position from CANcoder
    ).until(isClimberAtBottom)

     //safety timeout in case encoder fails or mechanism stalls
     .withTimeout(ClimberMotorConstants.climberMotionTimeoutSeconds);
  }

  //stop the climber motor
  public void stopClimber(){climberHardware.stopClimber();}


  public enum ClimberGoal{
    CLIMB,
    DESCEND,
    IDLE
  }


}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.intake.IntakeConstants.IntakeArmPosition;
import frc.robot.subsystems.intake.IntakeConstants.RollerAction;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Identifiable;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Coordinates two mechanisms: (1) a roller pair that moves fuel, and
 * (2) a pivoting arm that extends/retracts the rollers.
 *
 * Three different concepts appear below:
 *  - systemMode: whole-system purpose (STOWING, COLLECTING, TILTING, IDLING, EJECTING).
 *  - requestedRollerAction: what the ROLLERS are being asked to do.
 *  - targetArmPosition: the requested ARM POSITION, not the actual position.
 *
 * A request describes what the robot should do, not what its sensors
 * currently measure. AdvantageKit helps us record both kinds of information.
 *
 * JAVA NOTE: "extends SubsystemBase" means Intake inherits WPILib subsystem
 * behavior, including the periodic() method it can override below.
 * "implements Identifiable" means Intake agrees to meet the requirements
 * of the BearBots Identifiable interface.
 */
public class Intake extends SubsystemBase implements Identifiable {
    // STUDENT MAP: INTAKE SYSTEM DECISION-MAKING
    // PURPOSE: Keeps separate the whole-system mode, requested roller action, requested arm position, and measured conditions.
    // LOOK FOR: Read setSystemMode first, then periodic: the former sets requests; the latter sends commands to hardware.

    // This field holds a reference to hardware implementing IntakeIO.
    // "final" means we cannot later point this field to a different object.
    private final IntakeIO intakeHardware;

    // AdvantageKit generates IntakeIOInputsAutoLogged from @AutoLog
    // in IntakeIO. It holds the latest measured sensor values.
    private final IntakeIOInputsAutoLogged intakeSensorReadings = new IntakeIOInputsAutoLogged();

    // Alert for disconnected roller motor electronics.
    private final Alert rollerLeaderMotorDisconnectedAlert;

    // Alert for disconnected pivot-arm motor electronics.
    private final Alert armMotorDisconnectedAlert;

    // A Trigger answers a yes/no question based on measured encoder values.
    // It is not a command telling the arm to move.
    // @AutoLogOutput is an ANNOTATION, not a green comment. It marks a field
    // for AdvantageKit's automatic logging. The @ symbol is part of Java syntax.
    @AutoLogOutput
    public final Trigger isArmDeployed = new Trigger(
        // True only when the measured arm encoder is close to DEPLOYED_ANGLE.
        () -> Math.abs(intakeSensorReadings.armPositionDegrees
            - IntakeConstants.DEPLOYED_ANGLE.in(Degrees))
            <= IntakeConstants.ARM_POSITION_TOLERANCE.in(Degrees));

    @AutoLogOutput
    public final Trigger isArmRetracted = new Trigger(
        // True only when the measured arm encoder is close to RETRACTED_ANGLE.
        () -> Math.abs(intakeSensorReadings.armPositionDegrees
            - IntakeConstants.RETRACTED_ANGLE.in(Degrees))
            <= IntakeConstants.ARM_POSITION_TOLERANCE.in(Degrees));

    public final Trigger isArmMotorDisconnected = new Trigger(
        () -> !intakeSensorReadings.armMotorConnected);

    public final Trigger isRollerLeaderMotorDisconnected = new Trigger(
        () -> !intakeSensorReadings.rollerLeaderMotorConnected);

    @AutoLogOutput
    private IntakeSystemMode systemMode = IntakeSystemMode.IDLING; // overall system request

    @AutoLogOutput
    private IntakeArmPosition targetArmPosition = IntakeArmPosition.RETRACTED; // requested arm target

    @AutoLogOutput
    private RollerAction requestedRollerAction = RollerAction.STOPPED; // requested roller action

    // The actual command can temporarily override the roller request.
    // A requested action is not the same as a measured roller velocity.
    @AutoLogOutput
    private RollerAction commandedRollerAction = RollerAction.STOPPED;

    private final SysIdRoutine armIdentificationRoutine;

    /**
     * Creates a new Intake.
     */
    // A CONSTRUCTOR runs when another part of the program creates an Intake.
    // The caller passes an IntakeIO hardware object to the parameter "intakeIO".
    // Save its reference and create warnings for disconnected motors.
    public Intake(IntakeIO intakeIO) {
        this.intakeHardware = intakeIO;
        rollerLeaderMotorDisconnectedAlert =
            new Alert("Intake motor disconnected.", Alert.AlertType.kError);
        // Make another error alert for the pivot-arm motor.
        armMotorDisconnectedAlert =
            new Alert("Deploy motor disconnected.", Alert.AlertType.kError);
        // These alerts are updated later in periodic() using sensor feedback.

        // SysId tests help characterize the arm motor; this does not run
        // during ordinary intake control unless a SysId command is scheduled.
        armIdentificationRoutine = new SysIdRoutine(
            new Config(Volts.per(Second).of(0.5), Volts.of(3), null,
                (state) -> SignalLogger.writeString("state", state.toString())),
            new Mechanism(intakeHardware::setArmVoltage, null, this)
        );

    }

    // @Override tells Java that our periodic() replaces the inherited method
    // of the same name. WPILib calls this method on each scheduler cycle.
    @Override
    // STUDENT NOTE: Each cycle: read sensors, update alerts, and command motors.
    public void periodic() {
        // This method will be called once per scheduler run, every 20ms by default
        // Refresh measured sensor feedback, then record it for diagnostics.
        intakeHardware.updateInputs(intakeSensorReadings);
        Logger.processInputs("Intake", intakeSensorReadings);

        // Warn the driver if either subsystem controller is disconnected.
        rollerLeaderMotorDisconnectedAlert.set(!intakeSensorReadings.rollerLeaderMotorConnected);
        armMotorDisconnectedAlert.set(!intakeSensorReadings.armMotorConnected);

        // While the arm is requested to DEPLOY but has not yet reached that
        // angle, temporarily reverse the rollers slowly.
        // A measured arm position can therefore override the roller request.
        commandedRollerAction = requestedRollerAction;
        if (targetArmPosition == IntakeArmPosition.DEPLOYED && !isArmDeployed.getAsBoolean()) {
            commandedRollerAction = RollerAction.REVERSING_SLOWLY;
        }

        // Look up the speed for the chosen RollerAction and the angle for the
        // target IntakeArmPosition, then pass those values to the hardware.
        intakeHardware.setRollerSpeed(RotationsPerSecond.of(
            IntakeConstants.rollerSpeedRpsByAction.get(commandedRollerAction)));
        intakeHardware.setArmTargetAngle(Degrees.of(
            IntakeConstants.armTargetDegreesByPosition.get(targetArmPosition)));

        // The next scheduler cycle will update measurements and send fresh commands.
    }

    /**
     * LEGACY METHOD — not called by any Java file in this project.
     * Returns a command that waits 1.25 seconds and then only records
     * systemMode = STOWING. It does NOT call setSystemMode(), so it does not
     * request the arm to retract or change the roller request.
     * Study it as an example of a possible mismatch between a variable
     * describing the mode and the commands actually sent to hardware.
     */
    public Command waitThenRecordStowingMode() {
        return Commands.waitSeconds(1.25)
            .andThen(runOnce(() -> systemMode = IntakeSystemMode.STOWING));
    }

    /**
     * LEGACY METHOD — not called by any Java file in this project.
     * While scheduled, it repeatedly changes only the systemMode variable.
     * It does NOT call setSystemMode(), so it does not update the arm target
     * or roller request. Compare this with setSystemModeCommand() below.
     */
    public Command continuouslyRecordCollectingMode() {
        return
            run(() -> systemMode = IntakeSystemMode.COLLECTING);
    }

    /** Create a command that applies a whole-intake operating mode when scheduled. */
    // runOnce(...) creates a WPILib Command; setSystemMode() runs later,
    // when that command is actually scheduled.
    public Command setSystemModeCommand(IntakeSystemMode requestedMode) {
        return runOnce(() -> setSystemMode(requestedMode));
    }

    /**
     * Convert a whole-system mode into separate roller and arm targets.
     * IDLING and EJECTING keep the arm target requested by the previous mode.
     * This method does not itself move a motor; periodic() issues the outputs.
     */
    // STUDENT NOTE: Translate one goal for the entire intake into a roller request and an arm target.
    public void setSystemMode(IntakeSystemMode requestedMode) {
        this.systemMode = requestedMode;
        switch (requestedMode) {
            case STOWING -> {
                requestedRollerAction = RollerAction.RUNNING_SLOWLY;
                targetArmPosition = IntakeArmPosition.RETRACTED;
            }
            case COLLECTING -> {
                requestedRollerAction = RollerAction.RUNNING;
                targetArmPosition = IntakeArmPosition.DEPLOYED;
            }
            case TILTING -> {
                requestedRollerAction = RollerAction.RUNNING_SLOWLY;
                targetArmPosition = IntakeArmPosition.TILTED;
            }
            case IDLING -> {
                requestedRollerAction = RollerAction.STOPPED;
            }
            case EJECTING -> {
                requestedRollerAction = RollerAction.REVERSING;
            }
        }
    }

    public static enum IntakeSystemMode {
        /**
         * Request the intake to move back inside the frame perimeter.
         */
        STOWING,
        /**
         * Request an extended arm and running rollers to collect fuel.
         */
        COLLECTING,
        /**
         * Tilt the intake back to assist with funneling balls into the indexer.
         */
        TILTING,
        /**
         * Stop requesting roller movement while keeping the arm target.
         */
        IDLING,
        /**
         * Reverse the intake and spit out balls.
         */
        EJECTING

    }

    // STUDENT NOTE: Return a motor test that slowly ramps output for characterization.
    public Command sysIdQuasistatic(SysIdRoutine.Direction dir) {
        return armIdentificationRoutine.quasistatic(dir);
    }

    // STUDENT NOTE: Return a motor test that applies a faster step in output.
    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return armIdentificationRoutine.dynamic(dir);

    }

    // Number of degrees adjusted by one manual POV nudge.
    // This changes the TalonFX sensor position, not the arm target map.
    private static final double ARM_ANGLE_ADJUSTMENT_DEGREES = 3.0;

    // STUDENT NOTE: Request a small increase in the arm motor sensor position; this is not a target-position change.
    public Command nudgeArmUp() {
        return Commands.runOnce(() -> intakeHardware.adjustArmSensorPositionDegrees(ARM_ANGLE_ADJUSTMENT_DEGREES));
    }

    // STUDENT NOTE: Request a small decrease in the arm motor sensor position; this is not a target-position change.
    public Command nudgeArmDown() {
        return Commands.runOnce(() -> intakeHardware.adjustArmSensorPositionDegrees(-ARM_ANGLE_ADJUSTMENT_DEGREES));
    }

// End of the Intake class. The last brace closes the class opened above.
}

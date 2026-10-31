// BEARBOTS JAVA PRACTICE — INDEXER.JAVA
// INSTRUCTOR ANSWER KEY: completed version of the student exercise.
// Solutions are on the SAME line numbers as the student blanks.
// Leave the supplied hardware setup and advanced WPILib examples unchanged.
// A mentor must review your completed file before it is built or deployed.

// TASK 1 — GIVE THIS CHAPTER ITS LOCATION IN OUR ROBOT LIBRARY.
// The Indexer book is on the Subsystems shelf. In VS Code Explorer, this file
// is under src/main/java/frc/robot/subsystems/indexer.
// Write the PACKAGE declaration below using dots instead of slashes.
// Hint: compare the first non-comment line of Intake.java.

package frc.robot.subsystems.indexer;


// TASK 2 — TELL JAVA WHICH OTHER CLASSES THIS FILE WILL REFER TO.
// Each line needs to be an import statement ending in a semicolon.
// The paths below are provided so you are practicing Java syntax, not guessing
// which outside company wrote a class. Java imports NAME things; they do not
// copy the library into this file.
//
// WPILib: edu.wpi.first.math.filter.Debouncer
//         edu.wpi.first.units.Units
//         edu.wpi.first.wpilibj.DriverStation
//         edu.wpi.first.wpilibj2.command.button.Trigger
//         edu.wpi.first.wpilibj.Alert
//         edu.wpi.first.wpilibj.Alert.AlertType
//         edu.wpi.first.wpilibj2.command.Command
//         edu.wpi.first.wpilibj2.command.SubsystemBase
// BearBots: frc.robot.util.LoggedTunableNumber
// Java:    java.util.function.DoubleSupplier
// Lombok:  lombok.Setter
// AdvantageKit: org.littletonrobotics.junction.AutoLogOutput
//               org.littletonrobotics.junction.Logger
// Hint: "import" is followed by one full name, then a semicolon.

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;


// TASK 3 — START THE CLASS.
// Declare a PUBLIC class named Indexer that EXTENDS SubsystemBase.
// Add an opening curly brace { and keep the closing brace at the end.
// Hint: the first part looks like the header of your Intake class.

public class Indexer extends SubsystemBase {

    // PROVIDED: these fields let the Indexer store hardware references,
    // read sensors, and display alerts. You do not have to write them today.

    private final IndexerIO indexerHardware;

    private final IndexerIOInputsAutoLogged indexerSensorReadings = new IndexerIOInputsAutoLogged();

    private final Alert indexerMotorDisconnectedAlert;

    private final Alert overcurrentAlert;

    private final Trigger isIndexerOverCurrent;

    private final Debouncer overCurrentDebouncer;

    // TASK 4 — DECLARE THE REQUESTED OPERATING MODE.
    // @Setter creates a setter method; @AutoLogOutput allows AdvantageKit logging.
    // Write a PRIVATE field named requestedIndexerMode of TYPE IndexerOperatingMode.
    // Set its initial value to IndexerOperatingMode.IDLING.
    // Hint: look at requestedRollerAction in Intake.java.
    


    private final IndexerVisualizer indexerVisualizer;

    private final DoubleSupplier requestedIndexerSpeedRpsSupplier;

    private final LoggedTunableNumber tuningIndexerSpeedRps = new LoggedTunableNumber(
        "Indexer/Tuning/SpeedRPS", 0.0);


    // PROVIDED CONSTRUCTOR: store the hardware objects passed in by the caller.
    // Also set up motor warnings, an overcurrent trigger, and a visualizer.
    // You will study this section later, after the simpler motor-control tasks.
    public Indexer(IndexerIO indexerHardware, DoubleSupplier requestedIndexerSpeedRpsSupplier) {
        this.indexerHardware = indexerHardware;
        this.requestedIndexerSpeedRpsSupplier = requestedIndexerSpeedRpsSupplier;

        indexerMotorDisconnectedAlert = new Alert("Indexer motor disconnected", AlertType.kError);
        overCurrentDebouncer = new Debouncer(0.2);
        isIndexerOverCurrent = new Trigger(
            () -> overCurrentDebouncer.calculate(indexerSensorReadings.indexerCurrentAmps > 90.0));

//        overcurrentTrigger.onTrue(runIndexer());
        overcurrentAlert = new Alert("Indexer hitting current limit!", AlertType.kWarning);
        indexerVisualizer = new IndexerVisualizer();
    }


    // WPILib calls periodic() about every 20 milliseconds by default.
    // PROVIDED: update the measured sensor readings, log them, and show warnings.
    @Override
    public void periodic() {
        indexerHardware.updateInputs(indexerSensorReadings);

        Logger.processInputs("Indexer", indexerSensorReadings);

        indexerMotorDisconnectedAlert.set(!indexerSensorReadings.indexerLeaderMotorConnected);

        indexerVisualizer.update(Units.RotationsPerSecond.of(indexerSensorReadings.indexerSpeedRps));

        // PROVIDED: when the robot is disabled, request IDLING and stop the motor.
        if (DriverStation.isDisabled()) {
            requestedIndexerMode = IndexerOperatingMode.IDLING;
            indexerHardware.stop();
        }

        // PROVIDED: get the latest speed requested by the shooter and log it.
        // A DoubleSupplier is a way for the shooter to provide a changing number.
        var requestedSpeedRps = requestedIndexerSpeedRpsSupplier.getAsDouble();
        Logger.recordOutput("Indexer/TargetSpeed", requestedSpeedRps);

        // TASK 5 — THIS IS YOUR MAIN PROGRAMMING CHALLENGE.
        // Use if / else if statements to choose an action based on requestedIndexerMode.
        // For IDLING, call indexerHardware.stop().
        // For FEEDING, call indexerHardware.setIndexerSpeedRps(requestedSpeedRps).
        // For REVERSING, call indexerHardware.setIndexerDutyCycle(...)
        //   using the NEGATIVE of IndexerConstants.MOTOR_SPEED_PERCENTAGE.
        // Hint: enum values are compared using == (not =).
        // Write your conditions and motor instructions below.
        

    }


    // TASK 6 — CREATE A COMMAND THAT SETS THE REQUESTED MODE ONCE.
    // You have been given the method header. Write its return statement.
    // Use runOnce(...) to assign requestedIndexerMode to the parameter
    // requestedIndexerMode. Because the names match, use "this." on the field.
    // Hint: compare setSystemModeCommand(...) in Intake.java.
    public Command setOperatingModeCommand(IndexerOperatingMode requestedIndexerMode) {

        return runOnce(() -> this.requestedIndexerMode = requestedIndexerMode);

    }


    // PROVIDED ADVANCED EXAMPLE — READ, BUT DO NOT REWRITE THIS YET.
    // As long as the command is scheduled, request FEEDING. When it ends,
    // change the request to IDLING. We'll revisit runEnd() and finallyDo().
   


    // TASK 7 — CREATE A COMMAND TO STOP THE INDEXER.
    // Write ONE return statement using runOnce(...).
    // It needs to do TWO things: call indexerHardware.stop() and assign
    // IndexerOperatingMode.IDLING to requestedIndexerMode.
    // Hint: use braces { ... } inside the lambda to run two statements.
    


    // TASK 8 — DEFINE THE THREE POSSIBLE OPERATING MODES.
    // An enum is a limited list of named choices. Write the three values
    // FEEDING, IDLING, REVERSING, separated with commas.
    // Hint: compare IntakeSystemMode in Intake.java.
   

} // End of the Indexer class.

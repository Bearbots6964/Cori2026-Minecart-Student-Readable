# Understanding MineCart — Start here (first Java class)

**Audience:** A ninth grader who is comfortable thinking carefully but has not written a Java program yet. You do **not** need to understand every library call to follow how this robot makes a decision.

## The five-minute robot map

The 2026 BearBots robot has several systems. `RobotContainer.java` creates their objects and links gamepad controls to commands. `Intake.java` decides what the rollers and pivot arm should do. `Indexer.java` moves fuel toward the shooter. `Shooter.java` decides the desired flywheel speed and hood angle. `Drive.java` controls the four swerve modules. `Vision.java` checks camera estimates. `Superstructure.java` coordinates multiple systems for scoring and passing.

**The most important distinction:** a **request** is what software wants; a **commanded output** is what software sends to a motor controller; a **measurement** is what a sensor reports afterward. These can be different at the same instant.

## Read these files in this order

1. `src/main/java/frc/robot/RobotContainer.java` → locate `configureButtonBindings()`; do **not** try to understand the whole file.
2. `src/main/java/frc/robot/subsystems/intake/Intake.java` → find `IntakeSystemMode`, `setSystemMode()`, and `periodic()`.
3. `src/main/java/frc/robot/subsystems/intake/IntakeConstants.java` → compare `COLLECTING`, `RUNNING`, and `DEPLOYED`; these represent three different responsibilities.
4. `src/main/java/frc/robot/subsystems/intake/IntakeIO.java` → read what sensor readings arrive and what motor requests can be sent.
5. `src/main/java/frc/robot/subsystems/intake/IntakeIOTalonFX.java` → identify real motor controllers and the code that sends requests to them. Treat vendor-library internals as a black box for now.
6. `Indexer.java` and `Superstructure.java` → see how the robot coordinates more than one system.
7. `Module.java`, `Drive.java`, and `Vision.java` → understand measured position, desired motion, and camera corrections.
8. `Shooter.java` and `ShooterCalculator.java` → only after you know classes, methods, commands, and units. Some calculations require physics and trigonometry.

`generated/TunerConstants.java` was created using CTRE tools. Do not begin there; it contains important calibration information but is not designed as a first Java lesson. Some `util` files are also specialized algorithms, not beginner exercises.

## Follow one button press

Find this actual statement in `RobotContainer.java`:

```java
collectFuelButtonPressed.onTrue(
    intakeSystem.setSystemModeCommand(IntakeSystemMode.COLLECTING));
```

Read it as: **when the operator presses the collect button, schedule a command that requests the intake's COLLECTING mode.** It does not directly apply motor voltage. Now find the `COLLECTING` branch in `Intake.setSystemMode()`:

```java
case COLLECTING -> {
    requestedRollerAction = RollerAction.RUNNING;
    targetArmPosition = IntakeArmPosition.DEPLOYED;
}
```

Read it as: **ask the rollers to run and ask the arm to move to its deployed target.** These are requests, not measurements. In `Intake.periodic()`, the code reads sensors, may temporarily override roller action while the arm is moving, and sends the resulting motor requests through `intakeHardware`.

In `IntakeIOTalonFX.java`, the `TalonFX` objects communicate with the real controllers. `IntakeIO.java` describes the boundary between a request and the hardware that carries it out.

**Check your understanding:** What would happen if `targetArmPosition` were `DEPLOYED` but `isArmDeployed` were false? Look at `commandedRollerAction` in `periodic()`. The rollers are temporarily commanded to reverse slowly. That is an existing feature of the original robot code.

## Small Java dictionary (no analogies needed)

| Code word or symbol | Meaning here |
|---|---|
| `class` | A definition grouping related variables and methods. |
| `new Intake(...)` | Creates one Intake object. |
| `private` | Only code in this class can directly access that member. |
| `public` | Other classes may access the member. |
| `final` | A variable cannot be reassigned to another object after initialization; the object's internal values may still change. |
| `enum` | A type with a fixed set of named choices, such as `COLLECTING` or `IDLING`. |
| `void` | The method returns no value. |
| `boolean` | A value that is true or false. |
| `return` | Sends a result back to the code that called this method. |
| `.` | Accesses a method or member of a class/object, as in `intakeHardware.setRollerSpeed(...)`. |
| `@Override` | Marks a method that implements or replaces a method from a parent class or interface. |
| `periodic()` | WPILib calls this repeatedly, normally about every 20 milliseconds. |
| `Command` | WPILib's description of an action that runs when scheduled. Merely creating one does not necessarily execute it. |
| `Trigger` | A yes/no condition, such as whether the intake arm has reached its target. |
| `Supplier<T>` | A function the code can call to ask for the current value of type `T`. |
| `() -> ...` | A lambda: a small function passed to other code so it can run later. Learn to recognize it before trying to write one. |
| `object::method` | A method reference: passes a method for some other code to call later. |
| `runOnce(...)` | Creates a command that performs an action once when scheduled. |
| `whileTrue(...)` | Schedules a command while the associated Trigger is true. |
| `.in(Degrees)` | Gets the numeric value of an angle expressed in degrees. |
| `set...(...)` | Usually requests or changes a target; it does not prove the mechanism reached the target. |
| `is...` | Usually asks a true/false question about current state or feedback. |

## Names you should be able to read as English

- `IntakeSystemMode.COLLECTING`: the whole intake's purpose.
- `requestedRollerAction = RollerAction.RUNNING`: the roller motion being requested.
- `targetArmPosition = IntakeArmPosition.DEPLOYED`: the desired position, not the measured position.
- `isArmDeployed`: a condition based on the actual measured encoder position.
- `commandedRollerAction`: what software chooses to send to the rollers after checking conditions.
- `intakeSensorReadings.armPositionDegrees`: a measurement arriving from the hardware.

**Actions** often use verbs: `setArmTargetAngle`, `stopFlywheel`, `refreshAllianceHubTarget`. **Conditions** commonly use `is`, `has`, or an adjective: `isArmDeployed`, `isClimberAtTop`, `STOWING` versus `RETRACTED`.

## Important boundaries of this learning edition

Some calculations, especially camera transformations, PathPlanner internals, CTRE controller settings, motor characterization, and projectile physics, will *not* read like plain English to a first-day programmer. That does not mean you failed to learn Java; these sections require additional robotics and math lessons. The guide gives you a route through the understandable, team-authored control logic first.

This edition prioritizes **readability without changing robot behavior**. See `REVIEW_BEFORE_DEPLOYMENT.md` for questionable behaviors deliberately left unchanged; a naming/comment cleanup must not silently become an untested motor-control change.

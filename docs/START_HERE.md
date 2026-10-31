# Read MineCart from a button press to a motor

The quickest way to learn the project is to follow a single intake command through four layers. File links below are paths relative to the `src/main/java/frc/robot/` folder.

1. Open **`RobotContainer.java`**. Look for `collectFuelButtonPressed`, which reads the operator's B button.
2. Find `collectFuelButtonPressed.onTrue(intakeSystem.setSystemModeCommand(IntakeSystemMode.COLLECTING))`. The button **schedules a command**; it does not directly set voltage.
3. Open **`subsystems/intake/Intake.java`**. `setSystemMode()` translates a whole-system mode into `requestedRollerAction = RUNNING` and `targetArmPosition = DEPLOYED`.
4. Still in **`Intake.java`**, look at `periodic()`. It reads `intakeSensorReadings`, calculates `commandedRollerAction`, and sends separate speed and arm angle requests to `intakeHardware`.
5. Open **`subsystems/intake/IntakeIO.java`**. This interface describes the measurements received from hardware and the commands the subsystem can send.
6. Open **`subsystems/intake/IntakeIOTalonFX.java`**. It translates those requests into CTRE TalonFX commands for the roller and pivot arm motors.
7. Open **`subsystems/intake/IntakeConstants.java`**. The enum values tell you which *action* or *target position* the code is referring to; the maps provide the unchanged competition speed and angle values.

## The central naming idea

**Collecting** describes the whole intake's purpose. **Running** describes the roller mechanism's desired action. **Deployed** describes the arm's target position. **isArmDeployed** asks whether the encoder reports the arm is at that position.

The physical distinction is also important: the intake is made from a **roller mechanism** (leader and follower motors) and an **arm mechanism** (motor and absolute-position CANcoder). Do not confuse an arm angle, roller speed, requested target, and actual sensor measurement.

## One detail that seems strange

While the arm is *moving toward* its deployed position, `periodic()` commands the rollers to reverse slowly. That behavior was already in the original code. It is exposed through the clearer variable `commandedRollerAction`, which can differ from `requestedRollerAction` during that motion.

`waitThenRecordStowingMode()` and `continuouslyRecordCollectingMode()` intentionally only update the high-level `systemMode` field, **not** the arm/roller targets. Their old names (`tilt` and `autoIntake`) obscured this. Treat any actual behavior correction as a separate, tested change.

## Where to go next

- `subsystems/indexer/Indexer.java` — read `requestedIndexerMode` as a *request* (feeding/idling/reversing), not measured speed.
- `subsystems/shooter/Shooter.java` — distinguish `operatingMode`, `requestedHoodAngle`, `currentFieldTarget`, and `requestedIndexerSpeedRps`.
- `subsystems/superstructure/Superstructure.java` — follow `requestedRobotMode` into commands for individual mechanisms.
- `subsystems/drive/Drive.java` and `subsystems/drive/Module.java` — follow desired chassis motion to four wheel targets, then compare with measured wheel/gyro positions.
- `subsystems/vision/Vision.java` — distinguish camera observations from accepted robot pose estimates.

The CAD images in `docs/cad` can help you connect the code to the physical robot. Those images alone do not establish electrical mappings or calibration.

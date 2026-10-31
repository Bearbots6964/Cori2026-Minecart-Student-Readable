# Questions to resolve before deploying to MineCart

These are **pre-existing or previously documented behavioral concerns**, not syntax errors found in this student-commenting pass. They were *not* changed because the requested task was to make code understandable without guessing at robot hardware behavior.

1. **Intake mode recording versus motor commands.** `Intake.waitThenRecordStowingMode()` and `continuouslyRecordCollectingMode()` change only `systemMode`. They do not call `setSystemMode()`; the arm and rollers will keep their previous requests. Check callers before converting them into actual motor commands.
2. **Operator B button and B + Right Bumper overlap.** `collectFuelButtonPressed` is B alone; `retractIntakeButtonPressed` includes B and Right Bumper. Pressing both can satisfy both Triggers. Verify command scheduling and expected precedence; do not assume the more specific combination wins.
3. **Shooter dashboard tuning button may not schedule its returned command.** In the `Shooter` constructor, `Commands.runOnce(() -> setOperatingModeCommand(ShooterOperatingMode.TUNING))` creates a Command inside another command's action rather than directly calling `setOperatingMode` or scheduling the newly created command. The dashboard button may not perform the intended mode switch. Confirm before changing it.
4. **Robot idle mode does not necessarily retract the intake.** `Superstructure.idleSubsystems()` actually requests `IntakeSystemMode.COLLECTING`. An earlier enum comment suggested idle meant stow; the comment was corrected, but the behavior was left intact.
5. **Shooter angle/position conversion TODOs.** Verify linkage geometry, units, sensor zero, and motor/hood ratios before modifying `hoodAngleToPosition` / `hoodPositionToAngle` and their related functions.
6. **Shooter and field constants marked TODO.** Some map points, position values, and simulation integration remain unfinished or require calibration data. Values were not guessed.
7. **Climber is present but not connected in RobotContainer.** The relevant instantiation lines are commented out; the climber subsystem is not part of the current real-robot construction path.
8. **AdvantageKit logs might use different automatically generated field names.** Renamed fields/types can change automatic log paths. Literal logger keys were not changed; verify AdvantageScope and dashboards that expect historic names.
9. **Unused fields/imports and commented experiments.** Some older or unfinished elements are still in the source. They do not necessarily prevent compilation but may require separate cleanup after confirming why they were retained.

**Recommended process:** Treat behavior fixes as separate Git commits. First verify compilation and basic functionality of this readability edition; only then correct one suspected bug at a time, with tests.

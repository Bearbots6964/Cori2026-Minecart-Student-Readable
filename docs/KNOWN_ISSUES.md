# Issues intentionally separated from the naming pass

The code in this archive is meant to behave like the uploaded MineCart code. These observations are **not** silently corrected, because doing so might affect robot behavior and could require hardware knowledge.

1. **Intake recording versus actuation.** `waitThenRecordStowingMode()` and `continuouslyRecordCollectingMode()` only assign `systemMode` in the preserved original logic. They do not call `setSystemMode()` to update arm and roller requests. The names now make this distinction explicit. Review their callers, schedule, and actual expected motor actions before fixing.
2. **Shooter angle conversion TODOs.** `Shooter.java` contains two TODO comments around hood angle/position conversion. The mechanical gear and encoder reference must be established before making corrections.
3. **Shooter calibration TODO.** `ShooterConstants.java` contains a reminder to fill a lookup map from test data. Verify any uncalibrated ranges on the robot.
4. **Fuel simulation integration TODO.** `RobotContainer.java` notes that simulated fuel movement is not fully linked to simulated intake behavior.
5. **Other unfinished constant values.** `Constants.java` includes TODO markers for measured values and bump-related settings. They were not guessed or filled in.
6. **Intake gear-ratio comment contradictions.** Old comments claimed ratios that disagreed with the actual numeric expressions. Comments were rewritten to emphasize the existing code values and flag the need to verify real gear stages; **no physical ratio was changed**.
7. **Unused fields and incomplete features.** Some members are not referenced by the Java code (for example, certain indexer overcurrent and simulation fields). They were left in place, because an unused field may be scaffolding for a future feature. Removing it would not improve the learning goal enough to justify functionality risk.
8. **AdvantageKit automatic telemetry.** Identifiers and enum values were renamed; their automatically derived logged names will therefore change. The literal logging keys remain the same. Check dashboards and log analysis tools that look up those dynamic field names.
9. **Subsystem naming versus vendor code.** `generated/TunerConstants.java`, PathPlanner resources, WPILib override signatures, CTRE and third-party APIs retain required contracts. A name that looks less like ordinary English may be an external API or conventional robotics term; don't rename it without testing and reference updates.

## A small behavior-verification example

In `Intake.periodic()`, if the arm target is `DEPLOYED` and the measured condition `isArmDeployed` is false, the commanded roller action remains `REVERSING_SLOWLY`. Otherwise `commandedRollerAction` equals `requestedRollerAction`. This is the same selection as before; a future bug fix should be its own commit, separate from this naming review.

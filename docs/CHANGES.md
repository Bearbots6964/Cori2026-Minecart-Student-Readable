# Summary of this refactoring pass

- Adopted the BearBots naming principle: distinguish **whole system intent**, **mechanism requests**, **device-level motor outputs**, **position targets**, and **measured sensor states**.
- Completed the intake's central names (`IntakeSystemMode`, `requestedRollerAction`, `commandedRollerAction`, `targetArmPosition`, `isArmDeployed`, etc.) and made the real hardware IO method names match the same physical structure.
- Applied descriptive terminology throughout RobotContainer, drivetrain, swerve modules, climber, indexer, shooter, superstructure, vision and utility code. Updated all referencing files, callers, method references, enum constants, and generated Lombok accessor names as needed.
- Named cameras according to the actual configured device literals: left/right/back.
- Added class-level student reading guides in all 58 checked-in Java files, with extra conceptual explanations in important control-flow areas.
- Removed 32 unused import statements.
- Preserved numeric settings, string literals and current robot output behavior; the intake's existing override is now visible as `commandedRollerAction`.
- Produced a full-source Java compilation result and independent literal audit. A native WPILib Gradle compile remains required on a WPILib-equipped computer, as documented in `VERIFICATION.md`.

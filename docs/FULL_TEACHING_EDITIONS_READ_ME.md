# Minecart full-project student-readable teaching edition — 2026-10-30

## What is included

- The complete 122-file baseline project from `Cori2026-Minecart-Student-Readable-Audited(1).zip`.
- `src/main/java/frc/robot/subsystems/intake/Intake.java` is the newer, annotated **worked example**.
- `src/main/java/frc/robot/subsystems/indexer/Indexer.java` is the **completed instructor answer**; the incomplete student worksheet is NOT in the source tree.
- The remaining 56 Java files received file/method explanation comments where useful, plus blank-line cleanup to keep comments immediately before the lines they describe. Where safe, end-of-line explanatory notes were moved above their code lines; wording was preserved.
- The CTRE-generated `TunerConstants.java` remains generated code, with only explanatory comments/spacing added. Do not hand-edit its calibration values.
- All baseline Gradle files, vendor deps, camera layouts, PathPlanner paths, and assets are preserved.

## How to read it as a beginner

1. Start with `docs/START_HERE.md`; follow one intake button press to an intake motor.
2. Open the completed `Intake.java` and study `setSystemMode()` and `periodic()`.
3. Next read the completed `Indexer.java`; trace `requestedIndexerMode` into motor commands in `periodic()`.
4. Look in `RobotContainer.java` to see which controller buttons schedule those commands.
5. Follow `*IO.java` from a subsystem to `*IOTalonFX.java` to see the hardware boundary.
6. Leave `generated/TunerConstants.java`, sensor math and fuel physics until after you understand the command path.

## Two kinds of files: worked example vs deployed logic

`Intake.java` is an annotated, functioning subsystem. `Indexer.java` includes task descriptions because it is the completed version of the student worksheet; **it contains its correct executable statements**. A separate PDF worksheet intentionally omits statements and must never replace this file in a robot project.

## What was checked

- `PASS`: all 58 Java files have the same non-comment Java lexer tokens as the supplied baseline. This preserves executable expressions, declarations, literal values, import statements, and their order.
- `PASS`: ZIP integrity and unchanged file inventory (122 original files + documentation).
- `PASS`: every non-Java baseline asset remains byte-for-byte identical.
- `BUILD ATTEMPT BLOCKED`: `./gradlew --offline --no-daemon compileJava` could not run: the Gradle 8.11 wrapper distribution was not cached, and fetching `services.gradle.org` failed.
- `NOT VERIFIED HERE`: native WPILib build, deployment, and physical robot behavior. Lexical preservation is not a replacement for a robot build and controlled motor test.

## Known issues were NOT silently fixed

Review `docs/KNOWN_ISSUES.md` and `docs/REVIEW_BEFORE_DEPLOYMENT.md` for existing behavior questions (intake mode-only helpers, shooter hood calibration, unconnected overcurrent alerts, etc.). The goal of this revision is readability, not changing robot control.

## Practical compile check

On a computer with the **WPILib 2026** project dependencies available, open the folder containing `build.gradle` and run **WPILib: Build Robot Code**. Only after a successful build should mentors check the robot safely, preserving your currently functioning version as a fallback.

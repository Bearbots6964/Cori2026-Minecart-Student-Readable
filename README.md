# Cori 2026 MineCart — Student-Readable Audit

**BearBots 6964 | Second-pass readability edition for beginning Java students**

This is a separate review copy of the 2026 robot software. Every one of the 58 checked-in Java source files has been reviewed. The team code uses more descriptive names to distinguish an **operator request**, a **mechanism target**, **motor output commands**, **physical electronics**, and **measured feedback**. Begin with **`docs/NINTH_GRADE_START_HERE.md`** for the ninth-grade walkthrough. See `docs/START_HERE.md` to follow the code in VS Code, `docs/NAME_CROSSWALK.md` for key before/after names, and `docs/VERIFICATION.md` for the exact build evidence and limits.

## Start here

1. Read `docs/NINTH_GRADE_START_HERE.md` before opening advanced vendor files.
2. Follow the operator B-button trace from `RobotContainer.java` through `Intake.java` to `IntakeIOTalonFX.java`.
3. Read `docs/FINAL_READABILITY_AUDIT.md` for what was checked and `docs/REVIEW_BEFORE_DEPLOYMENT.md` for potential existing bugs.

## Source structure

- `Robot.java` — WPILib lifecycle, including autonomous and teleoperated operation.
- `RobotContainer.java` — constructs subsystems, chooses real/simulation hardware, assigns operator controls, and registers autonomous commands.
- `subsystems/intake` — roller leader/follower motors, pivoting arm motor, and arm CANcoder.
- `subsystems/indexer` — moves fuel from the intake to the shooter.
- `subsystems/shooter` — hood, flywheels, target-selection logic, and shot calculations.
- `subsystems/drive` — four swerve modules, heading sensor, and robot pose estimation.
- `subsystems/vision` — camera-based AprilTag measurements and pose uncertainty.
- `subsystems/superstructure` — combines shooter, intake, and indexer into robot scoring modes.
- `subsystems/climber` — climber motor, absolute position feedback, and limits.
- `commands` — movement commands built from gamepad input.
- `util` — field zones, hub timing, logging, simulation, and math helpers.

## Important limits

**The full Java source passed a second independent Java 17 compilation check using dependencies from the original robot JAR.** Temporary compile-only stand-ins were needed for Lombok and AdvantageKit generated code. The normal 2026 WPILib **Gradle build could not run in this environment**: Gradle 8.11 must be downloaded, and network name resolution is unavailable. Therefore this is **not yet verified as a native Gradle build or physically tested on MineCart**.

No motor tuning values, sensor offsets, CAN IDs, WPILib scheduling decisions, PathPlanner names, or literal strings/numbers were deliberately changed. One intake control-flow expression was made more explicit so `commandedRollerAction` can be logged; it selects the same roller speed as the original. Some *automatically inferred AdvantageKit telemetry field names* will change because Java field names changed. Review logging consumers before treating this as deployable.

**Do not replace the working competition code before a full WPILib build, simulation where applicable, and staged robot tests.** The potential behavior bugs listed in `docs/KNOWN_ISSUES.md` are deliberately left unchanged.

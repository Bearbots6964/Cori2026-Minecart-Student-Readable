# Verification report — full English-naming pass

## Files and provenance

- Starting robot project: `Cori2026-Minecart(2).zip` (uploaded original, retained for comparison).
- Starting readability edition: `Cori2026-Minecart-Readable-Review.zip`, whose changes were checked against the original.
- Result: 58 hand-written Java files, Gradle/WPILib project resources, vendor dependency descriptions, and robot deploy resources. All 58 Java files reviewed; files already well named were annotated with a reading guide rather than pointlessly renamed.
- Generated `BuildConstants` / AdvantageKit output and historical build caches are deliberately **not** packaged. Do not commit generated Gradle output in GitHub.

## Checks performed here

**PASS — Whole-source Java 17 compilation check**

All 58 Java files compiled with `javac --release 17 -proc:none` and the original project's application JAR on the classpath for WPILib, CTRE, PathPlanner and other vendor classes. The check used temporary verification-only source substitutes for Lombok-generated accessors and AdvantageKit-generated `*InputsAutoLogged` classes, since their real Gradle annotation processors are unavailable. These temporary substitute files **are not included in the project ZIP**. All 68 Java compilation units in this independent check compiled successfully. This confirms syntax, types, and cross-file reference consistency in that setup, but it **does not substitute for the real build plugins**.

**PASS — Literal preservation audit**

Compared Java source code before and after the latest naming pass in all 58 source files, excluding comments. **Zero differences** in Java numeric literals, character literals, and string literals. Thus the supplied motor numbers, CAN IDs, angle limits, timing values, tuning parameters, PathPlanner named-command strings, dashboard literal keys, and other code literals were not modified by the naming work.

**REVIEWED — Motor-output selection**

In `Intake.periodic()`, the old `if`/`else` chose reverse-slow while deploying or the requested roller action otherwise. The new named `commandedRollerAction` contains that exact same choice, then sends the same speed from the same map. This is a transparent expression of the old algorithm, not a change to roller output. No physical robot was present for a behavioral test.

**CLEANUP — Unused imports**

32 verified unused import statements from the previous edition were removed. Source and compilation pass afterward. No executable logic was removed.

## Checks not possible here

**NOT VERIFIED — Native 2026 WPILib Gradle build**

Attempting `./gradlew --offline --no-daemon build` requested `gradle-8.11-bin.zip` from `services.gradle.org`. The Gradle distribution is not cached and this runtime cannot resolve the download host (`UnknownHostException`). Consequently, **do not describe this archive as fully Gradle-build-verified**. The Windows `gradlew.bat` and Gradle wrapper files are provided for building on the user's installed WPILib machine. The Unix `gradlew` line endings were normalized to LF for cross-platform compatibility.

**NOT VERIFIED — Real / simulation robot behavior**

No roboRIO, CAN bus, TalonFX, CANcoder, Driver Station, hardware-equipped test stand, or full PathPlanner/WPILib simulator run was available here. Validate on installed WPILib before deploying. The preserved behavior questions in `KNOWN_ISSUES.md` need separate testing.

## Required check on a WPILib 2026 computer

1. Extract the ZIP; open the **folder containing `build.gradle`**, rather than an enclosing archive or nested folder.
2. In **WPILib VS Code**, use **Ctrl+Shift+P → WPILib: Build Robot Code**. Wait for `BUILD SUCCESSFUL`.
3. Confirm there are no red Problems. Unused-field warnings/TODO notices are different from build failures. If VS Code reports only phantom generated-class errors despite successful build, try `Java: Clean Java Language Server Workspace` and reload.
4. Test the robot's real and simulated configurations, PathPlanner autonomous commands, arm motion, roller reversal during deployment, indexer feeding, flywheel/hood, heading/vision, and disabled/teleop transitions in a controlled environment. Keep wheels clear of the ground or hardware disabled as appropriate during the first checks.
5. Retain the working 2026 code and only replace it after these checks succeed.

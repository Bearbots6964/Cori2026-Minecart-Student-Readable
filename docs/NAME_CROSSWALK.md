# Before-and-after naming crosswalk

This review applies the team's naming philosophy across the robot: **whole-system intent → mechanism request → hardware command → measured feedback**. These examples are not a complete list; the source itself is the reference.

| Previous code name | New name | What it describes |
|---|---|---|
| `IntakeGoal goal` | `IntakeSystemMode systemMode` | Whole intake purpose (`STOWING`, `COLLECTING`, `TILTING`, `IDLING`, `EJECTING`) |
| `IntakeArmPosition deployMode` | `IntakeArmPosition targetArmPosition` | Desired arm position (`RETRACTED`, `DEPLOYED`, `TILTED`) |
| `RollerAction intakeMode` | `RollerAction requestedRollerAction` | Desired roller speed/direction |
| inline roller override | `RollerAction commandedRollerAction` | Roller command actually selected each scheduler cycle |
| `isDeployed` / `isRetracted` | `isArmDeployed` / `isArmRetracted` | Measured arm-position conditions, not actions |
| `setGoal()` / `setGoalCommand()` on Intake | `setSystemMode()` / `setSystemModeCommand()` | Apply a whole-system intake intent |
| `tilt()` on Intake | `waitThenRecordStowingMode()` | Old behavior: wait and update only `systemMode` |
| `autoIntake()` on Intake | `continuouslyRecordCollectingMode()` | Old behavior: update only `systemMode` while scheduled |
| `setIntakeMotorVelocity()` | `setRollerSpeed()` | Command roller mechanism angular velocity |
| `setDeployPosition()` | `setArmTargetAngle()` | Request pivot arm angle |
| `setDeployVoltage()` | `setArmVoltage()` | Command arm motor voltage |
| `intakeMotorConnected` | `rollerLeaderMotorConnected` | Specific TalonFX connection feedback |
| `deployMotorPositionDegrees` | `armPositionDegrees` | Actual arm angle reported by hardware |
| `IntakeConstants` gear/target values | Same numeric values | Calibration preserved despite renamed fields |
| `IndexerGoal goal` | `IndexerOperatingMode requestedIndexerMode` | Feeding, idling, or reversing intent |
| `ShooterGoal goal` | `ShooterOperatingMode operatingMode` | Shooter scoring/passing/tuning state |
| `Superstructure.Goal goal` | `Superstructure.RobotOperatingMode requestedRobotMode` | Coordinated scoring system behavior |
| `RobotContainer.drive` | `RobotContainer.swerveDrive` | Chassis system, not an individual motor |
| `RobotContainer.operatorController` | `operatorGamepad` | Physical input device |
| `Drive.modules` | `swerveModules` | Four independently steered drive modules |
| `Drive.gyroInputs` | `gyroSensorReadings` | Actual heading/gyro feedback |
| `Vision.consumer` | `submitVisionPoseMeasurement` | Callback that receives an accepted vision pose |
| `camera0Name` / `camera1Name` / `camera2Name` | `leftCameraName` / `rightCameraName` / `rearCameraName` | Physical names, checked against configured `Left-Camera`, `Right-Camera`, `Back-Camera` |
| `climberMotorConstants` | `ClimberMotorConstants` | Java type with standard UpperCamelCase capitalization |
| `FuelSim.FIELD_COR` | `FIELD_COLLISION_BOUNCE_FACTOR` | Collision coefficient of restitution |

### Naming rules

- **Verbs** name actions and commands: `setArmTargetAngle`, `stopArmMotor`, `refreshAllianceHubTarget`.
- **Adjectives or state words** name positions or observed conditions: `DEPLOYED`, `RETRACTED`, `isArmDeployed`.
- **requested / target** means intent. **commanded** means a chosen motor instruction. **measured / sensorReadings** means hardware feedback.
- Use mechanism nouns (`roller`, `arm`, `hood`, `flywheel`, `indexer`) rather than applying `mode`, `motor`, or `deploy` indiscriminately.
- Do not rename vendor/WPILib methods or CTRE-generated calibration variables merely for stylistic consistency. Vendor-generated code and library APIs have external contracts.

### Telemetry compatibility

Automatic AdvantageKit field/enum names may differ from the previous robot log. **Literal** string keys passed to logging and other APIs were held constant. If a dashboard, AdvantageScope template, or log processor expects fields by the old automatically generated names, update or alias that external consumer before use.

## Second-pass classroom readability refinements

| Before this pass | After this pass | Why |
|---|---|---|
| `f(distance, speed, hood)` | `addShotSpeedAndHoodCalibration(distanceMeters, flywheelSpeedRps, hoodAngleDegrees)` | Says exactly what the data point represents |
| `t(distance, time)` | `addFlightTimeCalibration(distanceMeters, flightTimeSeconds)` | Makes units and purpose clear |
| `i(distance, speed)` | `addIndexerFeedSpeedCalibration(distanceMeters, indexerSpeedRps)` | Avoids one-letter method names |
| `shooterAcel` | `measuredFlywheelAcceleration` | Corrects the abbreviation and marks feedback as measured |
| `shooterAmps`, `shooterVolts` | `measuredFlywheelCurrent`, `measuredFlywheelVoltage` | Names both the mechanism and kind of measured quantity |
| `followRequest`, `followRequest2`, `followRequest3` | `flywheelFollower1ControlRequest`, etc. | Identifies which follower controller each request belongs to |
| `pv`, `mm` | `hoodPositionVoltageRequest`, `motionMagicHoodPositionRequest` | Describes the vendor control request objects |

The final student version also contains the `STUDENT MAP` and `STUDENT NOTE` comments and fixes several inaccurate historical comments.

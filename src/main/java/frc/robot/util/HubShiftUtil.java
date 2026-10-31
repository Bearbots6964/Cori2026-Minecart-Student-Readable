// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;


import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.AutoConstants.ACTIVE_PRESHOOT_TIME;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import java.util.function.Supplier;

// FILE GUIDE: Match-timing helper for which hub is active during scheduled scoring windows.
public class HubShiftUtil {

    // STUDENT MAP: HUB ACTIVITY TIMING
    // PURPOSE: Determines which alliance hub is active for the current portion of a match.
    // LOOK FOR: The returned shift information describes match state; it is not a physical sensor input.
    public enum ShiftEnum {
        TRANSITION,
        SHIFT1,
        SHIFT2,
        SHIFT3,
        SHIFT4,
        ENDGAME,
        AUTO,
        DISABLED;
    }

    public record ShiftInfo(ShiftEnum currentShift, double elapsedTime, double remainingTime, boolean active) {}

    private static Timer hubShiftTimer = new Timer();
    private static final ShiftEnum[] shiftPatterns = ShiftEnum.values();

    private static final double[] shiftStartingTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
    private static final double[] shiftEndingTimes = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

    private static final double approachingActiveFudge = -ACTIVE_PRESHOOT_TIME.in(Seconds);

    public static final double autoEndTime = 20.0;
    public static final double teleopDuration = 140.0;
    private static final boolean[] activeSchedule = {true, true, false, true, false, true};
    private static final boolean[] inactiveSchedule = {true, false, true, false, true, true};

    private static Supplier<Optional<Boolean>> winningAllianceOverride = Optional::empty;

    // READ THIS: Read an optional test override for the alliance choice.
    public static Optional<Boolean> getAllianceWinOverride() {
        return winningAllianceOverride.get();
    }

    // READ THIS: Supply an optional test override for which alliance won.
    public static void setAllianceWinOverride(Supplier<Optional<Boolean>> winningAllianceOverride) {
        HubShiftUtil.winningAllianceOverride = winningAllianceOverride;
    }

    // STUDENT NOTE: Select which alliance hub is active first according to match information.
    public static Alliance getFirstActiveAlliance() {
        var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        // Return override value
        var winOverride = getAllianceWinOverride();
        if (winOverride.isPresent()) {
            return winOverride.get()
                ? (alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue)
                : (alliance == Alliance.Blue ? Alliance.Blue : Alliance.Red);
        }

        // Return FMS value
        String message = DriverStation.getGameSpecificMessage();
        if (message.length() > 0) {
            char character = message.charAt(0);
            if (character == 'R') {
                return Alliance.Blue;
            } else if (character == 'B') {
                return Alliance.Red;
            }
        }

        // Return default value
        return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
    }

    /** Starts the timer at the begining of teleop. */
    public static void initialize() {
        hubShiftTimer.restart();
    }

    // READ THIS: Build or read the activity schedule for the two hubs.
    private static boolean[] getSchedule(boolean override) {
        boolean[] currentSchedule;
        Alliance startAlliance = getFirstActiveAlliance();
        boolean activeFirst = startAlliance == DriverStation.getAlliance().orElse(Alliance.Blue);
        if (override) {
            activeFirst = !activeFirst;
        }
        currentSchedule = activeFirst ? activeSchedule : inactiveSchedule;
        return currentSchedule;
    }

    // READ THIS: Build or read the activity schedule for the two hubs.
    private static boolean[] getSchedule() {
        return getSchedule(false);
    }

    // READ THIS: Determine the current scoring phase and time remaining from a schedule.
    private static ShiftInfo getShiftInfo(boolean[] currentSchedule, double[] shiftStartingTimes, double[] shiftEndingTimes) {
        double currentTime = hubShiftTimer.get();
        double stateTimeElapsed = hubShiftTimer.get();
        double stateTimeRemaining = 0.0;
        boolean active = false;
        ShiftEnum currentShift = ShiftEnum.DISABLED;

        if (DriverStation.isAutonomousEnabled()) {
            stateTimeElapsed = currentTime;
            stateTimeRemaining = autoEndTime - currentTime;
            active = true;
            currentShift = ShiftEnum.AUTO;
        } else if (DriverStation.isEnabled()) {
            int currentShiftIndex = -1;
            for (int i = 0; i < shiftStartingTimes.length; i++) {
                if (currentTime >= shiftStartingTimes[i] && currentTime < shiftEndingTimes[i]) {
                    currentShiftIndex = i;
                    break;
                }
            }
            if (currentShiftIndex < 0) {

                // After last shift, so assume endgame
                currentShiftIndex = shiftStartingTimes.length - 1;
            }

            // Calculate elapsed and remaining time in the current shift, ignoring combined shifts
            stateTimeElapsed = currentTime - shiftStartingTimes[currentShiftIndex];
            stateTimeRemaining = shiftEndingTimes[currentShiftIndex] - currentTime;

            // If the state is the same as the last shift, combine the elapsed time
            if (currentShiftIndex > 0) {
                if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex - 1]) {
                    stateTimeElapsed = currentTime - shiftStartingTimes[currentShiftIndex - 1];
                }
            }

            // If the state is the same as the next shift, combine the remaining time
            if (currentShiftIndex < shiftEndingTimes.length - 1) {
                if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex + 1]) {
                    stateTimeRemaining = shiftEndingTimes[currentShiftIndex + 1] - currentTime;
                }
            }

            active = currentSchedule[currentShiftIndex];
            currentShift = shiftPatterns[currentShiftIndex];
        }
        ShiftInfo shiftInfo = new ShiftInfo(currentShift, stateTimeElapsed, stateTimeRemaining, active);
        return shiftInfo;
    }

    // STUDENT NOTE: Report current active hub and shift timing using official match timing.
    public static ShiftInfo getOfficialShiftInfo() {
        return getShiftInfo(getSchedule(), shiftStartingTimes, shiftEndingTimes);
    }

    // STUDENT NOTE: Report shift information accounting for the optional override.
    public static ShiftInfo getShiftedShiftInfo(boolean override) {
        boolean[] shiftSchedule = getSchedule(override);

        // Starting active
        if (shiftSchedule[1] == true) {
            double[] shiftedShiftStartTimes = {
                0.0, 10.0, 35.0, 60.0 + approachingActiveFudge, 85.0, 110.0 + approachingActiveFudge
            };
            double[] shiftedShiftEndTimes = {
                10.0, 35.0, 60.0 + approachingActiveFudge, 85.0, 110.0 + approachingActiveFudge, 140.0
            };
            return getShiftInfo(shiftSchedule, shiftedShiftStartTimes, shiftedShiftEndTimes);
        }
        double[] shiftedShiftStartTimes = {
            0.0, 10.0, 35.0 + approachingActiveFudge, 60.0, 85.0 + approachingActiveFudge, 110.0
        };
        double[] shiftedShiftEndTimes = {
            10.0, 35.0 + approachingActiveFudge, 60.0, 85.0 + approachingActiveFudge, 110.0, 140.0
        };
        return getShiftInfo(shiftSchedule, shiftedShiftStartTimes, shiftedShiftEndTimes);
    }

    // STUDENT NOTE: Report shift information accounting for the optional override.
    public static ShiftInfo getShiftedShiftInfo() {
        return getShiftedShiftInfo(false);
    }
}

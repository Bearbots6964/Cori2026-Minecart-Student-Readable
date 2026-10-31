// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;


import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {

    // STUDENT MAP: SHARED SETUP VALUES
    // PURPOSE: Stores robot-wide settings such as which environment is running and selected physical dimensions.
    // LOOK FOR: A constant is a named value; changing one can change robot behavior.
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final class ControllerPreferences {
        public static final double countdownRumbleIntensity = 0.35;
        public static final double hubActiveRumbleIntensity = 1.0;
        public static final double hubInactiveRumbleIntensity = 0.25;
        public static final double endgameRumbleIntensity = 1.0;
    }

    public static final class ShotCalculationConstants {
        public static final InterpolatingDoubleTreeMap distanceToShotSpeed = new InterpolatingDoubleTreeMap();

        // TODO -- populate with real values
        public static final Transform3d BALL_TRANSFORM_LEFT = new Transform3d(-0.24, 0.09, 0.5, Rotation3d.kZero);
        public static final Transform3d BALL_TRANSFORM_CENTER = new Transform3d(-0.24, 0, 0.5, Rotation3d.kZero);
        public static final Transform3d BALL_TRANSFORM_RIGHT = new Transform3d(-0.24, -0.09, 0.5, Rotation3d.kZero);
    }

    public static final class Dimensions {

        // frame to edge of bumper
        public static final Distance BUMPER_THICKNESS = Inches.of(3.625);

        // height from floor to top of bumper
        public static final Distance BUMPER_HEIGHT = Inches.of(6);

        // left to right (y-axis)
        public static final Distance FRAME_SIZE_Y = Inches.of(27.5);

        // front to back (x-axis)
        public static final Distance FRAME_SIZE_X = Inches.of(27.5);

        public static final Distance FULL_WIDTH = FRAME_SIZE_Y.plus(BUMPER_THICKNESS.times(2));
        public static final Distance FULL_LENGTH = FRAME_SIZE_X.plus(BUMPER_THICKNESS.times(2));
    }

    public static final class AutoConstants {

        // TODO: get real values
        public static final Rotation2d[] BUMP_TRAVERSAL_HEADINGS = new Rotation2d[] {
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(90),
            Rotation2d.fromDegrees(180),
            Rotation2d.fromDegrees(270)
        };

        // TODO: get real value
        // m/s
        public static final double BUMP_TRAVERSAL_SPEED = 1.5;

        // m/s
        public static final double SLOW_SPEED = 1.5;

        public static final Time ACTIVE_PRESHOOT_TIME = Units.Seconds.of(2);
    }

}

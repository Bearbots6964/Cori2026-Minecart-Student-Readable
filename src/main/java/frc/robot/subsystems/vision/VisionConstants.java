// Copyright 2021-2025 FRC 6328
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

package frc.robot.subsystems.vision;


import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

// FILE GUIDE: Camera locations, AprilTag layout, and thresholds used to judge vision data quality.
public class VisionConstants {

    // STUDENT MAP: CAMERA LOCATIONS AND FILTERING LIMITS
    // PURPOSE: Records where cameras are mounted and how uncertain camera estimates are treated.
    // LOOK FOR: These are calibration values, not live readings.

    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Physical camera names, matching the Left-Camera, Right-Camera, and Back-Camera devices.
    public static String leftCameraName = "Left-Camera";
    public static String rightCameraName = "Right-Camera";
    public static String rearCameraName = "Back-Camera";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToLeftCamera = new Transform3d(Inches.of(-13.541109), Inches.of(11.816561), Inches.of(11.807330+3.702225), new Rotation3d(Degrees.of(-3.925087), Degrees.of(-15.0), Degrees.of(90.0)));
    public static Transform3d robotToRightCamera = new Transform3d(Inches.of(-13.541109), Inches.of(-11.816561), Inches.of(11.807330+3.702225), new Rotation3d(Degrees.of(3.925087), Degrees.of(-15.0), Degrees.of(-90.0)));
    public static Transform3d robotToRearCamera = new Transform3d(Inches.of(-12.952218), Inches.of(0.484892), Inches.of(5.220422+3.702225), new Rotation3d(Degrees.of(0.0), Degrees.of(-15.0), Degrees.of(180.0)));

    public static Transform3d turretToLimelight = new Transform3d(Units.inchesToMeters(-7.773307), 0.0, Units.inchesToMeters(4.728530), new Rotation3d(0, Units.degreesToRadians(-30.0), Units.degreesToRadians(-180.0)));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    // Meters
    public static double linearStdDevBaseline = 0.0662390368;

    // Radians
    public static double angularStdDevBaseline = 0.06;

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {

        // Camera 0
        1.0,

        // Camera 1
        1.0
    };

    // Multipliers to apply for MegaTag 2 observations
    // More stable than full 3D solve
    public static double linearStdDevMegatag2Factor = 0.5;

    // No rotation data available
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;
}

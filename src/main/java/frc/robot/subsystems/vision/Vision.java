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


import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

// FILE GUIDE: Camera observation processing: rejects implausible poses and passes accepted measurements to drive localization.
public class Vision extends SubsystemBase {

    // STUDENT MAP: CAMERA POSE VALIDATION
    // PURPOSE: Reviews camera estimates, rejects implausible ones, and forwards accepted robot positions to the drive estimator.
    // LOOK FOR: An accepted camera estimate helps locate the robot; it does not directly turn the wheels.
    private final VisionConsumer submitVisionPoseMeasurement;
    private final VisionIO[] cameras;
    private final VisionIOInputsAutoLogged[] cameraSensorReadings;
    private final Alert[] cameraDisconnectedAlerts;

    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();
    List<Pose3d> tagPoses = new LinkedList<>();
    List<Pose3d> robotPoses = new LinkedList<>();
    List<Pose3d> robotPosesAccepted = new LinkedList<>();
    List<Pose3d> robotPosesRejected = new LinkedList<>();

    // READ THIS: Connect camera implementations to a callback that accepts localization measurements.
    public Vision(VisionConsumer submitVisionPoseMeasurement, VisionIO... cameras) {
        this.submitVisionPoseMeasurement = submitVisionPoseMeasurement;
        this.cameras = cameras;

        // Initialize inputs
        this.cameraSensorReadings = new VisionIOInputsAutoLogged[cameras.length];
        for (int i = 0; i < cameraSensorReadings.length; i++) {
            cameraSensorReadings[i] = new VisionIOInputsAutoLogged();
        }

        // Initialize disconnected alerts
        this.cameraDisconnectedAlerts = new Alert[cameras.length];
        for (int i = 0; i < cameraSensorReadings.length; i++) {
            cameraDisconnectedAlerts[i] =
                    new Alert("Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
        }
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    // STUDENT NOTE: Return a camera-reported horizontal target offset.
    public Rotation2d getTargetX(int cameraIndex) {
        return cameraSensorReadings[cameraIndex].latestTargetObservation.tx();
    }

    /**
     * Read camera observations. Reject poses that fail quality/field checks;
     * submit accepted poses and uncertainty estimates to the drivetrain.
     */
    @Override

    // STUDENT NOTE: Collect new camera estimates, reject bad data, and send accepted estimates to the drive estimator.
    public void periodic() {
        for (int i = 0; i < cameras.length; i++) {
            cameras[i].updateInputs(cameraSensorReadings[i]);
            Logger.processInputs("Vision/Camera" + Integer.toString(i), cameraSensorReadings[i]);
        }

        // Initialize logging values
        allTagPoses.clear();
        allRobotPoses.clear();
        allRobotPosesAccepted.clear();
        allRobotPosesRejected.clear();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < cameras.length; cameraIndex++) {

            // Update disconnected alert
            cameraDisconnectedAlerts[cameraIndex].set(!cameraSensorReadings[cameraIndex].connected);

            // Initialize logging values
            tagPoses.clear();
            robotPoses.clear();
            robotPosesAccepted.clear();
            robotPosesRejected.clear();

            // Add tag poses
            for (int tagId : cameraSensorReadings[cameraIndex].tagIds) {
                var tagPose = aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : cameraSensorReadings[cameraIndex].poseObservations) {

                // Check whether to reject pose
                // Must have at least one tag
                boolean rejectPose = observation.tagCount() == 0
                        || (observation.tagCount() == 1

                                // Cannot be high ambiguity
                                && observation.ambiguity() > maxAmbiguity)

                        // Must have realistic Z coordinate
                        || Math.abs(observation.pose().getZ()) > maxZError

                        // Must be within the field boundaries
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX() > aprilTagLayout.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > aprilTagLayout.getFieldWidth();

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // A longer-distance or lower-tag-count observation is treated
                // as less certain. The drivetrain uses these uncertainties when
                // combining camera data with wheel/gyro odometry.
                // Calculate standard deviations
                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = linearStdDevBaseline * stdDevFactor;
                double angularStdDev = angularStdDevBaseline * stdDevFactor;
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= linearStdDevMegatag2Factor;
                    angularStdDev *= angularStdDevMegatag2Factor;
                }
                if (cameraIndex < cameraStdDevFactors.length) {
                    linearStdDev *= cameraStdDevFactors[cameraIndex];
                    angularStdDev *= cameraStdDevFactors[cameraIndex];
                }

                // Send vision observation
                submitVisionPoseMeasurement.addVisionMeasurement(
                        observation.pose().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }

            // Log camera datadata
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
                    tagPoses.toArray(new Pose3d[0]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
                    robotPoses.toArray(new Pose3d[0]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[0]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[0]));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesAccepted",
                allRobotPosesAccepted.toArray(new Pose3d[0]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesRejected",
                allRobotPosesRejected.toArray(new Pose3d[0]));
    }

    @FunctionalInterface
    public interface VisionConsumer {
        void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }
}

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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {

    // STUDENT MAP: REAL PHOTONVISION CAMERA ADAPTER
    // PURPOSE: Reads camera results and translates them into the VisionIO measurement format.
    // LOOK FOR: PhotonVision supplies the detection results; this class converts and forwards them.
    protected final PhotonCamera photonCamera;
    protected final Transform3d robotToCamera;

    /**
     * Creates a new VisionIOPhotonVision.
     *
     * @param name The configured name of the photonCamera.
     * @param robotToCamera The 3D position of the photonCamera relative to the robot.
     */
    public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
        photonCamera = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
    }

    // READ THIS: Read available sensor signals into the measured-inputs object.
    @Override
    public void updateInputs(VisionIOInputs measuredInputs) {
        measuredInputs.connected = photonCamera.isConnected();

        // Read new camera observations
        Set<Short> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();
        for (var result : photonCamera.getAllUnreadResults()) {

            // Update latest target observation
            if (result.hasTargets()) {
                measuredInputs.latestTargetObservation = new TargetObservation(
                        Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                        Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
            } else {
                measuredInputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
            }

            // Add pose observation
            // Multitag result
            if (result.multitagResult.isPresent()) {
                var multitagResult = result.multitagResult.get();

                // Calculate robot pose
                Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                // Calculate average tag distance
                double totalTagDistance = 0.0;
                for (var target : result.targets) {
                    totalTagDistance +=
                            target.bestCameraToTarget.getTranslation().getNorm();
                }

                // Add tag IDs
                tagIds.addAll(multitagResult.fiducialIDsUsed);

                // Add observation
                poseObservations.add(new PoseObservation(

                        // Timestamp
                        result.getTimestampSeconds(),

                        // 3D pose estimate
                        robotPose,

                        // Ambiguity
                        multitagResult.estimatedPose.ambiguity,

                        // Tag count
                        multitagResult.fiducialIDsUsed.size(),

                        // Average tag distance
                        totalTagDistance / result.targets.size(),

                        // Observation type
                        PoseObservationType.PHOTONVISION));

            // Single tag result
            } else if (!result.targets.isEmpty()) {
                var target = result.targets.get(0);

                // Calculate robot pose
                var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
                if (tagPose.isPresent()) {
                    Transform3d fieldToTarget = new Transform3d(
                            tagPose.get().getTranslation(), tagPose.get().getRotation());
                    Transform3d cameraToTarget = target.bestCameraToTarget;
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                    // Add tag ID
                    tagIds.add((short) target.fiducialId);

                    // Add observation
                    poseObservations.add(new PoseObservation(

                            // Timestamp
                            result.getTimestampSeconds(),

                            // 3D pose estimate
                            robotPose,

                            // Ambiguity
                            target.poseAmbiguity,

                            // Tag count
                            1,

                            // Average tag distance
                            cameraToTarget.getTranslation().getNorm(),

                            // Observation type
                            PoseObservationType.PHOTONVISION));
                }
            }
        }

        // Save pose observations to inputs object
        measuredInputs.poseObservations = new PoseObservation[poseObservations.size()];
        for (int i = 0; i < poseObservations.size(); i++) {
            measuredInputs.poseObservations[i] = poseObservations.get(i);
        }

        // Save tag IDs to inputs objects
        measuredInputs.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
            measuredInputs.tagIds[i++] = id;
        }
    }
}

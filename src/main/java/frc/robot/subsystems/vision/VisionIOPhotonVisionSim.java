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


import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {

    // STUDENT MAP: SIMULATED PHOTONVISION CAMERA
    // PURPOSE: Models what a camera might see at a supplied simulated robot pose.
    // LOOK FOR: Simulation-only measurements are not evidence of real camera calibration.
    private static VisionSystemSim visionSimulation;

    private final Supplier<Pose2d> poseSupplier;
    private final PhotonCameraSim simulatedCamera;

    /**
     * Creates a new VisionIOPhotonVisionSim.
     *
     * @param name The name of the camera.
     * @param poseSupplier Supplier for the robot pose to use in simulation.
     */
    public VisionIOPhotonVisionSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
        super(name, robotToCamera);
        this.poseSupplier = poseSupplier;

        // Initialize vision sim
        if (visionSimulation == null) {
            visionSimulation = new VisionSystemSim("main");
            visionSimulation.addAprilTags(aprilTagLayout);
        }

        // Add sim camera
        var cameraProperties = new SimCameraProperties();
        simulatedCamera = new PhotonCameraSim(photonCamera, cameraProperties);
        visionSimulation.addCamera(simulatedCamera, robotToCamera);
    }

    // READ THIS: Read available sensor signals into the measured-inputs object.
    @Override
    public void updateInputs(VisionIOInputs measuredInputs) {
        visionSimulation.update(poseSupplier.get());
        super.updateInputs(measuredInputs);
    }
}

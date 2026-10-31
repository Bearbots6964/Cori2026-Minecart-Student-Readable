package frc.robot.subsystems.drive;


import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.util.Units;
import frc.robot.util.PhoenixUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;

// FILE GUIDE: Simulation gyro adapter: provides simulated heading and angular velocity rather than reading a physical sensor.
public class GyroIOSim implements GyroIO {

    // STUDENT MAP: SIMULATED DIRECTION SENSOR
    // PURPOSE: Reads simulated gyro heading and turning rate instead of a physical gyro.
    // LOOK FOR: Used during physics simulation, not the actual competition robot.
    private final GyroSimulation simulatedGyro;

    // READ THIS: Save the simulated gyro model used to produce measurements.
    public GyroIOSim(GyroSimulation simulatedGyro) {
        this.simulatedGyro = simulatedGyro;
    }

    // READ THIS: Read available sensor signals into the measured-inputs object.
    @Override
    public void updateInputs(GyroIOInputs measuredInputs) {
        measuredInputs.connected = true;
        measuredInputs.yawPosition = simulatedGyro.getGyroReading();
        measuredInputs.yawVelocityRadPerSec = Units.degreesToRadians(
                simulatedGyro.getMeasuredAngularVelocity().in(RadiansPerSecond));

        measuredInputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
        measuredInputs.odometryYawPositions = simulatedGyro.getCachedGyroReadings();
    }
}

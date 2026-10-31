package frc.robot.subsystems.indexer;


import org.littletonrobotics.junction.AutoLog;

// FILE GUIDE: Indexer hardware contract: measured motor data comes in; stop, duty cycle, voltage, and speed commands go out.
public interface IndexerIO {

    // STUDENT MAP: INDEXER HARDWARE INTERFACE
    // PURPOSE: Lists the data and control requests the Indexer subsystem uses.
    // LOOK FOR: An interface states what a motor controller must do; it is not the controller itself.
    @AutoLog
    public class IndexerIOInputs {
        public boolean indexerLeaderMotorConnected = false;
        public boolean followerMotorConnected = false;
        public double indexerSpeedRps = 0.0;
        public double indexerCurrentAmps = 0.0;
        public double indexerAppliedVoltage = 0.0;

    }

    // READ THIS: Read available sensor signals into the measured-inputs object.
    public default void updateInputs(IndexerIOInputs measuredInputs) {}

    // READ THIS: Send an indexer voltage request to its motor controller.
    public default void setIndexerVoltage(double volts) {}

    // READ THIS: Send an open-loop indexer percentage to its motor controller.
    public default void setIndexerDutyCycle(double input) {}

    // READ THIS: Request a target indexer wheel speed in rotations per second.
    public default void setIndexerSpeedRps(double velocity) {}

    // READ THIS: Send the stop request through the hardware interface.
    public default void stop() {}

}

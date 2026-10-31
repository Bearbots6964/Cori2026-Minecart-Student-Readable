package frc.robot.subsystems.shooter;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

// FILE GUIDE: Shooter hardware contract: reports sensor measurements and accepts flywheel and hood commands.
public interface ShooterIO {

    // STUDENT MAP: SHOOTER HARDWARE INTERFACE
    // PURPOSE: Lists the sensor measurements and motor instructions expected by Shooter.java.
    // LOOK FOR: The implementation in ShooterIOTalonFX.java communicates with the real electronics.
    @AutoLog
    public class ShooterIOInputs {
        public boolean flywheelLeaderConnected = false;
        public boolean flywheelFollower1Connected = false;
        public boolean flywheelFollower2Connected = false;
        public boolean flywheelFollower3Connected = false;
        public boolean hoodMotorConnected = false;

        public Voltage flywheelLeaderAppliedVoltage = Volts.zero();
        public Current flywheelLeaderMeasuredCurrent = Amps.zero();
        public AngularVelocity flywheelLeaderMeasuredSpeed = RotationsPerSecond.zero();
        public AngularAcceleration flywheelLeaderMeasuredAcceleration = RotationsPerSecondPerSecond.zero();

        public Voltage hoodMotorVoltage = Volts.zero();
        public Current hoodMotorCurrent = Amps.zero();
        public AngularVelocity hoodMotorAngularVelocity = RotationsPerSecond.zero();
        public Angle hoodPosition = Rotations.zero();
    }

    // READ THIS: Read available sensor signals into the measured-inputs object.
    public default void updateInputs(ShooterIOInputs measuredInputs) {}

    // READ THIS: Send a flywheel speed request to its motor controller.
    public default void setFlywheelSpeed(AngularVelocity velocity) {}

    // READ THIS: Send a hood position request to its motor controller.
    public default void setHoodPosition(Angle position) {}

    // READ THIS: Ask the flywheel motor hardware to stop.
    public default void stopFlywheel() {}

    // READ THIS: Ask the hood motor hardware to stop.
    public default void stopHood() {}

    // READ THIS: Send updated feedback-control gains for the flywheel.
    public default void setFlywheelPIDConstants(double kP, double kD, double kV, double kS) {}

    // READ THIS: Send updated feedback-control gains for the hood.
    public default void setHoodPIDConstants(double kP, double kD, double kV, double kS) {}

    // READ THIS: Send a flywheel voltage request to its motor controller.
    public default void setFlywheelVoltage(Voltage volts) {}

    // READ THIS: Send a hood voltage request to its motor controller.
    public default void setHoodVoltage(Voltage volts) {}
}

package frc.robot.subsystems.intake;


import edu.wpi.first.units.Units;
import java.util.function.BooleanSupplier;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

// FILE GUIDE: Simulated intake hardware adapter: supplies simulated or placeholder inputs behind the same interface.
public class IntakeIOSim implements IntakeIO {

    // STUDENT MAP: INTAKE SIMULATION ADAPTER
    // PURPOSE: Supplies an implementation of the intake hardware interface for simulation.
    // LOOK FOR: Only implemented methods simulate effects; default empty methods do not simulate moving hardware.
    private final IntakeSimulation rollerSimulation;
    private final SwerveDriveSimulation swerveDriveSimulation;

    // READ THIS: Set up the simulated intake hardware interface.
    public IntakeIOSim(SwerveDriveSimulation swerveDriveSimulation, BooleanSupplier customIntakeCondition) {
        this.swerveDriveSimulation = swerveDriveSimulation;
        this.rollerSimulation = IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            swerveDriveSimulation,
            Units.Inches.of(25.5),
            Units.Inches.of(5.375),
            IntakeSide.FRONT,
            3
        );
        this.rollerSimulation.setCustomIntakeCondition(gp -> customIntakeCondition.getAsBoolean());
    }

}
